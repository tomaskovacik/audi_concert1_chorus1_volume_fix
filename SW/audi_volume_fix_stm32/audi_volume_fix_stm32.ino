// HWv5 — dual-SPI man-in-the-middle bridge for Audi Concert1/Chorus1 volume fix
// STM32 intercepts SPI between radio MCU and front panel, forwarding and modifying packets.
// Define HWV5_BRIDGE to activate bridge mode (active); omit for passive sniff mode.
//
// Bridge pin assignment:
//   MCU side  (SPI1 slave):  PA5=SCK, PA7=MOSI, PA15=STATUS(OUTPUT), PA3=CS
//   Panel side (SPI2 master): PB13=SCK, PB14=MISO, PB15=MOSI, PB1=STATUS(INPUT), PA8=CS
//   GALA: PB5 (TIM3_CH2, speed pulse from HC05 pin23 via 1kΩ — optional)

/* version 5 — bridge mode (replaces passive sniffer with active man-in-the-middle)
mcuCLK    = PA5  (SPI1_SCK  - input, MCU drives CLK)
mcuDATA   = PA7  (SPI1_MOSI - data received by STM32 in slave mode)
mcuSTATUS = PA15 (OUTPUT: STM32 drives STATUS to MCU, acting as front panel)
mcuCS     = PA3  (output: hw-inverted PA15, mirrors SPI1_NSS on board)
panelCLK    = PB13 (SPI2_SCK  output to panel)
panelMOSI   = PB15 (SPI2_MOSI output to panel)
panelMISO   = PB14 (SPI2_MISO input from panel)
panelSTATUS = PB1  (input: panel drives STATUS)
panelCS     = PA8  (output: hw-inverted PB1)
*/
#define HWV5_BRIDGE
#include <SPI.h>
#include <Wire.h>
#include <FlexWire.h>
#include "audi_concert_panel.h"

FlexWire SWire = FlexWire(PB11, PB10);

#ifdef HWV5_BRIDGE
SPIClass SPI_2(PB15, PB14, PB13);  // SPI2: PB13=SCK, PB14=MISO, PB15=MOSI (panel side master)
#endif

#define USE_SERIAL
#define USEDSERIAL Serial1

// ── Config flash storage ───────────────────────────────────────────────────
// STM32F103C6 = 32KB flash. Reserve the last 1KB page for config.
// We only write on explicit serial command so no wear-levelling needed.
#define CFG_FLASH_PAGE  ((uint32_t)(0x08000000u + 32u*1024u - 1u*1024u))  // 0x8007C00
#define CFG_MAGIC0  0xA5u
#define CFG_MAGIC1  0x5Au
#define CFG_MAGIC2  0xC3u


#define DEFAULT_VOL   3
#define DEFAULT_GALA  0   // GALA off by default
#define DEFAULT_TA    3

// GALA speed input pin (PB5 = TIM3_CH2, tied with PB4/TIM3_CH1 — parallel with HC05 pin23, BC558 PNP high-side driver via 1kΩ)
#define GALA_PIN  PB5
/*
    SPI communication between motorola MC68HC05B32 cpu to front panel ST6280
    basics:
    SPI MODE 0

    CLK idle high, while
    STATUS idle high, it's like CS but it's triggered by slave here)
    DATA idle high

    communication principle:
    CLK line going LOW is signaling someone wanna talk
    whatever it's slave (panel, cose we have buttons there ...) or master
    after CLK is low, slave bring STATE line LOW, and when he is ready to
    receive data he will bring STATE HIGH, then master will clock out data
    or clock in data, after everything is transmitted, CLOCK goes HIGH and
    shortly after this STATE goes HIGH signaling end of packet.
    While CLK is still low after STATE goes LOW, signaling there is probably
    more data to come.

*/
#define mcuCLK    PA5  // SPI1_SCK  - MCU drives CLK (input, passive sniff)
#define mcuDATA   PA7  // SPI1_MOSI - shared DATA via resistors (input, passive sniff)
#define mcuCS     PA3  // output: mirrors STATUS → HW inverter → SPI1_NSS (PA4)
#define mcuSTATUS PA15
#ifdef HWV5_BRIDGE
// Panel side (SPI2 master) — STM32 acts as Audi MCU toward front panel
#define panelCLK    PB13  // SPI2_SCK  - output to panel
#define panelMOSI   PB15  // SPI2_MOSI - output to panel (10k resistor in HW)
#define panelMISO   PB14  // SPI2_MISO - input from panel (1k resistor in HW)
#define panelSTATUS PB1   // STATUS from panel (input, panel drives this)
#define panelCS     PA8   // CS to panel: output = !PB1 (hw wired to PB12 for inversion)
#endif
#define displayRESET PB8
#define VERSION "2.1-25.05.26-HWv5"

//we are trying to interface with communication to TDA7342 which is 0x44 so...
#define I2C_7BITADDR 0x44
//array is flat circular buffer of howmanypackets * howmanybytesinpacket bytes
#define howmanypackets 20
#define howmanybytesinpacket 15
#define PACKET_IDX(pkt, byte) ((pkt) * howmanybytesinpacket + (byte))

const byte MY_ADDRESS = I2C_7BITADDR;

/*
   CircularPacketBuffer - wraps a flat byte array as a circular queue of fixed-size
   packets (howmanypackets x howmanybytesinpacket).

   read(dst)  - copy current read packet into dst[] and advance rdp
   write(val) - append val to the current write packet at wbp and advance wbp
   commit()   - zero-fill remaining bytes in write packet then advance wdp
   available()- true when at least one unread packet is waiting
   busy       - set by ISR while actively writing a packet; main loop must not
                read while busy is set
*/
struct CircularPacketBuffer {
  volatile uint8_t *buf;
  volatile uint8_t rdp;  // read packet pointer
  volatile uint8_t wdp;  // write packet pointer
  volatile uint8_t wbp;  // write byte pointer within current write packet
  volatile bool busy;    // ISR-active flag

  bool available() const { return rdp != wdp; }

  // Copy the current read packet into dst[] then advance rdp.
  // No zero-fill needed — commit() zero-fills on write.
  void read(uint8_t dst[]) {
    for (uint8_t i = 0; i < howmanybytesinpacket; i++)
      dst[i] = buf[PACKET_IDX(rdp, i)];
    if (++rdp == howmanypackets) rdp = 0;
  }

  // Append val at wbp in the current write packet and advance wbp
  void write(uint8_t val) {
    buf[PACKET_IDX(wdp, wbp)] = val;
    wbp++;
  }

  // Zero-fill remaining bytes in current write packet then advance wdp
  // Silently drops the packet if the buffer is full (wdp would lap rdp).
  void commit() {
    while (wbp < howmanybytesinpacket) buf[PACKET_IDX(wdp, wbp++)] = 0;
    wbp = 0;
    uint8_t next = wdp + 1;
    if (next == howmanypackets) next = 0;
    if (next != rdp) wdp = next; // drop silently if full
  }
};

/*
   Flat backing arrays for the two circular packet buffers.
   Access them only through the panel_message / i2c_data helper objects below.
*/
volatile uint8_t _panel_msg_buf[howmanypackets * howmanybytesinpacket]; // SPI front-panel data
volatile uint8_t _i2c_data_buf[howmanypackets * howmanybytesinpacket];  // I2C data from MCU

CircularPacketBuffer panel_message = { _panel_msg_buf, 0, 0, 0, false }; // SPI front panel messages
CircularPacketBuffer i2c_data      = { _i2c_data_buf,  0, 0, 0, false }; // I2C packets from MCU

#ifdef HWV5_BRIDGE
// ── Bridge circular buffers (SPI1↔SPI2 man-in-the-middle) ────────────────────────
#define CIRC_BUF_SIZE 256
struct CircBuf {
    volatile uint8_t  data[CIRC_BUF_SIZE];
    volatile uint16_t head = 0, tail = 0;
    void push(uint8_t b) { uint16_t n=(head+1)%CIRC_BUF_SIZE; if(n!=tail){data[head]=b;head=n;} }
    bool pop(uint8_t &b) { if(head==tail)return false; b=data[tail]; tail=(tail+1)%CIRC_BUF_SIZE; return true; }
    bool empty() const { return head==tail; }
    bool full()  const { return ((head+1)%CIRC_BUF_SIZE)==tail; }
};
CircBuf mcu_to_panel;  // SPI1 RX → SPI2 TX  (MCU→panel display data)
CircBuf panel_to_mcu;  // SPI2 RX → SPI1 TX  (panel→MCU button data)

// Direct packet array for MCU→panel data (populated by SPI1 ISR)
volatile uint8_t _msg[howmanypackets][howmanybytesinpacket];
volatile uint8_t dwdp = 0;   // write packet pointer (ISR)
volatile uint8_t dwbp = 0;   // write byte pointer (ISR)
volatile uint8_t drdp = 0;   // read packet pointer (loop)
volatile uint8_t grabbing_SPI = 0;  // set by ISR during active byte capture

// Panel→MCU packet mirror array
volatile uint8_t _panel_msg[howmanypackets][howmanybytesinpacket];
volatile uint8_t panel_dwdp = 0, panel_dwbp = 0, panel_drdp = 0;
volatile uint8_t panel_grabbing_SPI = 0;

// Set by panelStatusChange ISR when panel wants to send button data
volatile uint8_t panel_wants_to_send = 0;
// Set in loop() when we signal MCU that panel has button data via STATUS LOW
volatile uint8_t mcu_read_initiated = 0;
#endif

Config cfg;  // loaded once in setup(), updated on serial command

volatile uint8_t volume         = 0xBA;
volatile uint8_t current_volume = 0xBA;
volatile uint8_t saved_volume   = 0xBA;

volatile uint8_t loudness         = 0x0E;
volatile uint8_t current_loudness = 0x0E;

volatile bool grab_volume    = true;
volatile bool mute           = false;
volatile bool in_volume_recalc = false;

// GALA state
volatile uint16_t gala_captime = 0;  // pulse width in µs (filled by ISR)
uint16_t gala_prev_speed = 0;
// Rolling average over last 8 pulses to suppress ±1 µs jitter
#define GALA_AVG_N 8
static uint16_t gala_pulse_buf[GALA_AVG_N];
static uint8_t  gala_pulse_idx = 0;
static bool     gala_buf_full  = false;

bool displayRESETstate = false;

uint8_t volume_packet[howmanybytesinpacket];
uint8_t loudness_packet[howmanybytesinpacket];


void sendI2C(const uint8_t data[howmanybytesinpacket]);


void set_volume();
void set_loudness();

void set_mute();
void set_unmute();
void receiveEvent(int howMany);
#ifdef HWV5_BRIDGE
void mcuClkChange();
void spi1ByteReceived();
void panelStatusChange();
void receivePanelPacket();
void sendToPanel();
#endif

// ── Config (raw flash, 1KB page) ──────────────────────────────────────────

static Config readConfig() {
  const Config *p = (const Config *)CFG_FLASH_PAGE;
  Config c = *p;
  if (c.magic[0] == CFG_MAGIC0 && c.magic[1] == CFG_MAGIC1 && c.magic[2] == CFG_MAGIC2
      && c.crc == (uint8_t)(c.vol + c.gala + c.ta)) {
    // CRC ok — clamp each field to valid range (guard against partial flash corruption)
    if (c.vol  < 1 || c.vol  > 5) c.vol  = DEFAULT_VOL;
    if (c.gala > 5)                c.gala = DEFAULT_GALA;
    if (c.ta   < 1 || c.ta   > 5) c.ta   = DEFAULT_TA;
    return c;
  }
  // magic/CRC mismatch — full defaults
  c.magic[0] = CFG_MAGIC0; c.magic[1] = CFG_MAGIC1; c.magic[2] = CFG_MAGIC2;
  c.vol  = DEFAULT_VOL;
  c.gala = DEFAULT_GALA;
  c.ta   = DEFAULT_TA;
  c.crc  = c.vol + c.gala + c.ta;
  return c;
}

static bool writeConfig(Config c) {
  c.magic[0] = CFG_MAGIC0; c.magic[1] = CFG_MAGIC1; c.magic[2] = CFG_MAGIC2;
  c.crc = c.vol + c.gala + c.ta;
  HAL_FLASH_Unlock();
  FLASH_EraseInitTypeDef eraseInit;
  eraseInit.TypeErase   = FLASH_TYPEERASE_PAGES;
  eraseInit.PageAddress = CFG_FLASH_PAGE;
  eraseInit.NbPages     = 1;
  uint32_t pageError    = 0;
  HAL_FLASHEx_Erase(&eraseInit, &pageError);
  const uint16_t *src_words = (const uint16_t *)&c;
  uint32_t flash_addr = CFG_FLASH_PAGE;
  for (uint8_t i = 0; i < (sizeof(Config) + 1) / 2; i++, flash_addr += 2)
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, flash_addr, (uint64_t)src_words[i]) != HAL_OK) {
      HAL_FLASH_Lock();
#ifdef USE_SERIAL
      USEDSERIAL.print(F("CFG_SAVE: FAIL vol=")); USEDSERIAL.print(c.vol);
      USEDSERIAL.print(F(" gala="));              USEDSERIAL.print(c.gala);
      USEDSERIAL.print(F(" ta="));                USEDSERIAL.println(c.ta);
#endif
      return false;
    }
  HAL_FLASH_Lock();
#ifdef USE_SERIAL
  USEDSERIAL.print(F("CFG_SAVE: OK vol=")); USEDSERIAL.print(c.vol);
  USEDSERIAL.print(F(" gala="));            USEDSERIAL.print(c.gala);
  USEDSERIAL.print(F(" ta="));              USEDSERIAL.println(c.ta);
#endif
  return true;
}

static uint8_t volLevelToHex(uint8_t level) {
  // levels 1-5 → volume hex (higher = quieter on TDA7342)
  static const uint8_t tbl[] = { 0xC2, 0xBE, 0xBA, 0xB6, 0xB2 };
  if (level < 1) level = 1;
  if (level > 5) level = 5;
  return tbl[level - 1];
}

// ── GALA ISRs ─────────────────────────────────────────────────────────────

void galaRising() {
  TIM2->CNT = 0;
  TIM2->CR1 |= TIM_CR1_CEN;
  attachInterrupt(digitalPinToInterrupt(GALA_PIN), galaFalling, FALLING);
}

void galaFalling() {
  TIM2->CR1 &= ~TIM_CR1_CEN;
  gala_captime = (uint16_t)TIM2->CNT;
  attachInterrupt(digitalPinToInterrupt(GALA_PIN), galaRising, RISING);
}

void setup ()
{
  // Load config from flash and apply start volume
  cfg = readConfig();
  uint8_t start_vol = volLevelToHex(cfg.vol);
  volume = current_volume = saved_volume = start_vol;

  volume_packet[0] = 0x02;
  loudness_packet[0] = 0x02;
  volume_packet[1] = 0x02;
  loudness_packet[1] = 0x01;
  Wire.begin (MY_ADDRESS, false, false);
  Wire.onReceive (receiveEvent);
  SWire.begin();

  // Enable SPI1 and AFIO clocks
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_SPI1EN;
  // Disable JTAG, keep SWD (SWJ_CFG=010) → frees PB3/PB4/PA15 for GPIO; clear SPI1_REMAP → default pins
  AFIO->MAPR = (AFIO->MAPR & ~(AFIO_MAPR_SWJ_CFG | AFIO_MAPR_SPI1_REMAP)) | AFIO_MAPR_SWJ_CFG_JTAGDISABLE;

#ifdef HWV5_BRIDGE
  // ── Bridge mode: STM32 acts AS front panel to radio MCU ──────────────────────────
  // STM32 drives STATUS (PA15) as output; watches CLK (PA5) for byte framing.
  // SPI1 slave with software NSS + RXNE interrupt for byte capture.
  pinMode(mcuCLK, INPUT_PULLUP);   // PA5: SCK from MCU (input)
  attachInterrupt(digitalPinToInterrupt(mcuCLK), mcuClkChange, CHANGE);
  pinMode(mcuDATA, INPUT_PULLUP);  // PA7: MOSI from MCU (input)
  pinMode(mcuSTATUS, OUTPUT);      // PA15: STM32 drives STATUS to MCU (output)
  digitalWrite(mcuSTATUS, HIGH);   // idle HIGH
  pinMode(mcuCS, OUTPUT);          // PA3: hardware-inverted STATUS → PA4/NSS
  digitalWrite(mcuCS, LOW);        // STATUS HIGH → CS LOW
  // SPI1 slave: call SPI.begin() to configure AF pins, then patch to slave mode
  SPI.begin();
  SPI1->CR1 &= ~SPI_CR1_SPE;
  SPI1->CR1 &= ~SPI_CR1_MSTR;
  SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI; // software NSS, always selected; CPOL=0, CPHA=0
  SPI1->CR1 |= SPI_CR1_SPE;
  SPI1->DR   = 0x00;                       // pre-load MISO idle value
  SPI1->CR2 |= SPI_CR2_RXNEIE;            // fire SPI1_IRQHandler on each received byte
  NVIC_EnableIRQ(SPI1_IRQn);

  // ── Bridge mode: STM32 acts AS radio MCU to front panel ──────────────────────────
  // Panel drives STATUS; STM32 drives CLK (PB13) between packets as GPIO HIGH.
  // SPI2 is only activated inside sendToPanel()/receivePanelPacket().
  pinMode(panelCLK, OUTPUT);
  digitalWrite(panelCLK, HIGH);      // CLK idle HIGH = no transfer in progress
  pinMode(panelSTATUS, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(panelSTATUS), panelStatusChange, CHANGE);
  pinMode(panelCS, OUTPUT);
  digitalWrite(panelCS, LOW);        // STATUS idle HIGH → CS LOW
#else
  // ── Sniff mode: passive monitoring of MCU↔panel SPI bus ─────────────────────────
  // STATUS ISR gates NSS (PA3→PA4/NSS) to frame each byte.
  pinMode(mcuSTATUS, INPUT_PULLUP);
  pinMode(mcuCS, OUTPUT);
  digitalWrite(mcuCS, !digitalRead(mcuSTATUS));
  attachInterrupt(digitalPinToInterrupt(mcuSTATUS), mcuStatusChange, CHANGE);

  // PA4/NSS, PA5/SCK, PA7/MOSI must be INPUT before enabling SPI1
  pinMode(PA4, INPUT);  // NSS driven via PA3→inverter
  pinMode(PA5, INPUT);
  pinMode(PA7, INPUT);

  // SPI1 slave: SSM=0 → hardware NSS (PA4).  NSS LOW = selected = shift reg active.
  SPI1->CR1 = 0;  // MSTR=0, SSM=0, CPOL=0, CPHA=0
  SPI1->CR1 |= SPI_CR1_SPE;
  SPI1->CR2 = 0;  // no interrupts; loop() polls SR
#endif

  pinMode(displayRESET, INPUT);

  // GALA: TIM2 at 1µs resolution for speed pulse measurement
  if (cfg.gala > 0) {
    pinMode(GALA_PIN, INPUT_PULLDOWN);
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 71;       // 72MHz / (71+1) = 1MHz = 1µs ticks
    TIM2->ARR = 0xFFFF;
    TIM2->CNT = 0;
    TIM2->CR1 = 0;        // stopped
    TIM2->EGR = TIM_EGR_UG;  // force update to load PSC/ARR
    attachInterrupt(digitalPinToInterrupt(GALA_PIN), galaRising, RISING);
  }

#ifdef USE_SERIAL
  USEDSERIAL.begin(115200);
  printInfo();
#endif

  set_volume();
}  // end of setup

void printInfo() {
  USEDSERIAL.print(F("Firmware version: "));
  USEDSERIAL.println(F(VERSION));
  USEDSERIAL.println(F("(C) kovo, GPL3"));
  USEDSERIAL.println(F("https://www.tindie.com/products/tomaskovacik/volume-fix-for-audi-concert1chorus1/"));
  USEDSERIAL.println(F("https://github.com/tomaskovacik/audi_concert1_chorus1_volume_fix"));
  // CFG_LOAD: config loaded from flash at startup
  USEDSERIAL.print(F("CFG_LOAD: vol=")); USEDSERIAL.print(cfg.vol);
  USEDSERIAL.print(F(" gala="));        USEDSERIAL.print(cfg.gala);
  USEDSERIAL.print(F(" ta="));          USEDSERIAL.println(cfg.ta);
  if (cfg.gala > 0) {
    USEDSERIAL.print(F("CFG_LOAD: GALA base_thr="));
    USEDSERIAL.print(100 - (cfg.gala - 1) * 15);
    USEDSERIAL.println(F(" km/h"));
  } else {
    USEDSERIAL.println(F("CFG_LOAD: GALA disabled (send g1..g5 to enable)"));
  }
}

void loop()
{
#ifdef USE_SERIAL
  if (USEDSERIAL.available()) {
    char ch = USEDSERIAL.read();
    bool changed = false;
    if (ch == 'v' || ch == 'V' || ch == 'h' || ch == 'H' || ch == '?') {
      printInfo();
    } else if (ch == 's' && USEDSERIAL.available()) {
      uint8_t level = USEDSERIAL.read() - '0';
      if (level >= 1 && level <= 5) { cfg.vol = level; changed = true; }
    } else if (ch == 'g' && USEDSERIAL.available()) {
      uint8_t level = USEDSERIAL.read() - '0';
      if (level <= 5) { cfg.gala = level; changed = true; }
    }
    if (changed) {
      if (writeConfig(cfg)) {
        USEDSERIAL.println(F("Config saved. Restart to apply."));
      } else {
        USEDSERIAL.println(F("Config write FAILED."));
      }
      printInfo();
    }
  }
#endif

  // displayRESET edge: reload start volume from config when radio panel wakes up
  bool reset_high = digitalRead(displayRESET);
  if (reset_high && !displayRESETstate) {
    displayRESETstate = true;
    uint8_t start_vol = volLevelToHex(cfg.vol);
    volume = current_volume = saved_volume = start_vol;
#ifdef USE_SERIAL
    USEDSERIAL.println(F("Reset HIGH — reloaded start volume"));
#endif
  }
  if (!reset_high && displayRESETstate) {
    displayRESETstate = false;
  }

#ifdef HWV5_BRIDGE
  // ── SPI bridge forwarding ────────────────────────────────────────────────────────
  // Panel-initiated button press: panelStatusChange ISR detected STATUS FALLING
  // with CLK HIGH and already drove CLK LOW. Complete byte exchange here.
  if (panel_wants_to_send && !panel_grabbing_SPI) {
    receivePanelPacket();
  }

  // MCU→panel display forward: send queued bytes to panel via SPI2.
  if (!panel_grabbing_SPI && !mcu_to_panel.empty()) {
    sendToPanel();
  }

  // Panel→MCU button forwarding: signal MCU (drive STATUS LOW while CLK HIGH),
  // then SPI1 clocks out the pre-loaded button byte via MISO.
  if (!grabbing_SPI && !mcu_read_initiated && !panel_to_mcu.empty()) {
    uint8_t tx;
    if (panel_to_mcu.pop(tx)) {
      noInterrupts();
      SPI1->DR = tx;
      mcu_read_initiated = 1;
      digitalWrite(mcuCS, HIGH);
      digitalWrite(mcuSTATUS, LOW);   // "panel has data" interrupt to MCU
      interrupts();
      uint32_t t = micros();
      while (!grabbing_SPI && (micros() - t) < 5000);
      if (!grabbing_SPI) {
        mcu_read_initiated = 0;
        SPI1->DR = 0x00;
        digitalWrite(mcuSTATUS, HIGH);
        digitalWrite(mcuCS, LOW);
      }
    }
  }
#endif

#ifdef HWV5_BRIDGE
  if (!grabbing_SPI) {
    while (drdp != dwdp) {
      uint8_t _data[howmanybytesinpacket];
      for (uint8_t i = 0; i < howmanybytesinpacket; i++) _data[i] = _msg[drdp][i];
#ifdef USE_SERIAL
      if (_data[0] == 0x25) {
        USEDSERIAL.print(F("BTN "));
        USEDSERIAL.println(_data[1], HEX);
      }
#endif
      if (_data[0] == 0x25) {
        if (grab_volume && (_data[1] == PANEL_KNOB_UP || _data[1] == PANEL_REMOTE_VOLUME_UP)) {
          set_volume_up();
          set_volume();
        }
        if (grab_volume && (_data[1] == PANEL_KNOB_DOWN || _data[1] == PANEL_REMOTE_VOLUME_DOWN)) {
          set_volume_down();
          set_volume();
        }
      }
      if (_data[0] == 0x9A) {
        decode_display_data(_data);
      }
      drdp++; if (drdp == howmanypackets) drdp = 0;
    }
  }
#else
  if (!panel_message.busy) {
    while (panel_message.available()) {
      uint8_t _data[howmanybytesinpacket];
      panel_message.read(_data);
#ifdef USE_SERIAL
      if (_data[0] == 0x25) {
        USEDSERIAL.print(F("BTN "));
        USEDSERIAL.println(_data[1], HEX);
      }
#endif
      if (_data[0] == 0x25) {
        if (grab_volume && (_data[1] == PANEL_KNOB_UP || _data[1] == PANEL_REMOTE_VOLUME_UP)) {
          set_volume_up();
          set_volume();
        }
        if (grab_volume && (_data[1] == PANEL_KNOB_DOWN || _data[1] == PANEL_REMOTE_VOLUME_DOWN)) {
          set_volume_down();
          set_volume();
        }
      }
      if (_data[0] == 0x9A) {
        decode_display_data(_data);
      }

    }
  }
#endif
  if (!i2c_data.busy) {
    while (i2c_data.available()) {
      uint8_t _data[howmanybytesinpacket];
      i2c_data.read(_data);
#ifdef USE_SERIAL
      // incoming I2C from radio (HC05 → TDA7342) — disabled, use TDA for outgoing
      // USEDSERIAL.print(F("I2C"));
      // for (uint8_t i = 0; i < howmanybytesinpacket; i++) {
      //   USEDSERIAL.print(' '); USEDSERIAL.print(_data[i], HEX);
      // }
      // USEDSERIAL.println();
#endif
      if ((_data[1] & 0x0f) == 1 || (_data[1] & 0x0F) == 2) {
        // volume/loudness packet from panel — ignore, we control volume ourselves
      } else if (_data[1] == 8) { // MUTE
        if ((_data[2] & B00000001)) {
          if (!mute && !in_volume_recalc) {
            mute = true;
            saved_volume = current_volume;
            volume = 0xFF;
            set_volume();
          }
          sendI2C(_data);
        } else {
          if (mute) {
            mute = false;
            volume = saved_volume;
            set_volume();
          }
          sendI2C(_data);
        }
      } else {
        sendI2C(_data);
      }
    }
  }

  // ── GALA speed-based volume ──────────────────────────────────────────────
  if (cfg.gala > 0 && gala_captime > 0) {
    uint16_t pulse_width_us = gala_captime;
    gala_captime = 0;

    // Rolling average of last GALA_AVG_N pulse widths to suppress µs jitter
    gala_pulse_buf[gala_pulse_idx] = pulse_width_us;
    gala_pulse_idx = (gala_pulse_idx + 1) % GALA_AVG_N;
    if (gala_pulse_idx == 0) gala_buf_full = true;
    uint8_t n = gala_buf_full ? GALA_AVG_N : gala_pulse_idx;
    uint32_t sum = 0;
    for (uint8_t i = 0; i < n; i++) sum += gala_pulse_buf[i];
    uint16_t avg_pulse_us = (uint16_t)(sum / n);

    uint16_t speed_kmh = (uint16_t)(1000000UL / (2UL * avg_pulse_us));

    if (gala_prev_speed != speed_kmh) {
      // Speed threshold base and 30 km/h steps: vol up / loudness down as speed rises
      uint16_t base_speed_thr = (uint16_t)(100 - (cfg.gala - 1) * 15);
#ifdef USE_SERIAL
      USEDSERIAL.print(F("GALA_SPEED: ")); USEDSERIAL.print(gala_prev_speed);
      USEDSERIAL.print(F("->")); USEDSERIAL.print(speed_kmh);
      USEDSERIAL.print(F(" km/h pulse_us=")); USEDSERIAL.print(avg_pulse_us);
      USEDSERIAL.print(F(" base_thr=")); USEDSERIAL.println(base_speed_thr);
#endif

      // Going faster — step volume up and loudness down at each 30 km/h band
      for (uint8_t band = 0; band < 5; band++) {
        uint16_t vol_speed_thr  = base_speed_thr + band * 30;
        uint16_t loud_speed_thr = vol_speed_thr + 15;
        if (gala_prev_speed <= vol_speed_thr && vol_speed_thr < speed_kmh) {
          set_volume_up(); set_volume();
#ifdef USE_SERIAL
          USEDSERIAL.print(F("GALA_VOL: UP band=")); USEDSERIAL.print(band);
          USEDSERIAL.print(F(" thr=")); USEDSERIAL.print(vol_speed_thr);
          USEDSERIAL.print(F(" vol=0x")); USEDSERIAL.println(volume, HEX);
#endif
        }
        if (gala_prev_speed <= loud_speed_thr && loud_speed_thr < speed_kmh && loudness > 0x06) {
          loudness--; current_loudness = loudness + 1; set_loudness();
#ifdef USE_SERIAL
          USEDSERIAL.print(F("GALA_LOUD: DOWN band=")); USEDSERIAL.print(band);
          USEDSERIAL.print(F(" thr=")); USEDSERIAL.print(loud_speed_thr);
          USEDSERIAL.print(F(" loud=0x")); USEDSERIAL.println(loudness, HEX);
#endif
        }
        // Slowing down — step volume down and loudness up
        if (speed_kmh < vol_speed_thr && vol_speed_thr <= gala_prev_speed) {
          set_volume_down(); set_volume();
#ifdef USE_SERIAL
          USEDSERIAL.print(F("GALA_VOL: DOWN band=")); USEDSERIAL.print(band);
          USEDSERIAL.print(F(" thr=")); USEDSERIAL.print(vol_speed_thr);
          USEDSERIAL.print(F(" vol=0x")); USEDSERIAL.println(volume, HEX);
#endif
        }
        if (speed_kmh < loud_speed_thr && loud_speed_thr <= gala_prev_speed && loudness < 0x0E) {
          loudness++; current_loudness = loudness - 1; set_loudness();
#ifdef USE_SERIAL
          USEDSERIAL.print(F("GALA_LOUD: UP band=")); USEDSERIAL.print(band);
          USEDSERIAL.print(F(" thr=")); USEDSERIAL.print(loud_speed_thr);
          USEDSERIAL.print(F(" loud=0x")); USEDSERIAL.println(loudness, HEX);
#endif
        }
      }
    }
    gala_prev_speed = speed_kmh;
  }
}

void set_mute() {
  if (!mute) {
    mute = true;
    static const uint8_t mute_data[howmanybytesinpacket] = {0x02, 0x08, 0x81, 0};
    sendI2C(mute_data);
  }
}
void set_unmute() {
  if (mute) {
    mute = false;
    static const uint8_t unmute_data[howmanybytesinpacket] = {0x02, 0x08, 0x80, 0};
    sendI2C(unmute_data);
  }
}

// Volume levels for up (louder) steps — descending order, 0xFF excluded
static const uint8_t vol_up_levels[] = {
  0xEA, 0xD2, 0xBA, 0xA2, 0x92, 0x82, 0x72, 0x66,
  0x5E, 0x5A, 0x56, 0x52, 0x4E, 0x4A, 0x46, 0x42,
  0x3E, 0x3A, 0x36, 0x32, 0x2E, 0x2A, 0x26, 0x22,
  0x1E, 0x1A, 0x16, 0x12, 0x10
};

// Volume levels for down (quieter) steps — descending order
static const uint8_t vol_down_levels[] = {
  0xFF, 0xEE, 0xD6, 0xBE, 0xA4, 0x94, 0x84, 0x74,
  0x68, 0x60, 0x5C, 0x58, 0x54, 0x50, 0x4C, 0x48,
  0x44, 0x40, 0x3C, 0x38, 0x34, 0x30, 0x2C, 0x28,
  0x24, 0x20, 0x1C, 0x18, 0x14
};

void set_volume_up() {
  in_volume_recalc = true;
  for (uint8_t i = 0; i < sizeof(vol_up_levels); i++) {
    if (volume > vol_up_levels[i]) {
      volume = vol_up_levels[i];
      return;
    }
  }
  volume = 0x10; // floor: loudest level seen in original comms
}

void set_volume_down() {
  in_volume_recalc = true;
  for (int8_t i = (int8_t)sizeof(vol_down_levels) - 1; i >= 0; i--) {
    if (volume < vol_down_levels[i]) {
      volume = vol_down_levels[i];
      return;
    }
  }
}

void set_volume() {
  if (volume == 0xFF && !mute) saved_volume = current_volume;

  while (volume != current_volume) {
    if (current_volume > volume)
      current_volume -= (current_volume - volume == 1) ? 1 : 2;
    else
      current_volume += (volume - current_volume == 1) ? 1 : 2;
    volume_packet[2] = current_volume;
    set_loudness();
    sendI2C(volume_packet);
  }

  if (volume == 0xFF) set_mute();
  if (volume < 0xFE) set_unmute();
  in_volume_recalc = false;
}

void set_loudness()
{
  // Linear formula: step of 4 from 0x5E downward, special case above 0x5E
  if (volume > 0x66) {
    loudness = 0x0E;
  } else if (volume > 0x5E) {
    loudness = 0x0D;
  } else {
    int8_t loud_calc = 0x0C - (int8_t)((0x5E - volume) / 4);
    loudness = (loud_calc < 0x06) ? 0x06 : (uint8_t)loud_calc;
  }
  while (current_loudness != loudness) {
    if (current_loudness < loudness) {
      loudness_packet[2] = ++current_loudness;
    }
    if (current_loudness > loudness) {
      loudness_packet[2] = --current_loudness;
    }
    sendI2C(loudness_packet);
  }
}

/*
   SPI packet structure (0x9A packets):
   byte 0: 0x9A (packet type)
   byte 1: subtype:
     0x48: ASCII display text
     0x13: button/mode LEDs (bytes 2-4: mode flags)
     0x32: AM/FM frequency
     0xA2: CD changer (byte 2: disc, byte 3: track)
     0x23: display clear
     0x61: tape mode (byte 2: direction/fwd/rew/eject)
     0x58: settings menu — ASCII text (bytes 2-9), e.g. "VOL  3  ", "GALA 2  ", "GALA OFF"
     0x71: tone/balance menu (byte 2 upper nibble: BAS/TRE/BAL/FAD selector)
*/

void decode_display_data(uint8_t _data[howmanybytesinpacket]) {
  grab_volume = true;

  if (_data[1] == 0x58) {
    // Settings menu: suppress volume knob and auto-save config from display text
    grab_volume = false;

    // "VOL  X  " — start volume level 1-5 (bytes 2-9)
    if (_data[2]=='V' && _data[3]=='O' && _data[4]=='L' && _data[5]==' '
        && _data[6]==' ' && _data[8]==' ' && _data[9]==' ') {
      uint8_t level = _data[7] - '0';
      if (level >= 1 && level <= 5) {
        cfg.vol = level;
#ifdef USE_SERIAL
        USEDSERIAL.print(F("CFG_PANEL: vol=")); USEDSERIAL.println(cfg.vol);
#endif
        writeConfig(cfg);
      }
    }
    // "TA   X  " — TA level 1-5
    if (_data[2]=='T' && _data[3]=='A' && _data[4]==' ' && _data[5]==' '
        && _data[6]==' ' && _data[8]==' ' && _data[9]==' ') {
      uint8_t level = _data[7] - '0';
      if (level >= 1 && level <= 5) {
        cfg.ta = level;
#ifdef USE_SERIAL
        USEDSERIAL.print(F("CFG_PANEL: ta=")); USEDSERIAL.println(cfg.ta);
#endif
        writeConfig(cfg);
      }
    }
    // "GALA OFF" or "GALA X  " — GALA level 0-5
    if (_data[2]=='G' && _data[3]=='A' && _data[4]=='L' && _data[5]=='A' && _data[6]==' ') {
      if (_data[7]=='O' && _data[8]=='F' && _data[9]=='F') {
        cfg.gala = 0;
#ifdef USE_SERIAL
        USEDSERIAL.println(F("CFG_PANEL: gala=0 (OFF)"));
#endif
        writeConfig(cfg);
      } else if (_data[8]==' ' && _data[9]==' ') {
        uint8_t level = _data[7] - '0';
        if (level >= 1 && level <= 5) {
          cfg.gala = level;
#ifdef USE_SERIAL
          USEDSERIAL.print(F("CFG_PANEL: gala=")); USEDSERIAL.println(cfg.gala);
#endif
          writeConfig(cfg);
        }
      }
    }
  }

  if (_data[1] == 0x71 && (_data[2] >> 4) <= 7) grab_volume = false; // BAS/TRE/BAL/FAD

#ifdef USE_SERIAL
  USEDSERIAL.print(F("SPI"));
  for (uint8_t i = 0; i < howmanybytesinpacket; i++) {
    USEDSERIAL.print(' '); USEDSERIAL.print(_data[i], HEX);
  }
  USEDSERIAL.println();
#endif
}

#ifndef HWV5_BRIDGE
// STATUS LOW  → NSS LOW  → SPI1 selected → counts 8 CLK edges → RXNE set.
// STATUS HIGH → NSS HIGH → SPI1 deselected → byte complete, read DR here in ISR.
void mcuStatusChange()
{
    if (digitalRead(mcuSTATUS)) {
        // STATUS RISING → deselect SPI1 (byte complete)
        digitalWrite(mcuCS, LOW);   // PA3 LOW → inverter → NSS HIGH

        // Harvest the completed byte immediately — before loop() gets a chance to run
        if (SPI1->SR & SPI_SR_RXNE) {
            uint8_t b = (uint8_t)SPI1->DR;
            panel_message.write(b);
            if (panel_message.wbp == howmanybytesinpacket)
                panel_message.wbp = 0; // overflow guard
        }

        if (digitalRead(mcuCLK)) {
            panel_message.commit();
            panel_message.busy = 0;
        }
        // else: CLK LOW = more bytes coming, busy stays 1
    } else {
        // STATUS FALLING → select SPI1 (new byte starting)
        digitalWrite(mcuCS, HIGH);  // PA3 HIGH → inverter → NSS LOW
        panel_message.busy = 1;
    }
}
#endif // !HWV5_BRIDGE

// called by interrupt service routine when incoming data arrives
void receiveEvent (int howMany)
{
  if (howMany <= 0) return;
  if (howMany >= howmanybytesinpacket) howMany = howmanybytesinpacket - 1;
  i2c_data.busy = 1;
  i2c_data.write((uint8_t)howMany);
  for (uint8_t i = 0; i < howMany; i++) {
    i2c_data.write(Wire.read());
  }
  i2c_data.commit();
  i2c_data.busy = 0;
}

void sendI2C (const uint8_t data[howmanybytesinpacket]) {
#ifdef USE_SERIAL
  USEDSERIAL.print(F("TDA"));
  for (uint8_t i = 1; i <= data[0]; i++) {
    USEDSERIAL.print(' '); USEDSERIAL.print(data[i], HEX);
  }
  USEDSERIAL.println();
#endif
  SWire.beginTransmission(MY_ADDRESS);

  for (byte i = 0 ; i < data[0]; i++) {
    while (!SWire.write(data[i + 1])) {}
  }
  SWire.endTransmission();
}

#ifdef HWV5_BRIDGE
// ═══════════════════════════════════════════════════════════════════════════════════════
// Bridge ISRs: MCU side (SPI1 slave) + Panel side (SPI2 master)
// ═══════════════════════════════════════════════════════════════════════════════════════

// ── MCU side: CLK-edge state machine ────────────────────────────────────────────────
// CLK FALLING: MCU starts a byte transfer. Mimic panel: ACK with STATUS LOW→HIGH.
// CLK RISING:  End of packet. Grab last byte, close packet, return STATUS to idle.
void mcuClkChange()
{
    if (!digitalRead(mcuCLK)) {
        // CLK LOW — MCU starts byte transfer
        grabbing_SPI = 1;
        digitalWrite(mcuCS, HIGH);
        digitalWrite(mcuSTATUS, LOW);
        digitalWrite(mcuSTATUS, HIGH);
        digitalWrite(mcuCS, LOW);
        // Flush any stale RX before new byte arrives
        if (SPI1->SR & SPI_SR_RXNE) (void)SPI1->DR;
    } else {
        // CLK HIGH — end of packet
        if (SPI1->SR & SPI_SR_RXNE) {
            uint8_t b = (uint8_t)SPI1->DR;
            if (!mcu_read_initiated) {
                _msg[dwdp][dwbp++] = b;
                mcu_to_panel.push(b);
            }
        }
        while (dwbp < howmanybytesinpacket) _msg[dwdp][dwbp++] = 0;
        dwbp = 0;
        dwdp++; if (dwdp == howmanypackets) dwdp = 0;
        grabbing_SPI = 0;
        mcu_read_initiated = 0;
        digitalWrite(mcuSTATUS, HIGH);
        digitalWrite(mcuCS, LOW);
    }
}

// SPI1 RXNE — byte fully clocked in from MCU mid-packet.
inline void spi1ByteReceived()
{
    uint8_t b = (uint8_t)SPI1->DR;
    if (!mcu_read_initiated) {
        _msg[dwdp][dwbp++] = b;
        if (dwbp == howmanybytesinpacket) dwbp = 0;
        mcu_to_panel.push(b);
    }
    uint8_t tx;
    SPI1->DR = panel_to_mcu.pop(tx) ? tx : 0x00;
    // STATUS LOW→HIGH = byte done, ready for next
    digitalWrite(mcuCS, HIGH);
    digitalWrite(mcuSTATUS, LOW);
    digitalWrite(mcuSTATUS, HIGH);
    digitalWrite(mcuCS, LOW);
}

extern "C" void SPI1_IRQHandler(void);
extern "C" void SPI1_IRQHandler(void)
{
    if (SPI1->SR & SPI_SR_RXNE) spi1ByteReceived();
}

// ── Panel side: STATUS-edge state machine ───────────────────────────────────────────
// STATUS FALLING while CLK HIGH  → panel has button data; drive CLK LOW to ack.
// STATUS FALLING while CLK LOW   → byte-done handshake (sendToPanel handles it).
// STATUS RISING                  → panel ready for next byte.
void panelStatusChange()
{
    if (digitalRead(panelSTATUS)) {
        digitalWrite(panelCS, LOW);   // STATUS HIGH → CS LOW
    } else {
        digitalWrite(panelCS, HIGH);  // STATUS LOW → CS HIGH
        if (digitalRead(panelCLK)) {
            // CLK idle (HIGH): panel "I have data" interrupt
            digitalWrite(panelCLK, LOW);  // ACK: drive CLK LOW
            panel_wants_to_send = 1;
        }
        // If CLK already LOW: byte-done during sendToPanel() — it handles it
    }
}

// ── Receive one button-press packet from panel via SPI2 ─────────────────────────────
// Called from loop() after panelStatusChange() set panel_wants_to_send.
// CLK is already LOW (driven in ISR). Bring SPI2 up; clock bytes until done.
void receivePanelPacket()
{
    panel_wants_to_send = 0;
    panel_grabbing_SPI  = 1;

    SPI_2.begin();
    SPI_2.setBitOrder(MSBFIRST);
    SPI_2.setDataMode(SPI_MODE0);

    uint32_t t;
    for (;;) {
        t = micros();
        while (!digitalRead(panelSTATUS) && (micros() - t) < 5000);
        if (!digitalRead(panelSTATUS)) break;  // timeout

        uint8_t rx = SPI_2.transfer(0x00);
        panel_to_mcu.push(rx);
        _panel_msg[panel_dwdp][panel_dwbp] = rx;
        if (++panel_dwbp == howmanybytesinpacket) panel_dwbp = 0;

        t = micros();
        while (digitalRead(panelSTATUS) && (micros() - t) < 5000);

        delayMicroseconds(200);
        if (!digitalRead(panelSTATUS)) break;  // STATUS still LOW → packet done
    }

    SPI_2.end();
    pinMode(panelCLK, OUTPUT);
    digitalWrite(panelCLK, HIGH);
    panel_grabbing_SPI = 0;

    while (panel_dwbp < howmanybytesinpacket) _panel_msg[panel_dwdp][panel_dwbp++] = 0;
    panel_dwbp = 0;
    panel_dwdp++; if (panel_dwdp == howmanypackets) panel_dwdp = 0;
}

// ── Transmit one display packet from mcu_to_panel buffer to panel via SPI2 ──────────
// SPI2 is brought up per-packet (CLK LOW = start-of-packet), then torn down (CLK HIGH).
void sendToPanel()
{
    if (mcu_to_panel.empty()) return;

    uint8_t pkt[howmanybytesinpacket];
    uint8_t len = 0;
    uint8_t b;
    while (len < howmanybytesinpacket && mcu_to_panel.pop(b)) pkt[len++] = b;
    if (!len) return;

    panel_grabbing_SPI = 1;

    SPI_2.begin();
    SPI_2.setBitOrder(MSBFIRST);
    SPI_2.setDataMode(SPI_MODE0);

    uint32_t t;
    for (uint8_t i = 0; i < len; i++) {
        t = micros();
        while (!digitalRead(panelSTATUS) && (micros()-t) < 5000);
        uint8_t rx = SPI_2.transfer(pkt[i]);
        panel_to_mcu.push(rx);
        t = micros();
        while (digitalRead(panelSTATUS) && (micros()-t) < 5000);
    }

    SPI_2.end();
    pinMode(panelCLK, OUTPUT);
    digitalWrite(panelCLK, HIGH);
    panel_grabbing_SPI = 0;

    while (panel_dwbp < howmanybytesinpacket) _panel_msg[panel_dwdp][panel_dwbp++] = 0;
    panel_dwbp = 0;
    panel_dwdp++; if (panel_dwdp == howmanypackets) panel_dwdp = 0;
}

#endif // HWV5_BRIDGE
