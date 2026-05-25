// HWv5 — passive SPI sniffer for Audi Concert1/Chorus1 volume fix
// module is held in reset state when front panel is off, no last volume is stored

/* version 5 — passive sniffer (same role as HWv1-v4, CLK/DATA via SPI1 peripheral)
mcuCLK    = PA5  (SPI1_SCK  - input, MCU drives CLK)
mcuDATA   = PA7  (SPI1_MOSI - input, shared DATA via resistors)
mcuSTATUS = PA15 (input, panel drives STATUS)
GALA      = PA0  (input, speed pulse from vehicle — optional feature)
*/
#include <SPI.h>
#include <Wire_slave.h>

#include <FlexWire.h>
#include "audi_concert_panel.h"

FlexWire SWire = FlexWire(PB11, PB10);

#define USE_SERIAL
#define USEDSERIAL Serial1

// ── Config flash storage ───────────────────────────────────────────────────
// STM32F103C6 = 32KB flash. Reserve the last 1KB page for config.
// We only write on explicit serial command so no wear-levelling needed.
#define CFG_FLASH_PAGE  ((uint32_t)(0x08000000u + 32u*1024u - 1u*1024u))  // 0x8007C00
#define CFG_MAGIC0  0xA5u
#define CFG_MAGIC1  0x5Au
#define CFG_MAGIC2  0xC3u

// Raw flash write/erase API (part of the EEPROM library in Roger Clark core)
#include "flash_stm32.h"

struct Config {
  uint8_t magic[3];  // CFG_MAGIC0/1/2
  uint8_t vol;       // start volume level 1-5  (1=quietest, 5=loudest default=3)
  uint8_t gala;      // GALA level: 0=off, 1-5 (5=most aggressive speed-volume)
  uint8_t ta;        // reserved (traffic announcement volume bump, future use)
  uint8_t crc;       // vol+gala+ta checksum
};

#define DEFAULT_VOL   3
#define DEFAULT_GALA  0   // GALA off by default
#define DEFAULT_TA    3

// GALA speed input pin (PA0 = free pin on HWv5)
#define GALA_PIN  PA0
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

volatile uint8_t start_volume = 0xBA;

volatile uint8_t volume = start_volume;
volatile uint8_t current_volume = start_volume;
volatile uint8_t saved_volume = start_volume;

volatile uint8_t start_loudness = 0x0E;

volatile uint8_t loudness = start_loudness;
volatile uint8_t current_loudness = start_loudness;

volatile uint8_t grab_volume = 1;

volatile bool mute = false;
volatile bool in_volume_recalc = false;

// GALA state
volatile uint16_t gala_captime = 0;  // pulse width in µs (filled by ISR)
uint16_t gala_prev_speed = 0;

// displayRESET edge tracking
uint8_t displayRESETstate = 0;

uint8_t volume_packet[howmanybytesinpacket];
uint8_t loudness_packet[howmanybytesinpacket];


void sendI2C(const uint8_t data[howmanybytesinpacket]);


void set_volume();
void set_loudness();

void set_mute();
void set_unmute();
void receiveEvent(int howMany);

// ── Config (raw flash, 1KB page) ──────────────────────────────────────────

static Config readConfig() {
  const Config *p = (const Config *)CFG_FLASH_PAGE;
  Config c = *p;
  if (c.magic[0] == CFG_MAGIC0 && c.magic[1] == CFG_MAGIC1 && c.magic[2] == CFG_MAGIC2
      && c.crc == (uint8_t)(c.vol + c.gala + c.ta))
    return c;
  // defaults
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
  FLASH_Unlock();
  FLASH_ErasePage(CFG_FLASH_PAGE);
  const uint16_t *src = (const uint16_t *)&c;
  uint32_t addr = CFG_FLASH_PAGE;
  for (uint8_t i = 0; i < (sizeof(Config) + 1) / 2; i++, addr += 2)
    if (FLASH_ProgramHalfWord(addr, src[i]) != FLASH_COMPLETE) { FLASH_Lock(); return false; }
  FLASH_Lock();
  return true;
}

static uint8_t volLevelToHex(uint8_t level) {
  // levels 1-5 → volume hex (higher = quieter on TDA7342)
  static const uint8_t tbl[] = { 0x56, 0x52, 0x4E, 0x4A, 0x46 };
  if (level < 1) level = 1;
  if (level > 5) level = 5;
  return tbl[level - 1];
}

// ── GALA ISRs ─────────────────────────────────────────────────────────────

void galaRising() {
  Timer2.setCount(0);
  Timer2.resume();
  attachInterrupt(digitalPinToInterrupt(GALA_PIN), galaFalling, FALLING);
}

void galaFalling() {
  Timer2.pause();
  gala_captime = Timer2.getCount();
  attachInterrupt(digitalPinToInterrupt(GALA_PIN), galaRising, RISING);
}

void setup ()
{
  // Load config from flash and apply start volume
  Config cfg = readConfig();
  start_volume = volLevelToHex(cfg.vol);
  volume = start_volume;
  current_volume = start_volume;
  saved_volume = start_volume;

  volume_packet[0] = 0x02;
  loudness_packet[0] = 0x02;
  volume_packet[1] = 0x02;
  loudness_packet[1] = 0x01;
  Wire.begin (MY_ADDRESS);
  Wire.onReceive (receiveEvent);
  SWire.begin();

  // STATUS ISR gates NSS (PA3→PA4/NSS) so
  pinMode(mcuSTATUS, INPUT_PULLUP);
  pinMode(mcuCS, OUTPUT);
  digitalWrite(mcuCS, !digitalRead(mcuSTATUS));
  attachInterrupt(digitalPinToInterrupt(mcuSTATUS), mcuStatusChange, CHANGE);

  // Enable SPI1 and AFIO clocks directly (SPI.begin() may target SPI2 on some cores).
  // RCC APB2ENR (0x40021018): bit 0 = AFIOEN, bit 12 = SPI1EN
  volatile uint32_t *rcc_apb2enr = (volatile uint32_t*)0x40021018;
  *rcc_apb2enr |= (1u << 0) | (1u << 12);
  // Clear AFIO SPI1_REMAP (bit 0 of AFIO_MAPR) → default pins PA4-PA7
  volatile uint32_t *afio_mapr = (volatile uint32_t*)0x40010004;
  *afio_mapr &= ~(1u << 0);

  // PA4/NSS, PA5/SCK, PA7/MOSI must be INPUT before enabling SPI1
  gpio_set_mode(GPIOA, 4, GPIO_INPUT_FLOATING);  // NSS driven via PA3→inverter
  gpio_set_mode(GPIOA, 5, GPIO_INPUT_FLOATING);
  gpio_set_mode(GPIOA, 7, GPIO_INPUT_FLOATING);

  // SPI1 slave: SSM=0 → hardware NSS (PA4).  NSS LOW = selected = shift reg active.
  // PA3 HIGH → inverter → PA4 LOW → NSS LOW → SPI1 counts 8 CLK edges → RXNE.
  // PA3 LOW  → inverter → PA4 HIGH → NSS HIGH → shift reg resets → byte framed.
  SPI1->regs->CR1 = 0;  // MSTR=0, SSM=0, CPOL=0, CPHA=0
  SPI1->regs->CR1 |= SPI_CR1_SPE;
  SPI1->regs->CR2 = 0;  // no interrupts; loop() polls SR

  pinMode(displayRESET, INPUT);

  // GALA: Timer2 at 1µs resolution for speed pulse measurement
  if (cfg.gala > 0) {
    pinMode(GALA_PIN, INPUT_PULLUP);
    Timer2.setPrescaleFactor(72);  // 72MHz / 72 = 1MHz = 1µs ticks
    Timer2.setOverflow(0xFFFF);
    Timer2.pause();
    Timer2.setCount(0);
    attachInterrupt(digitalPinToInterrupt(GALA_PIN), galaRising, RISING);
  }

#ifdef USE_SERIAL
  USEDSERIAL.begin(115200);
  printInfo();
#endif

  set_volume();
}  // end of setup

void printInfo() {
  Config cfg = readConfig();
  USEDSERIAL.print(F("Firmware version: "));
  USEDSERIAL.println(F(VERSION));
  USEDSERIAL.println(F("(C) kovo, GPL3"));
  USEDSERIAL.println(F("https://www.tindie.com/products/tomaskovacik/volume-fix-for-audi-concert1chorus1/"));
  USEDSERIAL.println(F("https://github.com/tomaskovacik/audi_concert1_chorus1_volume_fix"));
  USEDSERIAL.print(F("Start volume level (s1-s5): ")); USEDSERIAL.println(cfg.vol);
  USEDSERIAL.print(F("GALA level (g0-g5, 0=off):  ")); USEDSERIAL.println(cfg.gala);
  if (cfg.gala > 0) {
    USEDSERIAL.print(F("  GALA speed threshold: "));
    USEDSERIAL.print(100 - (cfg.gala - 1) * 15);
    USEDSERIAL.println(F(" km/h base"));
  }
}

void loop()
{
#ifdef USE_SERIAL
  if (USEDSERIAL.available()) {
    char ch = USEDSERIAL.read();
    Config cfg = readConfig();
    bool changed = false;
    if (ch == 'v' || ch == 'V' || ch == 'h' || ch == 'H' || ch == '?') {
      printInfo();
    } else if (ch == 's' && USEDSERIAL.available()) {
      uint8_t lvl = USEDSERIAL.read() - '0';
      if (lvl >= 1 && lvl <= 5) { cfg.vol = lvl; changed = true; }
    } else if (ch == 'g' && USEDSERIAL.available()) {
      uint8_t lvl = USEDSERIAL.read() - '0';
      if (lvl <= 5) { cfg.gala = lvl; changed = true; }
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
  uint8_t rst = digitalRead(displayRESET);
  if (rst && !displayRESETstate) {
    displayRESETstate = 1;
    Config cfg = readConfig();
    start_volume = volLevelToHex(cfg.vol);
    volume = start_volume;
    current_volume = start_volume;
    saved_volume = start_volume;
#ifdef USE_SERIAL
    USEDSERIAL.println(F("Reset HIGH — reloaded start volume"));
#endif
  }
  if (!rst && displayRESETstate) {
    displayRESETstate = 0;
  }

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
        if (grab_volume == 1 && (_data[1] == PANEL_KNOB_UP || _data[1] == PANEL_REMOTE_VOLUME_UP)) {
          set_volume_up();
          set_volume();
        }
        if (grab_volume == 1 && (_data[1] == PANEL_KNOB_DOWN || _data[1] == PANEL_REMOTE_VOLUME_DOWN)) {
          set_volume_down();
          set_volume();
        }
      }
      if (_data[0] == 0x9A) {
        decode_display_data(_data);
      }

    }
  }
  if (!i2c_data.busy) {
    while (i2c_data.available()) {
      uint8_t _data[howmanybytesinpacket];
      i2c_data.read(_data);
#ifdef USE_SERIAL
      USEDSERIAL.print(F("I2C"));
      for (uint8_t i = 0; i < howmanybytesinpacket; i++) {
        USEDSERIAL.print(' '); USEDSERIAL.print(_data[i], HEX);
      }
      USEDSERIAL.println();
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
  Config gala_cfg = readConfig();
  if (gala_cfg.gala > 0 && gala_captime > 0) {
    uint16_t ct = gala_captime;
    gala_captime = 0;
    uint16_t cur_speed = (uint16_t)(1000000UL / (2UL * ct));

    if (gala_prev_speed != cur_speed) {
      // Speed threshold base and 30 km/h steps: vol up / loudness down as speed rises
      uint16_t thr = (uint16_t)(100 - (gala_cfg.gala - 1) * 15);

      // Going faster — step volume up and loudness down at each 30 km/h band
      for (uint8_t step = 0; step < 5; step++) {
        uint16_t v_thr = thr + step * 30;
        uint16_t l_thr = v_thr + 15;
        if (gala_prev_speed <= v_thr && v_thr < cur_speed) {
          set_volume_up(); set_volume();
        }
        if (gala_prev_speed <= l_thr && l_thr < cur_speed && loudness > 0x06) {
          loudness--; current_loudness = loudness + 1; set_loudness();
        }
        // Slowing down — step volume down and loudness up
        if (cur_speed < v_thr && v_thr <= gala_prev_speed) {
          set_volume_down(); set_volume();
        }
        if (cur_speed < l_thr && l_thr <= gala_prev_speed && loudness < 0x0E) {
          loudness++; current_loudness = loudness - 1; set_loudness();
        }
      }
    }
    gala_prev_speed = cur_speed;
  }
}

/*
   send mute data over i2c
*/
void set_mute() {
  if (!mute) {
    mute = true;
    static const uint8_t mute_data[howmanybytesinpacket] = {0x02, 0x08, 0x81, 0};
    sendI2C(mute_data);
  }
}
/*
   send unmute data over i2c
*/
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
    int8_t l = 0x0C - (int8_t)((0x5E - volume) / 4);
    loudness = (l < 0x06) ? 0x06 : (uint8_t)l;
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

   decoding display data  parameter is array with data packet
   I try to send just pointer, cose data array is not local, we can access it everywhere
   but it will get 1% more of storage program and it's probably not faster ...

   packet struckture:
   1st byte in packet is packet definition or something,.... it's always 0x96
   2nd byte in packet identified data send in packet:
          - 0x48 - plain asci data to display
          - 0x13 - leds on buttons indicating mode/functions
                  -3th byte: [nan|I|I|I|FM|AS|RDS|REG]
                              - 5th bit is  "pipe" between FM lethers, which make AM symbol something like F|M
                              - 6th and 7th bits are same "pipe" which made FM 1 and FM 2 like FM I(I)
                  - 4th byte: [nan|nan|nan|RD|Dolby|CPS|presets|presets]
                              - 0th and 1st bit are for stations presets 1,2,3,4,5,6
                  - 5th byte: LEDS [nan|MODE|AS|SCAN|FM|TP|AM|RDS]

          - 0x32 AM/FM frequency display
                  - 3th byte:??
                  - 4th byte: actual freq:
                      AM mode: (531+(9*4th byte)) in kHz
                      FM mode: (875+4th byte)/10 in Mhz
          - 0xA2 CD changer mode
                 - 3th packet is CD number, in hex (but here it's not important, we have only 6CD)
                 - 4th packet is Track number, again in hex, but no ABCDEF is used ...
          - 0x23 - display clear.
          - 0x61 - TAPE mode (display shows TAPE)
                   3th byte: 1/2 indicate direction of playback (/\ or \/)
                             3/4 indicate fast forward or rewind (< or > )
                             0 indicate eject

*/

void decode_display_data(uint8_t _data[howmanybytesinpacket]) {
  grab_volume = 1;

  // grab_volume logic: suppress volume knob handling while panel shows
  // bass/treble/balance/fade/volume-setting menus
  if (_data[1] == 0x58) grab_volume = 0;                    // settings menu text
  if (_data[1] == 0x71 && (_data[2] >> 4) <= 7) grab_volume = 0; // BAS/TRE/BAL/FAD

#ifdef USE_SERIAL
  USEDSERIAL.print(F("SPI"));
  for (uint8_t i = 0; i < howmanybytesinpacket; i++) {
    USEDSERIAL.print(' '); USEDSERIAL.print(_data[i], HEX);
  }
  USEDSERIAL.println();
#endif
}

// STATUS LOW  → NSS LOW  → SPI1 selected → counts 8 CLK edges → RXNE set.
// STATUS HIGH → NSS HIGH → SPI1 deselected → byte complete, read DR here in ISR.
void mcuStatusChange()
{
    if (digitalRead(mcuSTATUS)) {
        // STATUS RISING → deselect SPI1 (byte complete)
        digitalWrite(mcuCS, LOW);   // PA3 LOW → inverter → NSS HIGH

        // Harvest the completed byte immediately — before loop() gets a chance to run
        if (SPI1->regs->SR & SPI_SR_RXNE) {
            uint8_t b = (uint8_t)SPI1->regs->DR;
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
  SWire.beginTransmission(MY_ADDRESS);

  for (byte i = 0 ; i < data[0]; i++) {
    while (!SWire.write(data[i + 1])) {}
  }
  SWire.endTransmission();
}
