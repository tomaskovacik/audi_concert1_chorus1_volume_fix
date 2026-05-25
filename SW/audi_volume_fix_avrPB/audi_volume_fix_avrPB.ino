// AVR port — passive SPI sniffer for Audi Concert1/Chorus1 volume fix
// Target: ATmega328P (Arduino Nano / Pro Mini 5V 16MHz)
//
// Pinout:
//   mcuSTATUS    = 2   (INT0, panel STATUS/CS line)
//   mcuCLK       = 13  (SPI SCK  — MCU clock line, input)
//   mcuDATA      = 11  (SPI MOSI — shared DATA, input)
//   mcuCS        = 10  (SPI SS, output — driven directly from STATUS ISR)
//   displayRESET = 8
//   I2C slave    = A4 (SDA), A5 (SCL) — hardware Wire, captures MCU→TDA7342 traffic
//   I2C master   = A2 (SDA), A3 (SCL) — SlowSoftWire, sends fixed data to TDA7342

#include <Wire.h>
#include <SlowSoftWire.h>
#include "audi_concert_panel.h"

SlowSoftWire SWire = SlowSoftWire(A2, A3);

#define USE_SERIAL
#define USEDSERIAL Serial

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
#define mcuSTATUS    2   // INT0
#define mcuCLK       13  // SPI SCK  (input, passive sniff)
#define mcuDATA      11  // SPI MOSI (input, passive sniff)
#define mcuCS        10  // SPI SS   (output, driven from STATUS ISR)
#define displayRESET 8
#define VERSION "2.0-25.05.26-AVR"

#define I2C_7BITADDR 0x44
#define howmanypackets       20
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
  volatile uint8_t rdp;
  volatile uint8_t wdp;
  volatile uint8_t wbp;
  volatile bool busy;

  bool available() const { return rdp != wdp; }

  void read(uint8_t dst[]) {
    for (uint8_t i = 0; i < howmanybytesinpacket; i++)
      dst[i] = buf[PACKET_IDX(rdp, i)];
    if (++rdp == howmanypackets) rdp = 0;
  }

  void write(uint8_t val) {
    buf[PACKET_IDX(wdp, wbp)] = val;
    wbp++;
  }

  void commit() {
    while (wbp < howmanybytesinpacket) buf[PACKET_IDX(wdp, wbp++)] = 0;
    wbp = 0;
    uint8_t next = wdp + 1;
    if (next == howmanypackets) next = 0;
    if (next != rdp) wdp = next;
  }
};

volatile uint8_t _panel_msg_buf[howmanypackets * howmanybytesinpacket];
volatile uint8_t _i2c_data_buf[howmanypackets * howmanybytesinpacket];

CircularPacketBuffer panel_message = { _panel_msg_buf, 0, 0, 0, false };
CircularPacketBuffer i2c_data      = { _i2c_data_buf,  0, 0, 0, false };

volatile uint8_t start_volume    = 0xBA;
volatile uint8_t volume          = 0xBA;
volatile uint8_t current_volume  = 0xBA;
volatile uint8_t saved_volume    = 0xBA;

volatile uint8_t start_loudness   = 0x0E;
volatile uint8_t loudness         = 0x0E;
volatile uint8_t current_loudness = 0x0E;

volatile uint8_t grab_volume    = 1;
volatile bool    mute           = false;
volatile bool    in_volume_recalc = false;

uint8_t volume_packet[howmanybytesinpacket];
uint8_t loudness_packet[howmanybytesinpacket];

uint8_t displayRESETstate = 0;

void sendI2C(const uint8_t data[howmanybytesinpacket]);
void set_volume();
void set_loudness();
void set_mute();
void set_unmute();
void receiveEvent(int howMany);

void setup()
{
  volume_packet[0]  = 0x02;
  loudness_packet[0] = 0x02;
  volume_packet[1]  = 0x02;
  loudness_packet[1] = 0x01;

  Wire.begin(MY_ADDRESS);
  Wire.onReceive(receiveEvent);
  SWire.begin();

  // STATUS ISR mirrors STATUS to mcuCS (SS pin) to gate AVR hardware SPI slave.
  // mcuCS (pin 10 / SS) configured as OUTPUT so the ISR can drive it directly;
  // the SPI peripheral still honours the pin voltage for slave-select control.
  pinMode(mcuSTATUS, INPUT_PULLUP);
  pinMode(mcuCS, OUTPUT);
  digitalWrite(mcuCS, HIGH);  // SS HIGH = deselected, matches STATUS idle HIGH
  attachInterrupt(digitalPinToInterrupt(mcuSTATUS), mcuStatusChange, CHANGE);

  // AVR SPI slave: SPE=1, MSTR=0, CPOL=0, CPHA=0 (SPI Mode 0)
  pinMode(mcuCLK,  INPUT);
  pinMode(mcuDATA, INPUT);
  SPCR = (1 << SPE);

  pinMode(displayRESET, INPUT);

#ifdef USE_SERIAL
  USEDSERIAL.begin(115200);
  printInfo();
#endif

  set_volume();
}

void printInfo() {
  USEDSERIAL.print(F("Firmware version: "));
  USEDSERIAL.println(F(VERSION));
  USEDSERIAL.println(F("(C) kovo, GPL3"));
  USEDSERIAL.println(F("https://www.tindie.com/products/tomaskovacik/volume-fix-for-audi-concert1chorus1/"));
  USEDSERIAL.println(F("https://github.com/tomaskovacik/audi_concert1_chorus1_volume_fix"));
}

void loop()
{
#ifdef USE_SERIAL
  if (Serial.available()) {
    if (Serial.read() == 'v') printInfo();
  }
#endif

  uint8_t resetNow = digitalRead(displayRESET);
  if (resetNow && !displayRESETstate)  displayRESETstate = 1;
  if (!resetNow && displayRESETstate)  displayRESETstate = 0;

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
      } else if (_data[1] == 8) {
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
static const uint8_t vol_up_levels[] PROGMEM = {
  0xEA, 0xD2, 0xBA, 0xA2, 0x92, 0x82, 0x72, 0x66,
  0x5E, 0x5A, 0x56, 0x52, 0x4E, 0x4A, 0x46, 0x42,
  0x3E, 0x3A, 0x36, 0x32, 0x2E, 0x2A, 0x26, 0x22,
  0x1E, 0x1A, 0x16, 0x12, 0x10
};

// Volume levels for down (quieter) steps — descending order
static const uint8_t vol_down_levels[] PROGMEM = {
  0xFF, 0xEE, 0xD6, 0xBE, 0xA4, 0x94, 0x84, 0x74,
  0x68, 0x60, 0x5C, 0x58, 0x54, 0x50, 0x4C, 0x48,
  0x44, 0x40, 0x3C, 0x38, 0x34, 0x30, 0x2C, 0x28,
  0x24, 0x20, 0x1C, 0x18, 0x14
};

void set_volume_up() {
  in_volume_recalc = true;
  for (uint8_t i = 0; i < sizeof(vol_up_levels); i++) {
    if (volume > pgm_read_byte(&vol_up_levels[i])) {
      volume = pgm_read_byte(&vol_up_levels[i]);
      return;
    }
  }
  volume = 0x10;
}

void set_volume_down() {
  in_volume_recalc = true;
  for (int8_t i = (int8_t)sizeof(vol_down_levels) - 1; i >= 0; i--) {
    if (volume < pgm_read_byte(&vol_down_levels[i])) {
      volume = pgm_read_byte(&vol_down_levels[i]);
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
  if (volume > 0x66) {
    loudness = 0x0E;
  } else if (volume > 0x5E) {
    loudness = 0x0D;
  } else {
    int8_t l = 0x0C - (int8_t)((0x5E - volume) / 4);
    loudness = (l < 0x06) ? 0x06 : (uint8_t)l;
  }
  while (current_loudness != loudness) {
    if (current_loudness < loudness)      loudness_packet[2] = ++current_loudness;
    else if (current_loudness > loudness) loudness_packet[2] = --current_loudness;
    sendI2C(loudness_packet);
  }
}

void decode_display_data(uint8_t _data[howmanybytesinpacket]) {
  grab_volume = 1;
  if (_data[1] == 0x58) grab_volume = 0;
  if (_data[1] == 0x71 && (_data[2] >> 4) <= 7) grab_volume = 0;

#ifdef USE_SERIAL
  USEDSERIAL.print(F("SPI"));
  for (uint8_t i = 0; i < howmanybytesinpacket; i++) {
    USEDSERIAL.print(' '); USEDSERIAL.print(_data[i], HEX);
  }
  USEDSERIAL.println();
#endif
}

// STATUS FALLING → SS LOW  (SPI slave selected, starts capturing 8 bits)
// STATUS RISING  → harvest SPDR (byte complete), then SS HIGH (resets SPI for next byte)
void mcuStatusChange()
{
  if (digitalRead(mcuSTATUS)) {
    // STATUS RISING — byte complete; read SPDR before raising SS (raising SS resets SPI)
    if (SPSR & (1 << SPIF)) {
      uint8_t b = SPDR;  // reading SPDR clears SPIF
      panel_message.write(b);
      if (panel_message.wbp == howmanybytesinpacket)
        panel_message.wbp = 0;
    }
    digitalWrite(mcuCS, HIGH);  // deselect / reset SPI for next byte

    if (digitalRead(mcuCLK)) {
      panel_message.commit();
      panel_message.busy = false;
    }
  } else {
    // STATUS FALLING — new byte starting
    digitalWrite(mcuCS, LOW);   // select SPI slave
    panel_message.busy = true;
  }
}

void receiveEvent(int howMany)
{
  if (howMany <= 0) return;
  if (howMany >= howmanybytesinpacket) howMany = howmanybytesinpacket - 1;
  i2c_data.busy = true;
  i2c_data.write((uint8_t)howMany);
  for (uint8_t i = 0; i < howMany; i++) {
    i2c_data.write(Wire.read());
  }
  i2c_data.commit();
  i2c_data.busy = false;
}

void sendI2C(const uint8_t data[howmanybytesinpacket]) {
  SWire.beginTransmission(MY_ADDRESS);
  for (byte i = 0; i < data[0]; i++) {
    while (!SWire.write(data[i + 1])) {}
  }
  SWire.endTransmission();
}
