// - master for HW 
// HWv1 and HWv2 without any #define regarding HW version
// HWv3 with #define HWV3
// HWv4 with #define HWV4
// HWv5 with #define HWV5
// module is held in reset state when front panel is off, no last volume is stored ...
//#include <Wire.h>

//#define HWV3
//#define HWV4


/* version 5 — passive sniffer (same role as HWv1-v4, CLK/DATA via SPI1 peripheral)
mcuCLK    = PA5  (SPI1_SCK  - input, MCU drives CLK)
mcuDATA   = PA7  (SPI1_MOSI - input, shared DATA via resistors)
mcuSTATUS = PA15 (input, panel drives STATUS)
*/
#define HWV5

#ifdef HWV5
#include <SPI.h>
#endif
#include <Wire_slave.h> //wireslave for stm32, there is no single lib for slave/master

#include <FlexWire.h> //so we do not have single lib for slave/master, so we have to init another one for master .... cose we do not have 3HW i2c .... tiktak ...
#include "audi_concert_panel.h"

FlexWire SWire = FlexWire(PB11, PB10);

//TwoWire Swire = TwoWire(PB11, PB10);
#define USE_SERIAL
//use Serial for medium-high density devices like stm32F103C8/B
//use Serial1 for low-density devices like stm32f103c6
#define USEDSERIAL Serial1
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
//ARDUINO
//#define mcuCLK 3 //CLK
//#define mcuSTATUS 2 //STATUS/CS
//#define mcuDATA 4 //PD2 - DATA
//#define displayRESET 8
//STM32
#ifdef HWV5
#define mcuCLK    PA5  // SPI1_SCK  - MCU drives CLK (input, passive sniff)
#define mcuDATA   PA7  // SPI1_MOSI - shared DATA via resistors (input, passive sniff)
#define mcuCS     PA3  // output: mirrors STATUS → HW inverter → SPI1_NSS (PA4)
#else
#define mcuCLK PB3 //CLK
#define mcuDATA PB4//DATA
#endif
#ifdef HWV3
#define mcuSTATUS PA4 //STATUS/CS
#define VERSION "1.0-09.06.22-HWv3"
#else//hw v4 and v5
#define mcuSTATUS PA15 //STATUS/CS
#define VERSION "2.0-25.05.26-HWv4"
#endif
#if defined(HWV5) || defined(HWV4) || defined(HWV3)
#define displayRESET PB8 //not used anyway ... 
#else
#define VERSION "1.0-09.06.22"
#define displayRESET PB5
#endif


//this is SW i2c for arduino, did not work on STM32, cose there is some ASM woodoo :)))
//#define DATA_IS_HIGH (PIND & (1<<PD4))
//#define SDA_PORT PORTC
//#define SDA_PIN A2 // = A2
//#define SCL_PORT PORTC
//#define SCL_PIN A3 // = A3
//#define I2C_SLOWMODE 1
//#include <SoftI2CMaster.h>
//TwoWire Wire(PB9,PB8, SOFT_STANDARD);
//SoftWire SWire(PB10, PB11, SOFT_FAST);

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

  // Copy the current read packet into dst[], clear the slot, then advance rdp
  void read(uint8_t dst[]) {
    for (uint8_t i = 0; i < howmanybytesinpacket; i++)
      dst[i] = buf[PACKET_IDX(rdp, i)];
    for (uint8_t i = 0; i < howmanybytesinpacket; i++)
      buf[PACKET_IDX(rdp, i)] = 0;
    if (++rdp == howmanypackets) rdp = 0;
  }

  // Append val at wbp in the current write packet and advance wbp
  void write(uint8_t val) {
    buf[PACKET_IDX(wdp, wbp)] = val;
    wbp++;
  }

  // Zero-fill remaining bytes in current write packet then advance wdp
  void commit() {
    while (wbp < howmanybytesinpacket) buf[PACKET_IDX(wdp, wbp++)] = 0;
    wbp = 0;
    if (++wdp == howmanypackets) wdp = 0;
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

volatile uint8_t _byte; //temporary, incoming byte is shiffted here, then when we are done grabbing it, it is stored in array each packet alone in one row


volatile uint8_t start_volume = 0xBA; //was 0xE4- set it 4leves lower, some complaints from BOSE users ..

volatile uint8_t volume = start_volume; //set start volume here ...
volatile uint8_t current_volume = start_volume; //set start volume here ..
volatile uint8_t saved_volume = start_volume; //set start volume here .. 0xFF to be sure, not fucked up repro ......

volatile uint8_t start_loudness = 0x0E;

volatile uint8_t loudness = start_loudness; //start loudness : OFF
volatile uint8_t current_loudness = start_loudness; //start loudness : OFF

volatile uint8_t grab_volume = 1;

volatile uint8_t mute = 0;
volatile uint8_t in_volume_recalc = 0;


uint8_t volume_packet[howmanybytesinpacket];
uint8_t loudness_packet[howmanybytesinpacket];

uint8_t displayRESETstate = 0;
//uint8_t dumpI2cDataAndDoNotFix = 0;
/*
   functions
*/

/*
   function is sending i2c packet, one day with fixed volume values :)
*/
void sendI2C(uint8_t data[howmanybytesinpacket]);

/*
   decode all i2c from MCU heading to TDA7342, with probably fked volume data
*/
void decode_i2c(uint8_t data[howmanybytesinpacket]);

/*
   yeh this would one day fix fked volume based on data we have from front panel and so on...
*/
void set_volume();
void set_loudness();

/*
   calculate speaker attenuations, cose we are calculating this for each speaker, so I made a function to avoid long code...
*/
//void spk_atten(uint8_t c);

/*
   RISING interrupt on STATE line
   it enable RISING interrupt on CLK line to grab data on dataline when fired
   this one also change interrupt on STATE line to FALLING

   first after fired, we check if CLK is LOW or HIGH:

   if HIGH - indicating end of packet:
            immediately detach this interrupt, without this  it was acting strangely
            change rest of packet in array to zero so there is no junk from previous communication (or should I zero it before? nooooo  here we have lot of time, cose we are at end of packet \o/
            set display byte pointer to 0
            increment display write pointer
            again attach same interrupt RISING on STATE line

   if LOW - we are going to receive data/more data - next byte in packet
            enabling FALLING interrupt on STATE line
            zeroing _byte variable, just in case, we should not need this, cose there should be 8 runs of CLK pulses, so it should overflow all old data .... but just in case we are doing it
            setting grabbing_SPI flag
            enabling RISING interrupt on CLK line
*/
void enableInterruptOnCLK();

/*

    fired when STATE goes LOW, disable any CLK interrupt, and enable RISING interrupt on STATE line

*/
void disableInterruptOnCLK();

/*

   function to grab data if CLK goes HIGH

*/
void readCLK();

/*

   classic setup function

*/

void set_mute();
void set_unmute();

void setup ()
{
  //PB3 works only if this is called
  //enableDebugPorts(); //required if not USB upload is used, for example pure STLINK or serial upload - perfect for low-density devices (103C6 for example)
  volume_packet[0] = 0x02;
  loudness_packet[0] = 0x02;
  volume_packet[1] = 0x02;
  loudness_packet[1] = 0x01;
  //init slave i2c to grab data for TDA7342
  Wire.begin (MY_ADDRESS);
  Wire.onReceive (receiveEvent);
  //master i2c to send data(fixed) to TDA7342
  SWire.begin();

  // HWV5 passive sniffer: STATUS ISR gates NSS (PA3→PA4/NSS) so
  pinMode(mcuSTATUS, INPUT_PULLUP);
  pinMode(mcuCS, OUTPUT);
  digitalWrite(mcuCS, !digitalRead(mcuSTATUS));  // 
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
}

void loop()
{
#ifdef USE_SERIAL
  if (Serial.available()) {
    if (Serial.read() == 'v') printInfo();
  }
#endif
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
            mute = 1;
            saved_volume = current_volume;
            volume = 0xFF;
            set_volume();
          }
          sendI2C(_data);
        } else {
          if (mute) {
            mute = 0;
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

/*
   send mute data over i2c
*/
void set_mute() {
  if (!mute) {
    mute = 1;
    uint8_t mute_data[howmanybytesinpacket] = {0x02, 0x08, 0x81, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    sendI2C(mute_data);
  }
}
/*
   send unmute data over i2c
*/
void set_unmute() {
  if (mute) {
    mute = 0;
    uint8_t mute_data[howmanybytesinpacket] = {0x02, 0x08, 0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    sendI2C(mute_data);
  }
}

void set_volume_up() {
  in_volume_recalc = 1; //fix #3
  // volume, 0xFF=off, 0x00=full on
  // if (volume == 0xFF) set_unmute();
  if (volume > 0xEA) {
    volume = 0xEA;
  } else if (volume > 0xD2) {
    volume = 0xD2;
  } else if (volume > 0xBA) {
    volume = 0xBA;
  } else if (volume > 0xA2) {
    volume = 0xA2;
  } else if (volume > 0x92) {
    volume = 0x92;
  } else if (volume > 0x82) {
    volume = 0x82;
  } else if (volume > 0x72) {
    volume = 0x72;
  } else if (volume > 0x66) {
    volume = 0x66;
  } else if (volume > 0x5E) {
    volume = 0x5E;
  } else if (volume > 0x5A) {
    volume = 0x5A;
  } else if (volume > 0x56) {
    volume = 0x56;
  } else if (volume > 0x52) {
    volume = 0x52;
  } else if (volume > 0x4E) {
    volume = 0x4E;
  } else if (volume > 0x4A) {
    volume = 0x4A;
  } else if (volume > 0x46) {
    volume = 0x46;
  } else if (volume > 0x42) {
    volume = 0x42;
  } else if (volume > 0x3E) { //after this its decrement of 2
    volume = 0x3E;
  } else if (volume > 0x3A) {
    volume = 0x3A;
  } else if (volume > 0x36) {
    volume = 0x36;
  } else if (volume > 0x32) {
    volume = 0x32;
  } else if (volume > 0x2E) {
    volume = 0x2E;
  } else if (volume > 0x2A) {
    volume = 0x2A;
  } else if (volume > 0x26) {
    volume = 0x26;
  } else if (volume > 0x22) {
    volume = 0x22;
  } else if (volume > 0x1E) {
    volume = 0x1E;
  } else if (volume > 0x1A) {
    volume = 0x1A;
  } else if (volume > 0x16) {
    volume = 0x16;
  } else if (volume > 0x12) {
    volume = 0x12;
  } else if (volume > 0x10) {
    volume = 0x10;
  }
  if (volume < 0x10) volume = 0x10; //top volume, seen on original communication was never less then 0x10
}

void set_volume_down() {
  in_volume_recalc = 1; //fix #3
  if (volume < 0x14) {
    volume = 0x14;
  } else if (volume < 0x18) {
    volume = 0x18;//+4
  } else if (volume < 0x1C) {
    volume = 0x1C;//+4
  } else if (volume < 0x20) {
    volume = 0x20;//+4
  } else if (volume < 0x24) {
    volume = 0x24;//+4
  } else if (volume < 0x28) {
    volume = 0x28;//+4
  } else if (volume < 0x2C) {
    volume = 0x2C;//+4
  } else if (volume < 0x30) {
    volume = 0x30;//+4
  } else if (volume < 0x34) {
    volume = 0x34;//+4
  } else if (volume < 0x38) {
    volume = 0x38;//+4
  } else if (volume < 0x3C) {
    volume = 0x3C;//+4
  } else if (volume < 0x40) {
    volume = 0x40;//+4
  } else if (volume < 0x44) {
    volume = 0x44;//+4
  } else if (volume < 0x48) {
    volume = 0x48;//+4
  } else if (volume < 0x4C) {
    volume = 0x4C;//+4
  } else if (volume < 0x50) {
    volume = 0x50;//+4
  } else if (volume < 0x54) {
    volume = 0x54;//+4
  } else if (volume < 0x58) {
    volume = 0x58;//+4
  } else if (volume < 0x5C) {
    volume = 0x5C;//+4
  } else if (volume < 0x60) {
    volume = 0x60;//+4
  } else if (volume < 0x68) {
    volume = 0x68;//+8
  } else if (volume < 0x74) {
    volume = 0x74;//+12
  } else if (volume < 0x84) {
    volume = 0x84;//+16
  } else if (volume < 0x94) {
    volume = 0x94;//+16
  } else if (volume < 0xA4) {
    volume = 0xA4;//+16
  } else if (volume < 0xBE) {
    volume = 0xBE;
  } else if (volume < 0xD6) {
    volume = 0xD6;
  } else if (volume < 0xEE) {
    volume = 0xEE;
  } else if (volume < 0xFF) {
    volume = 0xFF;
  }
}
/*

   function which should fix volume data, somehow ... :))

*/
void set_volume() {
  if (volume == 0xFF && !mute) saved_volume = current_volume;

  while (volume != current_volume) { //need to fix volume
    //     USEDSERIAL.println("=================== before fix ====================");
    //     USEDSERIAL.print("current volume: ");  USEDSERIAL.println(current_volume, HEX);
    //     USEDSERIAL.print("volume: ");  USEDSERIAL.println(volume, HEX);
    //     USEDSERIAL.print("saved volume: ");  USEDSERIAL.println(saved_volume, HEX);
    //     USEDSERIAL.println("====================================================");
    // USEDSERIAL.print(F("fixing volume from "));  USEDSERIAL.print(current_volume, HEX);  USEDSERIAL.print(F(" to "));  USEDSERIAL.println(volume, HEX);
    if (current_volume > volume ) { //current volume is more then volume , so we are turning volume up, step is 2
      if ((current_volume - volume) == 1) current_volume = volume;
      else current_volume = current_volume - 2;
      volume_packet[2] = current_volume;
      set_loudness();
      sendI2C(volume_packet);
    }
    if (current_volume < volume) { //current volume is less then volume , so we are turning volume down, step is 4 but some steps are more not divadeble by 4 (from a4 to be, for example)
      if ((volume - current_volume) == 1) current_volume = volume;
      else current_volume = current_volume + 2; //so we stick to 2
      volume_packet[2] = current_volume;
      set_loudness();
      sendI2C(volume_packet);
    }

    //test delay(1);

    //     USEDSERIAL.println("=================== after fix ====================");
    //     USEDSERIAL.print("current volume: ");  USEDSERIAL.println(current_volume, HEX);
    //     USEDSERIAL.print("volume: ");  USEDSERIAL.println(volume, HEX);
    //     USEDSERIAL.print("saved volume: ");  USEDSERIAL.println(saved_volume, HEX);
    //     USEDSERIAL.print("current loudness: ");  USEDSERIAL.println(current_loudness, HEX);
    //     USEDSERIAL.print("Loudness: ");  USEDSERIAL.println(loudness, HEX);
    //    if (mute)  USEDSERIAL.println("Muted");
    //    else  USEDSERIAL.println("Unmuted");
    //     USEDSERIAL.println("====================================================");
  }

  if (volume == 0xFF) set_mute();
  if (volume < 0xFE) set_unmute();
  in_volume_recalc = 0;
}

void set_loudness()
{
  if (volume > 0x66) {
    loudness = 0x0E;
  } else if (0x66 >= volume && volume > 0x5E) {
    loudness = 0x0D;
  } else if (0x5E >= volume && volume > 0x5A) {
    loudness = 0x0C;
  } else if (0x5A >= volume && volume > 0x56) {
    loudness = 0x0B;
  } else if (0x56 >= volume && volume > 0x52) {
    loudness = 0x0A;
  } else if (0x52 >= volume && volume > 0x4E) {
    loudness = 0x09;
  } else if (0x4E >= volume && volume > 0x4A) {
    loudness = 0x08;
  } else if (0x4A >= volume && volume > 0x46) {
    loudness = 0x07;
  } else if (0x46 >= volume) {
    loudness = 0x06;
  }
  while (current_loudness != loudness) { //need to hack this, cose loudness is set while volume is changed
    // USEDSERIAL.print(F("fixing loudness from "));  USEDSERIAL.print(current_loudness, HEX);  USEDSERIAL.print(F(" to "));  USEDSERIAL.println(loudness, HEX);
    //loudness is changed in increments of 1 so
    if (current_loudness < loudness) {
      loudness_packet[2] = ++current_loudness;
    }
    if (current_loudness > loudness) {
      loudness_packet[2] = --current_loudness;
    }
    sendI2C(loudness_packet);
    //test delay(1);
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




// ── HWV5 passive SPI1 sniffer ────────────────────────────────────────────────
// STATUS LOW  → NSS LOW  → SPI1 selected → counts 8 CLK edges → RXNE set.
// STATUS HIGH → NSS HIGH → SPI1 deselected → byte complete, read DR here in ISR.
// Reading DR in the ISR (not loop()) avoids the race where loop() is busy with
// Serial output and the SPI DR gets overwritten before it can be read.
// Two bugs fixed vs. polling approach:
//  1. Last byte of packet was always dropped (busy=0 set before loop() read RXNE).
//  2. Middle bytes were dropped when Serial print kept loop() busy too long.
#ifdef HWV5
void mcuStatusChange()
{
    if (digitalRead(mcuSTATUS)) {
        // STATUS RISING → deselect SPI1 (byte complete)
        digitalWrite(mcuCS, LOW);   // PA3 LOW → inverter → NSS HIGH

        // Harvest the completed byte immediately — before loop() gets a chance to run
        // (mirrors disableInterruptOnCLK() in the original SW SPI implementation)
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
#endif // HWV5

//enable RISING interrupt on CLK line when STATUS line rises (non-HWV5 passive sniffing)
void enableInterruptOnCLK()
{
  if (digitalRead(mcuCLK)) {
    detachInterrupt(digitalPinToInterrupt(mcuSTATUS)); //we need  to do this, cose otherwise it's doing strange things

    //CLK is HIGH, this is end of  packet
    //after this interrupt is still set to rising on STATUS line,
    panel_message.commit(); //zero-fill remaining bytes and advance write pointer
    attachInterrupt(digitalPinToInterrupt(mcuSTATUS), enableInterruptOnCLK, RISING); //enable this interrupt again, with same parameters
    //after this interupt is still set to rising on STATUS line,
    panel_message.busy = 0;//we are safe to manipulate data in main loop, I just move this from disableInteruptOnCLK function
  } else {
    //clk is low, start of packet
    attachInterrupt(digitalPinToInterrupt(mcuSTATUS), disableInterruptOnCLK, FALLING); //setting falling interrupt on STATE line, indicating end of byte transfer
    _byte = 0; //new data, zeroing temporary variable used to clock in data , just to be sure
    panel_message.busy = 1;//set grabit flag to avoid messing with live packet data in main loop
#ifdef HWV5
    // Reset SPI1 shift register so it aligns on the next byte's 8 CLK edges
    SPI1->regs->CR1 &= ~SPI_CR1_SPE;
    if (SPI1->regs->SR & SPI_SR_RXNE) (void)SPI1->regs->DR;
    SPI1->regs->CR1 |= SPI_CR1_SPE;
#else
    attachInterrupt(digitalPinToInterrupt(mcuCLK), readCLK, RISING); //enabling interupt on CLK like, to grab data after each fire of this int routine
#endif
  }
}

//disable CLK interrupt while STATUS is low
void disableInterruptOnCLK()
{
#ifndef HWV5
  detachInterrupt(digitalPinToInterrupt(mcuCLK)); //so STATUS is low, so all data are clocked in:
#endif
  panel_message.write(_byte); //move data from tempporary variable to array based on pointer of current packet and current byte in packet
  if (panel_message.wbp == howmanybytesinpacket ) { //this can happen, but it must be last byte in packet, otherwise we will rewrite data in packet row
    panel_message.wbp = 0;
#ifdef USE_SERIAL
    USEDSERIAL.println(F("dwbp overflow"));//put this out, just to know,
#endif
  }
  //grabbing_SPI = 0;//we are safe to manipulate data in main loop, I just move this to enableInterruptOnCLK function to part which indicate end of packet transfer
  attachInterrupt(digitalPinToInterrupt(mcuSTATUS), enableInterruptOnCLK, RISING);// enable RISING interrupt on STATE line, indicating start of transmission of data
}

void readCLK()
{
  //  USEDSERIAL.println(digitalRead(mcuDATA),DEC);
  if (digitalRead(mcuDATA)) {
    // if (DATA_IS_HIGH) {
    // USEDSERIAL.print(1);
    _byte = (_byte << 1) | 1;
  } else {
    //  USEDSERIAL.print(0);
    _byte = (_byte << 1);
  }
}

// called by interrupt service routine when incoming data arrives
void receiveEvent (int howMany)
{
  // USEDSERIAL.print(F("grabing i2c: wdp: "));  USEDSERIAL.print(i2c_data.wdp);//  USEDSERIAL.print(F(" howmany: ");  USEDSERIAL.println(howMany);
  i2c_data.busy = 1;
  i2c_data.write(howMany);
  for (uint8_t i = 0; i < howMany; i++) {
    i2c_data.write(Wire.read());
  }
  i2c_data.commit();
  i2c_data.busy = 0;
}  // end of receiveEvent



void sendI2C (uint8_t data[howmanybytesinpacket]) {
  SWire.beginTransmission(MY_ADDRESS); // transmit to device

  for (byte i = 0 ; i < data[0]; i++) {
    //i2c_write(data[i + 1]);
    while (!SWire.write(data[i + 1])) {
    }              // sends one byte
  }
  SWire.endTransmission();    // stop transmitting
}

/*
#ifdef USE_SERIAL
void decode_i2c(uint8_t data[howmanybytesinpacket]) {
  uint8_t increments = 1; //at least 1 iteration of next FOR must run...
  if (data[1] > 0xf) {
    //autoincrement of subaddress:
    //packet is subbadress+data, data[0] is packet size , so number of incrementation is "packet_size - 1"
    increments = data[0] - 1;
  }
  for (uint8_t i = 0; i < increments; i++) {
    uint8_t subaddress = (data[1] & 0xf) + i;//subbadress is always lower 4bits of 2nd field in array plus increment
    uint8_t c = data[i + 2];//0th byte -> size, 1st byte->subaddress
    // USEDSERIAL.println(subaddress);
    switch (subaddress) {
      case 0:
        { // input selector

          USEDSERIAL.print(F("Input selector: "));
          // USEDSERIAL.print(c,HEX);
          // USEDSERIAL.print(F(" ");
          switch (c & B0000111) {
            case 1:
              USEDSERIAL.println(F("TAPE selected (IN2)"));
              break;
            case 2:
              USEDSERIAL.println(F("FM / AM selected (IN1) "));
              break;
            case 3:
              USEDSERIAL.println(F("TP selected (AM mono)"));
              break;

          }
          switch (c & B01000111) {
            case 0:
              USEDSERIAL.println(F("CD selected (0dB diferential input gain (IN3))"));
              break;
            case 40:
              USEDSERIAL.println(F("CD selected (-6dB diferential input gain (IN3))"));
              break;

          }
          switch (c & B00011000) {
            case 0:
              USEDSERIAL.println(F("11.25dB gain"));
              break;
            case 1:
              USEDSERIAL.println(F("7.5dB gain"));
              break;
            case 2:
              USEDSERIAL.println(F("3.75dB gain"));
              break;
            case 3:
              USEDSERIAL.println(F("0dB gain"));
              break;
          }
        }
        break;
      case 1:
        { // loudness
          USEDSERIAL.print(F("Loudness: "));
          if (c > 0xf) {
            USEDSERIAL.println(F("OFF"));
          } else {
            USEDSERIAL.print(F(" - "));
            USEDSERIAL.print(((c & 0xF) * 1.25), DEC);
            USEDSERIAL.println(F("dB"));
          }
        }
        break;
      case 2:
        { // volume
          USEDSERIAL.print(F("volume "));
          //           USEDSERIAL.println(c);
          float _volume = 0;
          //           USEDSERIAL.println((c & B00000011) * (-0.31));
          //           USEDSERIAL.println(((c >> 2) & B00000111) * (-1.25));
          //           USEDSERIAL.println(20 - (((c >> 5) & B00000111) * 10));
          _volume = (20 - (((c >> 5) & B00000111) * 10)) + (((c >> 2) & B00000111) * (-1.25)) + ((c & B00000011) * (-0.31));
          USEDSERIAL.print(_volume);
          USEDSERIAL.print(F("dB"));
          USEDSERIAL.print(F(" ( "));
          USEDSERIAL.print(c, HEX);
          USEDSERIAL.println(F(" )"));
        }
        break;
      case 3: // bass, treble
        {
          USEDSERIAL.print(F("Bass: "));
          //Bass
          switch (c >> 4) {
            case 0:
              USEDSERIAL.print(-14);
              break;
            case 1:
              USEDSERIAL.print(-12);
              break;
            case 2:
              USEDSERIAL.print(-10);
              break;
            case 3:
              USEDSERIAL.print(-8);
              break;
            case 4:
              USEDSERIAL.print(-6);
              break;
            case 5:
              USEDSERIAL.print(-4);
              break;
            case 6:
              USEDSERIAL.print(-2);
              break;
            case 7:
            case 15:
              USEDSERIAL.print(0);
              break;
            case 8:
              USEDSERIAL.print(14);
              break;
            case 9:
              USEDSERIAL.print(12);
              break;
            case 10:
              USEDSERIAL.print(10);
              break;
            case 11:
              USEDSERIAL.print(8);
              break;
            case 12:
              USEDSERIAL.print(6);
              break;
            case 13:
              USEDSERIAL.print(4);
              break;
            case 14:
              USEDSERIAL.print(2);
              break;
          }
          USEDSERIAL.print(F("dB, Treble: "));
          //treble
          switch (c & 0xF) {
            case 0:
              USEDSERIAL.print(18);
              break;
            case 1:
              USEDSERIAL.print(16);
              break;
            case 2:
              USEDSERIAL.print(-10);
              break;
            case 3:
              USEDSERIAL.print(-8);
              break;
            case 4:
              USEDSERIAL.print(-6);
              break;
            case 5:
              USEDSERIAL.print(-4);
              break;
            case 6:
              USEDSERIAL.print(-2);
              break;
            case 7:
            case 15:
              USEDSERIAL.print(0);
              break;
            case 8:
              USEDSERIAL.print(14);
              break;
            case 9:
              USEDSERIAL.print(12);
              break;
            case 10:
              USEDSERIAL.print(10);
              break;
            case 11:
              USEDSERIAL.print(8);
              break;
            case 12:
              USEDSERIAL.print(6);
              break;
            case 13:
              USEDSERIAL.print(4);
              break;
            case 14:
              USEDSERIAL.print(2);
              break;
          }
          USEDSERIAL.println(F("dB"));
        }
        break;
      case 4: // Speaker Attenuator left front
        USEDSERIAL.print(F("Speaker Attenuator left front: "));
        spk_atten(c);
        break;
      case 5: // Speaker Attenuator left rear
        USEDSERIAL.print(F("Speaker Attenuator left rear: "));
        spk_atten(c);
        break;
      case 6: // Speaker Attenuator right front
        USEDSERIAL.print(F("Speaker Attenuator right front: "));
        spk_atten(c);
        break;
      case 7: // Speaker Attenuator left rear
        USEDSERIAL.print(F("Speaker Attenuator left rear: "));
        spk_atten(c);
        break;
      case 8: // mute
        USEDSERIAL.print(F("Mute: "));
        //0th and 1st bits
        switch (c & B00000011) {
          case 1:
            USEDSERIAL.println(F("Soft Mute with fast slope (I = Imax)"));
            break;
          case 3:
            USEDSERIAL.println(F("Soft Mute with slow slope (I = Imin)"));
            break;
        }
        //3th bit
        if ((c >> 3) & 1)  USEDSERIAL.println(F("Direct Mute"));
        //2nd and 4th bit
        if (!((c >> 5) & 1)) {
          USEDSERIAL.print(F("Zero Crossing Mute "));
          if ((c >> 2) & 1) {
            USEDSERIAL.println(F("On"));
          } else {
            USEDSERIAL.println(F("Off"));
          }
        }
        //5th and 6th bit
        switch ((c >> 5) & B00000011) {
          case 0:
            USEDSERIAL.println(F("160mV ZC Window Threshold (WIN = 00)"));
            break;
          case 1:
            USEDSERIAL.println(F("80mV ZC Window Threshold (WIN = 01)"));
            break;
          case 2:
            USEDSERIAL.println(F("40mV ZC Window Threshold (WIN = 10)"));
            break;
          case 3:
            USEDSERIAL.println(F("20mV ZC Window Threshold (WIN = 11)"));
            break;
        }
        switch ((c >> 7) & B00000011) {
          case 0:
            USEDSERIAL.println(F("Nonsymmetrical Bass Cut"));
            break;
          case 1:
            USEDSERIAL.println(F("Symmetrical Bass Cut"));
            break;
        }
        break;
    }
  }
}

void spk_atten(uint8_t c) {
  if ((c & B00011111) == 0x1F)
  {
    USEDSERIAL.println(F("Muted"));
  } else {

    float low = (c & B00000111) * 1.25;
    uint8_t high = ((c >> 3) & B00000011) * 10;
    USEDSERIAL.print(-(low + high));
    USEDSERIAL.println(F("dB"));
  }
}

*/
