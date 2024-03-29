// - master for HW 
// HWv1 and HWv2 without any #define regarding HW version
// HWv3 with #define HWV3
// HWv4 with #define HWV4
// HWv5 with #define HWV5
// module is hold in reseted state when fron panel is off, no last volume is stored ... 
//#include <Wire.h>

//#define HWV3
//#define HWV4


/* version 5
mcuCLK    = PA5
mcuDATA   = PA6
mcuSTATUS = PA15
*/
#define HWV5

#include <Wire_slave.h> //wireslave for stm32, there is no single lib for slave/master

#include <SlowSoftWire.h> //so we do not have single lib for slave/master, so we have to init another one for master .... cose we do not have 3HW i2c .... tiktak ...
#include "audi_concert_panel.h"

SlowSoftWire SWire = SlowSoftWire(PB11, PB10);

//TwoWire Swire = TwoWire(PB11, PB10);

//#define USE_SERIAL
//use Serial for medium-high density devices like stm32F103C8/B
//user Serial1 for low densty devices like stm32f103c6
#define USEDSERIAL Serial1
/*
    SPI comunication between motorola MC68HC05B32 cpu to front panel ST6280
    basics:
    SPI MODE 0

    CLK idle high, while
    STATUS idle high, it's like CS but it's triggered by slave here)
    DATA idle high

    comunication principle:
    CLK line going LOW is signaling someone wanna talk
    whatever it's slave (panel, cose we have buttons there ...) or master
    after CLK is low, slave bring STATE line LOW, and when he is ready to
    receive data he will bring STATE HIGH, then master will clock out data
    or clock in data, after everything is transmited, CLOCK goes HIGH and
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
#define mcuCLK PA5 //CLK
#define mcuDATA PA6//DATA
#else
#define mcuCLK PB3 //CLK
#define mcuDATA PB4//DATA
#endif
#ifdef HWV3
#define mcuSTATUS PA4 //STATUS/CS
#define VERSION "1.0-09.06.22-HWv3"
#else//hw v4 and v5
#define mcuSTATUS PA15 //STATUS/CS
#define VERSION "1.0-09.06.22-HWv4"
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

//we are trying to interface with comunication to TDA7342 which is 0x44 so...
#define I2C_7BITADDR 0x44
//array is klasik byte howmanypackets X howmanybytesinpacket
#define howmanypackets 20
#define howmanybytesinpacket 15

const byte MY_ADDRESS = I2C_7BITADDR;

uint8_t wdp = 0; //write data pointer, data for i2c, going from 0 to howmanypackets
uint8_t rdp = 0; //read data pointer for i2c comunication, going from 0 to howmanypackets
uint8_t reading_i2c = 0; //flag indicating that we are busy grabing i2c data, so we should not mess with them in main loop
/*
   array for i2c data, size+subaddress+4 (for seting balance and fade ...) I never see more data  so, 8 should be preaty safe...
   also never see more then 3 packet send one after another, so 6 packets should be ok
   but also after while, I implemented grabing display data which can be more then 8 like 15-16 bytes per packet
*/
volatile uint8_t data[howmanypackets][howmanybytesinpacket];

volatile uint8_t _byte; //temporary, incoming byte is shiffted here, then when we are done grabbing it, it is stored in array each packet alone in one row
volatile uint8_t _msg[howmanypackets][howmanybytesinpacket]; //here we have array for packet for display
volatile uint8_t dwdp = 0; //display write data pointer, going from 0 to howmanypackets
volatile uint8_t dwbp = 0; //display write byte pointer, for each dwdp there is "howmanybytesinpacket" dwbp,  going from to howmanybytesinpacket, we do not need this, cose i2c comunication has exact number of byte per packet ...
volatile uint8_t grabing_SPI = 0; //flag indicating we are busy grabing front panel display data, so we should not mess with them in main loop
volatile uint8_t drdp = 0; //display read data pointer for front panel comunication

volatile uint8_t start_volume = 0xBA; //was 0xE4- set it 4leves lower, some complaines from BOSE users .. 

volatile uint8_t volume = start_volume; //set start volume here ...
volatile uint8_t current_volume = start_volume; //set start volume here ..
volatile uint8_t saved_volume = start_volume; //set start volume here .. 0xFF to be sure, not fucked up repro ......

volatile uint8_t start_loudness = 0x0E;

volatile uint8_t loudness = start_loudness; //start loudness : OFF
volatile uint8_t current_loudness = start_loudness; //start loudness : OFF

volatile uint8_t grab_volume = 1;

volatile uint8_t mute = 0;


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
   calculate speaker attuenations, cose we are calculating this for each speaker, so I make fction to avoid long code...
*/
void spk_atten(uint8_t c);

/*
   RISING interupt on STATE line
   it enable RISING interupt on CLK line to grab data on dataline when fired
   this one also change interupt on STATE line to FALLING

   first after fired, we check if CLK is LOW or HIGH:

   if HIGH - indicating end of packet:
            immediatly detach this interrupt, without this  it was acting strangely
            change rest of packet in arary to zero so there is no junc from previous comunication (or should I zeroed it before? nooooo  here we have lot of time, cose we are at end of packet \o/
            set display byte pointer to 0
            increment dispay write pointer
            again attach same interupt RISING on STATE line

   if LOW - we are going to receive data/more data - next byte in packet
            enabling FALLING interupt on STATE line
            zeroing _byte variable, just in case, we should not need this, cose ther should be 8 runs of CLK pulses, so it should overflow all old data .... but just in case we are doing it
            seting grabing_SPI flag
            enabling RISING interupt on CLK line
*/
void enableInteruptOnCLK();

/*

    fired when STATE goes LOW, disale any CLK interupt, and enable RISING interupt on STATE line

*/
void disableInteruptOnCLK();

/*

   function to grabing data if CLK goes HIGH

*/
void readCLK();

/*

   clasic setup function

*/
#ifdef USE_SERIAL
void dump_i2c_data(uint8_t _data[howmanybytesinpacket]);
#endif

void set_mute();
void set_unmute();

void setup ()
{
  //PB3 works inly in this is called
  //enableDebugPorts(); //required if not USB upload is used, for example pure STLINK or serial upload - perfect for lowdensity devices (103C6 4exmaple)
  volume_packet[0] = 0x02;
  loudness_packet[0] = 0x02;
  volume_packet[1] = 0x02;
  loudness_packet[1] = 0x01;
  //init slave i2c to grab data for TDA7342
  Wire.begin (MY_ADDRESS);
  Wire.onReceive (receiveEvent);
  //master i2c to send data(fixed) to TDA7342
  SWire.begin();

  //  for (uint8_t i = 2; i < howmanybytesinpacket; i++) {
  //    volume_packet[i] = 0;
  //    loudness_packet[i] = 0;
  //  }
  //init pins for display SPI
  pinMode(mcuSTATUS, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(mcuSTATUS), enableInteruptOnCLK, RISING);
  pinMode(mcuCLK, INPUT_PULLUP);
  pinMode(mcuDATA, INPUT_PULLUP);
  pinMode(displayRESET, INPUT);
  //init interrupt on STATUS line to grab data send betwen display and main CPU

  //serial for debug
#ifdef USE_SERIAL
  USEDSERIAL.begin(115200);
#endif
  //arduino
  //      if (!i2c_init()) // Initialize everything and check for bus lockup
  //#ifdef USE_SERIAL
  //    USEDSERIAL.println(F("I2C init failed");
  //#endif
  set_volume();
}  // end of setup

void printInfo() {
  USEDSERIAL.print(F("Firmware version: "));
  USEDSERIAL.println(F(VERSION));
  USEDSERIAL.println(F("(C) kovo, GPL3"));
  USEDSERIAL.println(F("https://www.tindie.com/products/tomaskovacik/volume-fix-for-audi-concert1chorus1/"));
  USEDSERIAL.println(F("https://github.com/tomaskovacik/audi_concert1_chorus1_volume_fix"));
  // USEDSERIAL.println((dumpI2cDataAndDoNotFix ? F("Dumping i2c only ") : F("Fixing volume")));
}

void loop()
{
#ifdef USE_SERIAL
  if (Serial.available()) {
    char serial_char = Serial.read();
    switch (serial_char) {
      case 'D':
      //      case 'd':
      //      {
      //        dumpI2cDataAndDoNotFix = !dumpI2cDataAndDoNotFix;
      //        printInfo();
      //      }
      //      break;
      case 'h':
      case 'H':
      case '?':
      case 'v':
      case 'V':
        {
          printInfo();
        }
        break;
    }
  }
#endif
  /*
    if (digitalRead(displayRESET) && !displayRESETstate) {
       USEDSERIAL.println("Reset HIGH");
      displayRESETstate = 1;
    }
    if (!digitalRead(displayRESET) && displayRESETstate) {
       USEDSERIAL.println("Reset LOW");
      displayRESETstate = 0;
      wdp = rdp = dwdp = drdp = 0;
      while(!digitalRead(displayRESET)){
        //caled for sleep
        delay(1);
      }
    }
  */
  if (!grabing_SPI) { //no data are send on SPI line

    while (drdp != dwdp) { //reading and writing pointers are not in sync, we have some data which should be analyzed
      //move current reading data from array of packet in separate variable,
      //here should be memcopy, no for ... but... who cares ...
      //or we should just send pointer drdp as function parameter, array with packet is not local ....no I try it and it will use 1% more of program storage space  ...
      uint8_t _data[howmanybytesinpacket];
      for (uint8_t i = 0; i < howmanybytesinpacket; i++) {
        _data[i] = _msg[drdp][i];
        //#ifdef USE_SERIAL
        //   USEDSERIAL.print(_data[i],HEX);
        //#endif
      }
      if (_data[0] == 0x25)//button push
      {
#ifdef USE_SERIAL
        decode_button_push(_data[1]); //function which send to serial port real function of pressed button in human language
#endif
        //if (!dumpI2cDataAndDoNotFix) {
        if (grab_volume == 1 && (_data[1] == PANEL_KNOB_UP || _data[1] == PANEL_REMOTE_VOLUME_UP)) { //volume nob was turned up, and cose grab_volume is set to 1, we  know that is volume not  bass/treble/balance/fade, we set grab_volume=0 when display shows bass/treble/balance/fade)
          set_volume_up();
          set_volume();
        }
        if (grab_volume == 1 &&  (_data[1] == PANEL_KNOB_DOWN || _data[1] == PANEL_REMOTE_VOLUME_DOWN)) { //some as previous but nob goes down
          set_volume_down();
          set_volume();
        }
        //}
      }
      if (_data[0] == 0x9A) { // packet starting with 0x95 is update for pannel, text, indications leds ....
        decode_display_data(_data);
      }
      drdp++; //after everything increment read pointer
      if (drdp == howmanypackets) drdp = 0; //reset to zero if we are at top

    }
  }
  if (!reading_i2c) {// not capturing i2c, safe to mess with it
    while (rdp != wdp) {//reading and writing pointers are not in sync, we have some data which should be analyzed
      //move current reading data from array of packet in separate variable,
      //here should be memcopy, no for ... but... who cares ...
      uint8_t _data[howmanybytesinpacket];
      // USEDSERIAL.println(rdp);
      // USEDSERIAL.println(wdp);
      for (uint8_t i = 0; i < howmanybytesinpacket; i++) {
        _data[i] = data[rdp][i];
        // USEDSERIAL.print(_data[i],HEX);  USEDSERIAL.print(" ");
      }
      // USEDSERIAL.println();
      // if (!dumpI2cDataAndDoNotFix) {
      if ((_data[1] & 0x0f) == 1 || (_data[1] & 0x0F) == 2) {//volume was set by panel, and is probably fucked :) , only fixing volume packet, subbaddress = ?
        // USEDSERIAL.println(F("volume or loudness IGNORING!"));
        // USEDSERIAL.println(F("VOLI!"));
        //set_volume();
      } else if (_data[1] == 8 ) { //MUTE
        // USEDSERIAL.println(F("MUTE "));
        if ((_data[2] & B00000001)) {
          //2 8 81
          // USEDSERIAL.println(F("Muting")); dump_i2c_data(_data);
          if (!mute) { //we are not already muted
            mute = 1; //set mute flag
            saved_volume = current_volume;//save current volume
            volume = 0xFF; //set volume to be 0xFF (volume full down,off)
            set_volume();//set new volume
            //test delay(5);//to be sure? should check this on scope,
          }
          sendI2C(_data);//but send mute  command out anyway
        } else { //if it's not 1 then it's zero :)
          //2 8 80
          // USEDSERIAL.println(F("Unmuting")); dump_i2c_data(_data);

          if (mute) { //only unmute, if we are not unmuted already
            mute = 0; //clear mute flag
            volume = saved_volume;
            //saved_volume = start_volume; //set this to safe value if we fucked something in code, which I probably did :)
            set_volume();
          }
          sendI2C(_data);//send unmute command out before volume set
        }
      } else {
        sendI2C(_data);
      }
      /*      } else { //dumpI2cDataAndDoNotFix - we are gonna just dump data
               USEDSERIAL.print(millis()+String(" [")); dump_i2c_data(_data);  USEDSERIAL.print(F("] "));
              sendI2C(_data);
            }*/


      for (uint8_t i = 0; i < howmanybytesinpacket; i++) {
        data[rdp][i] = 0;
      }
      rdp++;
      if (rdp == howmanypackets) rdp = 0;
    }
    //       USEDSERIAL.print(F("wdp: ");  USEDSERIAL.print(wdp);  USEDSERIAL.print(F(" rdp ");  USEDSERIAL.println(rdp);
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
  mute = 1; //fix #3
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
  if (volume < 0x10) volume = 0x10; //top volume, seen on original comunication was never less then 0x10
}

void set_volume_down() {
  mute = 1; //fix #3
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

#ifdef USE_SERIAL
void dump_i2c_data(uint8_t _data[howmanybytesinpacket]) {

  USEDSERIAL.print(F("unknown display data: "));
  // }
  for (uint8_t i = 0; i < howmanybytesinpacket; i++) {
    USEDSERIAL.print(_data[i], HEX);
    USEDSERIAL.print(F(" "));
    //     USEDSERIAL.write(_data[i]);
  }
  USEDSERIAL.println();
}
#endif

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
#ifdef USE_SERIAL
  //  if(_data[1] == 0x13)  USEDSERIAL.println(_data[2],BIN); //debug
  uint8_t dump = 0;
  switch (_data[1]) { //switching second byte, which indicate type of packet data
    case 0x13:
      //leds: whole packet: 9A 13 2E 0 29 0 0 0 0 0 0 0 0 0 0
      {
        grab_volume = 1;
        // USEDSERIAL.println(_data[2],BIN);
        if (_data[2] & B00000001)  USEDSERIAL.print(F("REG ")); //REG bit
        if (_data[2] & B00000010)  USEDSERIAL.print(F("RDS ")); //RDS bit
        if (_data[2] & B00000100)  USEDSERIAL.print(F("AS ")); //AS bit
        if (_data[2] & B00001000) {
          if (_data[2] & B01000000)  USEDSERIAL.print(F("FM1 ")); //FM + 1 bit
          if (_data[2] & B00100000)  USEDSERIAL.print(F("FM2 ")); //FM + 2 bit
          if (_data[2] & B00010000)  USEDSERIAL.print(F("AM ")); //FM+|=AM bit
        }
        USEDSERIAL.println();
        USEDSERIAL.print(F("Memory: "));  USEDSERIAL.println(_data[3] & B00000011); //Memory


        USEDSERIAL.println();
        USEDSERIAL.println(F("LEDS: "));
        //this are in data[2]
        if (_data[3] & B00001000)  USEDSERIAL.print(F("CPS ")); //CPS
        if (_data[3] & B00100000)  USEDSERIAL.print(F("Dolby ")); //Dolby
        if (_data[3] & B00010000)  USEDSERIAL.print(F("RD ")); //RD
        //other in data[4]
        if (_data[4] & B00000001)  USEDSERIAL.print(F("RDS "));
        if (_data[4] & B00000010)  USEDSERIAL.print(F("AM "));
        if (_data[4] & B00000100)  USEDSERIAL.print(F("TP "));
        if (_data[4] & B00001000)  USEDSERIAL.print(F("FM "));
        if (_data[4] & B00010000)  USEDSERIAL.print(F("SCAN "));
        if (_data[4] & B00100000)  USEDSERIAL.print(F("AS "));
        if (_data[4] & B01000000)  USEDSERIAL.print(F("MODE "));
        USEDSERIAL.println();
      }
      break;
    case 0x23:
      {
        USEDSERIAL.println(F("display clear"));
      }
      break;
    case 0x32: //freq?
      {
        grab_volume = 1;
        if (_data[3] == 0x10) { //AM
          uint16_t freq = 531;
          freq = freq + (_data[2] * 9);
          USEDSERIAL.print(F("freq: "));
          USEDSERIAL.print(freq);
          USEDSERIAL.println(F(" kHz (AM)"));
        } else { //FM
          float freq = 875;
          freq = freq + _data[2];
          USEDSERIAL.print(F("freq: "));
          USEDSERIAL.print(freq / 10, 1);
          USEDSERIAL.println(F(" Mhz (FM)"));
        }
      }
      break;
    case 0x48:
      {
        grab_volume = 1;
        USEDSERIAL.print(F("display data ASCI: "));
        // }
        for (uint8_t i = 2; i < howmanybytesinpacket; i++) {
          USEDSERIAL.write(_data[i]);
          //debug
          // USEDSERIAL.print(F(" ");
          // USEDSERIAL.write(_data[i]);
        }
        USEDSERIAL.println();
      }
      break;
    case 0x58:
      {
        grab_volume = 0;
        //54 41 20 20 20 35 20 20 0 0 0 0 0
        //TEXT: VOL  1->...->5
        //TEXT: SENS LO
        //TEXT: RM   ON
        //TEXT: NAV  1->...->5
        //TEXT: TEL  L
        //TEXT: TA   1->...->5
        //TEXT: GALA OFF->1-...->5
        USEDSERIAL.write(_data[2]);
        USEDSERIAL.write(_data[3]);
        USEDSERIAL.write(_data[4]);
        USEDSERIAL.write(_data[5]);
        USEDSERIAL.write(_data[6]);
        USEDSERIAL.write(_data[7]);
        USEDSERIAL.write(_data[8]);
        USEDSERIAL.write(_data[9]);
        USEDSERIAL.println();
      }
      break;
    case 0x61:
      {
        grab_volume = 1;
        switch (_data[2]) {
          case 0x01:
            USEDSERIAL.println(F("TAPE: /\\"));
            break;
          case 2:
            USEDSERIAL.println(F("TAPE: \\/"));
            break;
          case 3:
            USEDSERIAL.println(F("TAPE:  > (FF)"));
            break;
          case 4:
            USEDSERIAL.println(F("TAPE:  < (FR)"));
            break;
          case 0:
            USEDSERIAL.println(F("TAPE: Eject"));
            break;
          case 0x10:
            USEDSERIAL.println(F("TP-INFO"));
            break;
          case 0x0B:
            USEDSERIAL.println(F("SAFE"));
            break;
          //9A 61 14
          case 0x14:
            USEDSERIAL.println(F("DIAG."));
            break;
          case 0x17:
            USEDSERIAL.println(F("??????????"));
            break;
          //BOSE: 9A 61 1A
          case 0x1A:
            USEDSERIAL.println("      BOSE      ");
            break;
          default:
            dump = 1;
        }
      }
      break;
    case 0x71:
      {
        //   USEDSERIAL.print(F("Stored text::");
        switch (_data[2] >> 4) {
          case 0x00:
            grab_volume = 0;
            USEDSERIAL.print(F("BAS "));
            if ((_data[2] & 0x0F) == 0) {
              USEDSERIAL.println(0);
            } else {
              USEDSERIAL.print(F("+"));
              USEDSERIAL.println((_data[2] & 0x0F), DEC);
            }
            break;
          case 0x1:
            grab_volume = 0;
            USEDSERIAL.print(F("BAS "));
            if ((_data[2] & 0x0F) == 0) {
              USEDSERIAL.println(0);
            } else {
              USEDSERIAL.print(F("-"));
              USEDSERIAL.println((_data[2] & 0x0F), DEC);
            }
            break;
          case 0x2:
            grab_volume = 0;
            USEDSERIAL.print(F("TRE "));
            if ((_data[2] & 0x0F) == 0) {
              USEDSERIAL.println(0);
            } else {
              USEDSERIAL.print(F("+"));
              USEDSERIAL.println((_data[2] & 0x0F), DEC);
            }
            break;
          case 0x3:
            grab_volume = 0;
            USEDSERIAL.print(F("TRE "));
            if ((_data[2] & 0x0F) == 0) {
              USEDSERIAL.println(0);
            } else {
              USEDSERIAL.print(F("-"));
              USEDSERIAL.println((_data[2] & 0x0F), DEC);
            }
            break;
          case 0x4:
            grab_volume = 0;
            USEDSERIAL.print(F("BAL "));
            if ((_data[2] & 0x0F) == 0) {
              USEDSERIAL.println(0);
            } else {
              USEDSERIAL.print(F("R"));
              USEDSERIAL.println((_data[2] & 0x0F), DEC);
            }
            break;
          case 0x5:
            grab_volume = 0;
            USEDSERIAL.print(F("BAL "));
            if ((_data[2] & 0x0F) == 0) {
              USEDSERIAL.println(0);
            } else {
              USEDSERIAL.print(F("L"));
              USEDSERIAL.println((_data[2] & 0x0F), DEC);
            }
            break;
          case 0x6:
            grab_volume = 0;
            USEDSERIAL.print(F("FAD "));
            if ((_data[2] & 0x0F) == 0) {
              USEDSERIAL.println(0);
            } else {
              USEDSERIAL.print(F("F"));
              USEDSERIAL.println((_data[2] & 0x0F), DEC);
            }
            break;
          case 0x7:
            grab_volume = 0;
            USEDSERIAL.print(F("FAD "));
            if ((_data[2] & 0x0F) == 0) {
              USEDSERIAL.println(0);
            } else {
              USEDSERIAL.print(F("R"));
              USEDSERIAL.println((_data[2] & 0x0F), DEC);
            }
            break;
          case 0xA:
            grab_volume = 1;
            USEDSERIAL.print(F("TP - MEM ")); //A1 is MEM 1?
            USEDSERIAL.print((_data[2] & 0x0F), DEC);
            break;
          case 0xB:
            {
              grab_volume = 1;
              //GALA
              USEDSERIAL.print("GALA "); //start radio with [2] pressed

              if ((_data[2] & 0x0F) == 1)  USEDSERIAL.println(F("OFF"));
              if ((_data[2] & 0x0F) == 0)  USEDSERIAL.println(F("ODB"));
            }
            break;
          default:
            dump = 1;
        }
      }
      break;
    case 0x80:
      {
        if (_data[2] == 0x00)  USEDSERIAL.println(F("Shutdown"));
      }
      break;
    case 0x92:
      {
        USEDSERIAL.print(F("Entered safe code:"));
        USEDSERIAL.print(_data[2], HEX);
        USEDSERIAL.println(_data[3], HEX);
        // USEDSERIAL.print(_data[4],HEX);
        // USEDSERIAL.println(_data[5],HEX);
      }
      break;
    case 0xA2:
      {
        grab_volume = 1;
        USEDSERIAL.print(F("CD"));
        USEDSERIAL.print(_data[2], HEX);
        USEDSERIAL.print(F(" TR"));
        USEDSERIAL.println(_data[3], HEX);
      }
      break;
    case 0xE1:
      {
        if (_data[2] == 0xFB)  USEDSERIAL.println(F("Start"));
      }
      break;
    default:
      {
        dump = 1;
      }
  }
  if (dump) {
    dump_i2c_data(_data);
  }
  
#else

  if (_data[1] == 0x58)
  {
    grab_volume = 0;
    //54 41 20 20 20 35 20 20 0 0 0 0 0
    //TEXT: VOL  1->...->5
    //TEXT: SENS LO
    //TEXT: RM   ON
    //TEXT: NAV  1->...->5
    //TEXT: TEL  L
    //TEXT: TA   1->...->5
    //TEXT: GALA OFF->1-...->5
  }
  if (_data[1] == 0x71)
  {
    //   USEDSERIAL.print(F("Stored text::");
    switch (_data[2] >> 4) {
      case 0x00: grab_volume = 0; break;
      case 0x1: grab_volume = 0; break;
      case 0x2: grab_volume = 0; break;
      case 0x3: grab_volume = 0; break;
      case 0x4: grab_volume = 0; break;
      case 0x5: grab_volume = 0; break;
      case 0x6: grab_volume = 0; break;
      case 0x7: grab_volume = 0; break;
    }
  }

#endif

}



//enable RISING interupt on CLK line when STATUS line RISED
void enableInteruptOnCLK()
{
  if (digitalRead(mcuCLK)) {
    detachInterrupt(digitalPinToInterrupt(mcuSTATUS)); //we need  to do this, cose otherwise it's doing strange things

    //CLK is HIGH, this is end of  packet
    while (dwbp < howmanybytesinpacket) { //clean array from current write pointer to end of packet
      _msg[dwdp][dwbp++] = 0;
    }
    dwbp = 0; //set byte write pointer to begining
    dwdp++; //increment write pointer
    if (dwdp == howmanypackets) dwdp = 0; //if we reach last+1 position in array for packet, go back to 0
    attachInterrupt(digitalPinToInterrupt(mcuSTATUS), enableInteruptOnCLK, RISING); //enable this interrupt again, with same parameters
    //after this interupt is still set to rising on STATUS line,
    grabing_SPI = 0;//we are safe to manipulate data in main loop, I just move this from disableInteruptOnCLK function
  } else {
    //clk is low, start of packet
    attachInterrupt(digitalPinToInterrupt(mcuSTATUS), disableInteruptOnCLK, FALLING); //seting falling interupt on STATE line, indicating end of byte transfer
    _byte = 0; //new data, zeroing temporary variable used to clock in data , just to be sure
    grabing_SPI = 1;//set grabit flag to avoid messing with live packet data in main loop
    attachInterrupt(digitalPinToInterrupt(mcuCLK), readCLK, RISING); //enabling interupt on CLK like, to grab data after each fire of this int routine
  }
}

//disable CLK interupt while STATUS is low
void disableInteruptOnCLK()
{
  detachInterrupt(digitalPinToInterrupt(mcuCLK)); //so STATUS is low, so all data are clocked in:
  _msg[dwdp][dwbp++] = _byte; //move data from tempporary variable to array based on pointer of current packet and current byte in packet
  if (dwbp == howmanybytesinpacket ) { //this can happend, but it must be last byte in packet, otherwise we will rewrite data in packet row
    dwbp = 0;
#ifdef USE_SERIAL
    USEDSERIAL.println(F("dwbp overflow"));//put this out, just to know,
#endif
  }
  //grabing_SPI = 0;//we are safe to manipulate data in main loop, I just move this to enableInteruptOnCLK function to part wich indicate end of packet transfer
  attachInterrupt(digitalPinToInterrupt(mcuSTATUS), enableInteruptOnCLK, RISING);// enable RISING interupt on STATE line, indicating start of transmition of data
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
  // USEDSERIAL.print(F("grabing i2c: wdp: "));  USEDSERIAL.print(wdp);//  USEDSERIAL.print(F(" howmany: ");  USEDSERIAL.println(howMany);
  reading_i2c = 1;
  data[wdp][0] = howMany;
  for (uint8_t i = 0; i < howMany; i++) {

    data[wdp][i + 1] = Wire.read();
    // USEDSERIAL.print(data[wdp][i + 1], HEX);
  }

  wdp++;
  if (wdp == howmanypackets) wdp = 0;
  reading_i2c = 0;
}  // end of receiveEvent



void sendI2C (uint8_t data[howmanybytesinpacket]) {
#ifdef USE_SERIAL
  decode_i2c(data);
#endif
  //  int timeout_us = 5000;
  //  while (!i2c_start((I2C_7BITADDR << 1) | I2C_WRITE) && timeout_us > 0) {
  //    delayMicroseconds(20);
  //    timeout_us -= 20;
  //  }
  //
  //  if (timeout_us <= 0) { // start transfer
  //     USEDSERIAL.println(F("I2C device busy");
  //    return;
  //  }

  SWire.beginTransmission(MY_ADDRESS); // transmit to device

  //  USEDSERIAL.print(F("size: ");
  //  USEDSERIAL.println(data[0]);

  for (byte i = 0 ; i < data[0]; i++) {
    //i2c_write(data[i + 1]);
    while (!SWire.write(data[i + 1])) {
      //test delay(10);
    }              // sends one byte

    // USEDSERIAL.print(data[i + 1], HEX);
    // USEDSERIAL.print(F(" ");
  }
  //i2c_stop(); // send stop condition
  SWire.endTransmission();    // stop transmitting
  // USEDSERIAL.println();
  // decode_i2c(data);
}


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

void decode_button_push(uint8_t data) {
  //  USEDSERIAL.print(data,HEX);
  switch (data) {
    case PANEL_1:
      USEDSERIAL.println(F(" 1"));
      break;
    case PANEL_2:
      USEDSERIAL.println(F(" 2"));
      break;
    case PANEL_3:
      USEDSERIAL.println(F(" 3"));
      break;
    case PANEL_4:
      USEDSERIAL.println(F(" 4"));
      break;
    case PANEL_5:
      USEDSERIAL.println(F(" 5"));
      break;
    case PANEL_6:
      USEDSERIAL.println(F(" 6"));
      break;
    case PANEL_SEEK_UP:
      USEDSERIAL.println(F(" Seek > "));
      break;
    case PANEL_TP:
      USEDSERIAL.println(F(" TP"));
      break;
    case PANEL_RDS:
      USEDSERIAL.println(F(" RDS"));
      break;
    case PANEL_CPS:
      USEDSERIAL.println(F(" CPS"));
      break;
    case PANEL_MODE:
      USEDSERIAL.println(F(" MODE"));
      break;
    case PANEL_RD:
      USEDSERIAL.println(F(" RD(random ? )"));
      break;
    case PANEL_PREVIOUS_TRACK:
      USEDSERIAL.println(F(" << "));
      break;
    case PANEL_FADE:
      USEDSERIAL.println(F(" FAD"));
      break;
    case PANEL_BALANCE:
      USEDSERIAL.println(F(" BALANCE"));
      break;
    case PANEL_BASS:
      USEDSERIAL.println(F(" BASS"));
      break;
    case PANEL_AM:
      USEDSERIAL.println(F(" AM"));
      break;
    case PANEL_DOLBY:
      USEDSERIAL.println(F(" Dolby"));
      break;
    case PANEL_NEXT_TRACK:
      USEDSERIAL.println(F(" >>"));
      break;
    case PANEL_TREBLE:
      USEDSERIAL.println(F(" TREB"));
      break;
    case PANEL_AS:
      USEDSERIAL.println(F(" AS"));
      break;
    case PANEL_SCAN:
      USEDSERIAL.println(F(" SCAN"));
      break;
    case PANEL_FM:
      USEDSERIAL.println(F(" FM"));
      break;
    case PANEL_SEEK_DOWN:
      USEDSERIAL.println(F(" Seek < "));
      break;
    case PANEL_REVERSE:
      USEDSERIAL.println(F(" REV"));
      break;
    case PANEL_KNOB_UP:
      USEDSERIAL.println(F(" Knob + "));
      break;
    case PANEL_KNOB_DOWN:
      USEDSERIAL.println(F(" Knob - "));
      break;
    case PANEL_CODE_IN:
      USEDSERIAL.println(F(" Code in (TP + RDS)"));
      break;
    case PANEL_EJECT:
      USEDSERIAL.println(F("eject"));
      break;
    case PANEL_BUTTON_RELEASE:
      USEDSERIAL.println(F("button release"));
      break;
    case PANEL_REMOTE_VOLUME_UP:
      USEDSERIAL.println(F("Remote volume up"));
      break;
    case PANEL_REMOTE_VOLUME_DOWN:
      USEDSERIAL.println(F("Remote volume down"));
      break;
    case PANEL_REMOTE_RIGHT:
      USEDSERIAL.println(F("Remote right"));
      break;
    case PANEL_REMOTE_LEFT:
      USEDSERIAL.println(F("Remote left"));
      break;
    case PANEL_REMOTE_UP:
      USEDSERIAL.println(F("Remote up"));
      break;
    case PANEL_REMOTE_DOWN:
      USEDSERIAL.println(F("Remote down"));
      break;
    case PANEL_START:
      USEDSERIAL.println(F("Panel start"));
      break;
    default:
      USEDSERIAL.print(F("Unknown button pushed: "));  USEDSERIAL.println(data, HEX);
      break;
  }
}
#endif
