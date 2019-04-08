//#include <Wire.h>
#include <Wire_slave.h> //wireslave for stm32, there is no single lib for slave/master

#include <SlowSoftWire.h> //so we do not have single lib for slave/master, so we have to init another one for master .... cose we do not have 3HW i2c .... tiktak ...
#include "audi_concert_panel.h"

SlowSoftWire SWire = SlowSoftWire(PB11, PB10);

//TwoWire Swire = TwoWire(PB11, PB10);


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
//#define displayCLK 3 //CLK
//#define displaySTATUS 2 //STATUS/CS
//#define displayDATA 4 //PD2 - DATA
//#define displayRESET 8
//STM32
#define displayCLK PB3 //CLK
#define displaySTATUS PA15 //STATUS/CS
#define displayDATA PB4//DATA
#define displayRESET PB5


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

volatile uint8_t start_volume = 0x82;

volatile uint8_t volume = start_volume; //set start volume here ...
volatile uint8_t currentVolume = start_volume; //set start volume here ..
volatile uint8_t savedVolume = start_volume; //set start volume here .. 0xFF to be sure, not fucked up repro ......

volatile uint8_t start_loudness = 0x0E;

volatile uint8_t loudness = start_loudness; //start loudness : OFF
volatile uint8_t current_loudness = start_loudness; //start loudness : OFF

volatile uint8_t grab_volume = 1;

volatile uint8_t mute = 0;

String VFversion = "1.0";

uint8_t volumePacket[howmanybytesinpacket];
uint8_t loudnessPacket[howmanybytesinpacket];

uint8_t displayRESETstate = 0;

uint8_t serialDebug = 0;
uint8_t muteIsLegit = 0; //used only while SCAN is used in FM, maybe in CDCHANGER mode is required also... but it is probably problem on my audi concert 1 unit: https://github.com/tomaskovacik/vwcdavr/issues/10

uint16_t input;

enum INPUTS
{
  RADIO,
  TP,
  CDCHANGER,
  TAPE
};
/*
   functions
*/

/*
   function is sending i2c packet
*/
void sendI2C(uint8_t data[howmanybytesinpacket]);

/*
   decode all i2c from MCU heading to TDA7342, with probably fked volume data
*/
void decode_i2c(uint8_t data[howmanybytesinpacket]);

/*
   yeh this fix volume based on data we have from front panel and so on...
*/
void setVolume();
void setLoudness();

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
void dump_data(uint8_t _data[howmanybytesinpacket]);

void set_mute();
void set_unmute();
void muteVolume(uint8_t dbg);
void restoreVolume();

void setup ()
{
  volumePacket[0] = 0x02;
  loudnessPacket[0] = 0x02;
  volumePacket[1] = 0x02;
  loudnessPacket[1] = 0x01;
  //init slave i2c to grab data for TDA7342
  Wire.begin (MY_ADDRESS);
  Wire.onReceive (receiveEvent);
  //master i2c to send data(fixed) to TDA7342
  SWire.begin();

  //  for (uint8_t i = 2; i < howmanybytesinpacket; i++) {
  //    volumePacket[i] = 0;
  //    loudnessPacket[i] = 0;
  //  }
  //init pins for display SPI
  pinMode(displaySTATUS, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(displaySTATUS), enableInteruptOnCLK, RISING);
  pinMode(displayCLK, INPUT_PULLUP);
  pinMode(displayDATA, INPUT_PULLUP);
  pinMode(displayRESET, INPUT);
  //init interrupt on STATUS line to grab data send betwen display and main CPU

  //serial for debug
  Serial.begin(115200);
  //arduino
  //      if (!i2c_init()) // Initialize everything and check for bus lockup
  //        if (serialDebug) Serial.println(F("I2C init failed");
}  // end of setup


void loop()
{
  if (Serial.available()) {
    char serial_char = Serial.read();
    if (serial_char == 'd') {
      Serial.print(F("debug output "));
      if (!serialDebug) {
        serialDebug = 1;
        Serial.println("enabled");
      } else {
        serialDebug = 0;
        Serial.println("disabled");
      }
    }
    if (serial_char == 'V') {
      Serial.print(F("Firmware version: "));
      Serial.println(VFversion);
      Serial.println(F("(C) kovo, GPL3"));
      Serial.println(F("https://www.tindie.com/products/tomaskovacik/volume-fix-for-audi-concert1chorus1/"));
    }
  }
  if (digitalRead(displayRESET) && !displayRESETstate) {
    if (serialDebug) Serial.println("Reset HIGH");
    displayRESETstate = 1;
    //mute=1;
    muteIsLegit=1;
  }
  if (!digitalRead(displayRESET) && displayRESETstate) {
    //if (serialDebug) Serial.println("Reset LOW");
    displayRESETstate = 0;
    wdp = rdp = dwdp = drdp = 0;
    //mute=1;
    muteVolume(0);
  }
  if (!grabing_SPI) { //no data are send on SPI line
    while (drdp != dwdp) { //reading and writing pointers are not in sync, we have some data which should be analyzed
      //move current reading data from array of packets in separate variable,
      //here should be memcopy, no for ... but... who cares ...
      //or we should just send pointer drdp as function parameter, array with packet is not local ....no I try it and it will use 1% more of program storage space  ...
      uint8_t _data[howmanybytesinpacket];
      for (uint8_t i = 0; i < howmanybytesinpacket; i++) {
        _data[i] = _msg[drdp][i];
      }
      if (_data[0] == 0x25)//button push
      {
        decode_button_push(_data[1]); //function which send to Serial port real function of pressed button in human language
        if (grab_volume == 1 && _data[1] == 0x86) { //volume nob was turned up, and cose grab_volume is set to 1, we  know that is turn of knob is not  bass/treble/balance/fade, we set grab_volume=0 when display shows bass/treble/balance/fade)
          setVolumeUp();
          setVolume();
        }
        if (grab_volume == 1 && _data[1] == 0x88) { //some as previous but nob goes down
          setVolumeDown();
          setVolume();
        }
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
      //move current reading data from array of packets in separate variable,
      //here should be memcopy, no for ... but... who cares ...
      uint8_t _data[howmanybytesinpacket];
      for (uint8_t i = 0; i < howmanybytesinpacket; i++) {
        _data[i] = data[rdp][i];
      }
      //volume was set by panel, and is probably f###d :) , only fixing volume packet, subbaddress = ?
      if ((_data[1] & 0x0f) == 1 || (_data[1] & 0x0F) == 2) {
      } else if (_data[1] == 8 ) { //MUTE
        if ((_data[2] & B00000001)) {
          if (serialDebug) Serial.println("Orig mute");
          //          if (!mute) { //we are not already muted
          //            mute = 1; //set mute flag
          //            savedVolume = currentVolume;//save current volume
          //            volume = 0xFF; //set volume to be 0xFF (volume full down,off)
          //            setVolume();//set new volume
          //            delay(5);//to be sure? should check this on scope,
          //            sendI2C(_data);//but send mute  command out anyway
          //          }
          if (muteIsLegit) {
            if (serialDebug) Serial.println(" used");
            muteVolume(1);
            muteIsLegit = 0;
          }
        } else { //if it's not 1 then it's zero :)
          if (serialDebug) Serial.println("Orig unmute. mute="+String(mute));
                    if (mute) { //only unmute, if we are not unmuted already
                    restoreVolume();
                    }
        }
      } else if (_data[1] == 0x3 && _data[2] == 0xFF && grab_volume == 1) { //bass/treble must be ignored if we are seting volume, because main MCU will send it even wron volume is set
        if (serialDebug) Serial.println("Ignored bass/treble settings from orig MCU!");
      } else {
        sendI2C(_data);
      }

      for (uint8_t i = 0; i < howmanybytesinpacket; i++) {
        data[rdp][i] = 0;
      }

      rdp++;
      if (rdp == howmanypackets) rdp = 0;
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

void muteVolume(uint8_t dbg) {
  if(!mute && currentVolume != 0xFF){
  if (serialDebug) Serial.println("mute volume dbg:" + String(dbg) +" currentVolume: "+String(currentVolume));
  savedVolume = currentVolume;//save current volume
  volume = 0xFF; //set volume to be 0xFF (volume full down,off)
  setVolume();//set new volume
  set_mute();
  }
}

void restoreVolume() {
  if (mute && currentVolume==0xFF){
  set_unmute();
  volume = savedVolume;
  if (serialDebug) Serial.println("Restor volume. New volume: "+String(volume));
  //savedVolume = start_volume; //set this to safe value if we fucked something in code, which I probably did :)
  setVolume();
  }
}

void setVolumeUp() {
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

void setVolumeDown() {
  if (volume < 0x14) {
    volume = 0x14;
  } else  if (volume < 0x60) {
    volume = volume + 4; //+4
  } else if (volume < 0x68) {
    volume = volume + 8; //+8
  } else if (volume < 0x74) {
    volume = volume + 12; //+12
  } else if (volume < 0xA4) {
    volume = volume + 16; //+16
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
void setVolume() {
  while (volume != currentVolume) { //need to fix volume
    if (currentVolume > volume ) { //current volume is more then volume , so we are turning volume up, step is 2
      if ((currentVolume - volume) == 1) currentVolume = volume;
      else currentVolume = currentVolume - 2;
      volumePacket[2] = currentVolume;
      setLoudness();
      sendI2C(volumePacket);
    }
    if (currentVolume < volume) { //current volume is less then volume , so we are turning volume down, step is 4 but some steps are more not divadeble by 4 (from a4 to be, for example)
      if ((volume - currentVolume) == 1) currentVolume = volume;
      else currentVolume = currentVolume + 2; //so we stick to 2
      volumePacket[2] = currentVolume;
      setLoudness();
      sendI2C(volumePacket);
    }

    delay(1);
  }

  if (volume == 0xFF) set_mute();
  if (volume < 0xFE) set_unmute();
}

void setLoudness()
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
    //loudness is changed in increments of 1 so
    if (current_loudness < loudness) {
      loudnessPacket[2] = ++current_loudness;
    }
    if (current_loudness > loudness) {
      loudnessPacket[2] = --current_loudness;
    }
    sendI2C(loudnessPacket);
    delay(1);
  }
}

void dump_data(uint8_t _data[howmanybytesinpacket]) {

  if (serialDebug) Serial.print(F("unknown data: "));
  for (uint8_t i = 0; i < howmanybytesinpacket; i++) {
    if (serialDebug) Serial.print(_data[i], HEX);
    if (serialDebug) Serial.print(F(" "));
  }
  if (serialDebug) Serial.println();
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
  //  if(_data[1] == 0x13) if (serialDebug) Serial.println(_data[2],BIN); //debug
  uint8_t dump = 0;
  grab_volume = 1;
  switch (_data[1]) { //switching second byte, which indicate type of packet data

    case 0x13:
      //leds: whole packet: 9A 13 2E 0 29 0 0 0 0 0 0 0 0 0 0
      {
        //if (serialDebug) Serial.println(_data[2],BIN);
        if (_data[2] & B00000001) if (serialDebug) Serial.print(F("REG ")); //REG bit
        if (_data[2] & B00000010) if (serialDebug) Serial.print(F("RDS ")); //RDS bit
        if (_data[2] & B00000100) if (serialDebug) Serial.print(F("AS ")); //AS bit
        if (_data[2] & B00001000) {
          if (_data[2] & B01000000) if (serialDebug) Serial.print(F("FM1 ")); //FM + 1 bit
          if (_data[2] & B00100000) if (serialDebug) Serial.print(F("FM2 ")); //FM + 2 bit
          if (_data[2] & B00010000) if (serialDebug) Serial.print(F("AM ")); //FM+|=AM bit
        }
        if (serialDebug) Serial.println();
        if (serialDebug) Serial.print(F("Memory: ")); if (serialDebug) Serial.println(_data[3] & B00000011); //Memory


        if (serialDebug) Serial.println();
        if (serialDebug) Serial.println(F("LEDS: "));
        //this are in data[2]
        if (_data[3] & B00001000) if (serialDebug) Serial.print(F("CPS ")); //CPS
        if (_data[3] & B00100000) if (serialDebug) Serial.print(F("Dolby ")); //Dolby
        if (_data[3] & B00010000) if (serialDebug) Serial.print(F("RD ")); //RD
        //other in data[4]
        if (_data[4] & B00000001) if (serialDebug) Serial.print(F("RDS "));
        if (_data[4] & B00000010) if (serialDebug) Serial.print(F("AM "));
        if (_data[4] & B00000100) if (serialDebug) Serial.print(F("TP "));
        if (_data[4] & B00001000) if (serialDebug) Serial.print(F("FM "));
        if (_data[4] & B00010000) {
          if (serialDebug) Serial.print(F("SCAN "));
          if (input == RADIO) muteVolume(2);
          muteIsLegit = 1;
        }
        if (_data[4] & B00100000) if (serialDebug) Serial.print(F("AS "));
        if (_data[4] & B01000000) if (serialDebug) Serial.print(F("MODE "));
        if (serialDebug) Serial.println();
      }
      break;
    case 0x23:
      {
        if (serialDebug) Serial.println(F("display clear"));
      }
      break;
    case 0x32: //freq?
      {
        if (_data[3] == 0x10) { //AM
          uint16_t freq = 531;
          freq = freq + (_data[2] * 9);
          if (serialDebug) Serial.print(F("freq: "));
          if (serialDebug) Serial.print(freq);
          if (serialDebug) Serial.println(F(" kHz (AM)"));
        } else { //FM
          float freq = 875;
          freq = freq + _data[2];
          if (serialDebug) Serial.print(F("freq: "));
          if (serialDebug) Serial.print(freq / 10, 1);
          if (serialDebug) Serial.println(F(" Mhz (FM)"));
        }
      }
      break;
    case 0x48:
      {
        if (serialDebug) Serial.print(F("display data ASCI: "));
        // }
        for (uint8_t i = 2; i < howmanybytesinpacket; i++) {
          if (serialDebug) Serial.write(_data[i]);
          //debug
          //if (serialDebug) Serial.print(F(" ");
          //if (serialDebug) Serial.write(_data[i]);
        }
        if (serialDebug) Serial.println();
      }
      break;
    case 0x58: //text
      {
        //54 41 20 20 20 35 20 20 0 0 0 0 0
        if (serialDebug) Serial.write(_data[2]);
        if (serialDebug) Serial.write(_data[3]);
        if (serialDebug) Serial.write(_data[4]);
        if (serialDebug) Serial.write(_data[5]);
        if (serialDebug) Serial.write(_data[6]);
        if (serialDebug) Serial.write(_data[7]);
        if (serialDebug) Serial.write(_data[8]);
        if (serialDebug) Serial.write(_data[9]);
        if (serialDebug) Serial.println();
      }
      break;
    case 0x61:
      {
        switch (_data[2]) {
          case 0x01:
            if (serialDebug) Serial.println(F("TAPE: /\\"));
            break;
          case 2:
            if (serialDebug) Serial.println(F("TAPE: \\/"));
            break;
          case 3:
            if (serialDebug) Serial.println(F("TAPE:  > (FF)"));
            break;
          case 4:
            if (serialDebug) Serial.println(F("TAPE:  < (FR)"));
            break;
          case 5:
            if (serialDebug) Serial.println(F("CONNECT"));
            break;
          case 0:
            if (serialDebug) Serial.println(F("TAPE: Eject"));
            break;
          case 0x10:
            if (serialDebug) Serial.println(F("TP-INFO"));
            restoreVolume();
            break;
          case 0x11:
            if (serialDebug) Serial.println(F("TELEFON"));
            break;
          case 0x0B:
            if (serialDebug) Serial.println(F("SAFE"));
            break;
          case 0x17:
            if (serialDebug) Serial.println(F("??????????"));
            break;
          default:
            dump = 1;
        }
      }
      break;
    case 0x71:
      {
        //  if (serialDebug) Serial.print(F("Stored text::");
        switch (_data[2] >> 4) {
          case 0x00:
            grab_volume = 0;
            if (serialDebug) Serial.print(F("BAS "));
            if ((_data[2] & 0x0F) == 0) {
              if (serialDebug) Serial.println(0);
            } else {
              if (serialDebug) Serial.print(F("+"));
              if (serialDebug) Serial.println((_data[2] & 0x0F), DEC);
            }
            break;
          case 0x1:
            grab_volume = 0;
            if (serialDebug) Serial.print(F("BAS "));
            if ((_data[2] & 0x0F) == 0) {
              if (serialDebug) Serial.println(0);
            } else {
              if (serialDebug) Serial.print(F("-"));
              if (serialDebug) Serial.println((_data[2] & 0x0F), DEC);
            }
            break;
          case 0x2:
            grab_volume = 0;
            if (serialDebug) Serial.print(F("TRE "));
            if ((_data[2] & 0x0F) == 0) {
              if (serialDebug) Serial.println(0);
            } else {
              if (serialDebug) Serial.print(F("+"));
              if (serialDebug) Serial.println((_data[2] & 0x0F), DEC);
            }
            break;
          case 0x3:
            grab_volume = 0;
            if (serialDebug) Serial.print(F("TRE "));
            if ((_data[2] & 0x0F) == 0) {
              if (serialDebug) Serial.println(0);
            } else {
              if (serialDebug) Serial.print(F("-"));
              if (serialDebug) Serial.println((_data[2] & 0x0F), DEC);
            }
            break;
          case 0x4:
            grab_volume = 0;
            if (serialDebug) Serial.print(F("BAL "));
            if ((_data[2] & 0x0F) == 0) {
              if (serialDebug) Serial.println(0);
            } else {
              if (serialDebug) Serial.print(F("R"));
              if (serialDebug) Serial.println((_data[2] & 0x0F), DEC);
            }
            break;
          case 0x5:
            grab_volume = 0;
            if (serialDebug) Serial.print(F("BAL "));
            if ((_data[2] & 0x0F) == 0) {
              if (serialDebug) Serial.println(0);
            } else {
              if (serialDebug) Serial.print(F("L"));
              if (serialDebug) Serial.println((_data[2] & 0x0F), DEC);
            }
            break;
          case 0x6:
            grab_volume = 0;
            if (serialDebug) Serial.print(F("FAD "));
            if ((_data[2] & 0x0F) == 0) {
              if (serialDebug) Serial.println(0);
            } else {
              if (serialDebug) Serial.print(F("F"));
              if (serialDebug) Serial.println((_data[2] & 0x0F), DEC);
            }
            break;
          case 0x7:
            grab_volume = 0;
            if (serialDebug) Serial.print(F("FAD "));
            if ((_data[2] & 0x0F) == 0) {
              if (serialDebug) Serial.println(0);
            } else {
              if (serialDebug) Serial.print(F("R"));
              if (serialDebug) Serial.println((_data[2] & 0x0F), DEC);
            }
            break;
          case 0xA:
            grab_volume = 1;
            if (serialDebug) Serial.print(F("TP - MEM ")); //A1 is MEM 1?
            if (serialDebug) Serial.print((_data[2] & 0x0F), DEC);
            break;
          case 0xB:
            {
              //GALA
              if (serialDebug) Serial.print("GALA ");

              if ((_data[2] & 0x0F) == 1) if (serialDebug) Serial.println(F("OFF"));
              if ((_data[2] & 0x0F) == 0) if (serialDebug) Serial.println(F("ODB"));
            }
            break;
          default:
            dump = 1;
        }
      }
      break;
    case 0x80:
      {
        if (_data[2] == 0x00) {
          muteVolume(3);
          if (serialDebug) Serial.println(F("Shutdown"));
        }
      }
      break;
    case 0x92:
      {
        if (serialDebug) Serial.print(F("Entered safe code:"));
        if (serialDebug) Serial.print(_data[2], HEX);
        if (serialDebug) Serial.println(_data[3], HEX);
        //if (serialDebug) Serial.print(_data[4],HEX);
        //if (serialDebug) Serial.println(_data[5],HEX);
      }
      break;
    case 0xA2:
      {
        input = CDCHANGER;
        if (serialDebug) Serial.print(F("CD"));
        if (serialDebug) Serial.print(_data[2], HEX);
        if (serialDebug) Serial.print(F(" TR"));
        if (serialDebug) Serial.println(_data[3], HEX);
      }
      break;
    case 0xE1:
      {
        if (_data[2] == 0xFB) if (serialDebug) Serial.println(F("Start"));
      }
      break;
    default:
      {
        dump = 1;
      }
  }
  if (dump) {
    dump_data(_data);
  }
}


//enable RISING interupt on CLK line when STATUS line RISED
void enableInteruptOnCLK()
{
  if (digitalRead(displayCLK)) {
    detachInterrupt(digitalPinToInterrupt(displaySTATUS)); //we need  to do this, cose otherwise it's doing strange things

    //CLK is HIGH, this is end of  packet
    while (dwbp < howmanybytesinpacket) { //clean array from current write pointer to end of packet
      _msg[dwdp][dwbp++] = 0;
    }
    dwbp = 0; //set byte write pointer to begining
    dwdp++; //increment write pointer
    if (dwdp == howmanypackets) dwdp = 0; //if we reach last+1 position in array for packet, go back to 0
    attachInterrupt(digitalPinToInterrupt(displaySTATUS), enableInteruptOnCLK, RISING); //enable this interrupt again, with same parameters
    //after this interupt is still set to rising on STATUS line,
    grabing_SPI = 0;//we are safe to manipulate data in main loop, I just move this from disableInteruptOnCLK function
  } else {
    //clk is low, start of packet
    attachInterrupt(digitalPinToInterrupt(displaySTATUS), disableInteruptOnCLK, FALLING); //seting falling interupt on STATE line, indicating end of byte transfer
    _byte = 0; //new data, zeroing temporary variable used to clock in data , just to be sure
    grabing_SPI = 1;//set grabit flag to avoid messing with live packet data in main loop
    attachInterrupt(digitalPinToInterrupt(displayCLK), readCLK, RISING); //enabling interupt on CLK like, to grab data after each fire of this int routine
  }
}

//disable CLK interupt while STATUS is low
void disableInteruptOnCLK()
{
  detachInterrupt(digitalPinToInterrupt(displayCLK)); //so STATUS is low, so all data are clocked in:
  _msg[dwdp][dwbp++] = _byte; //move data from tempporary variable to array based on pointer of current packet and current byte in packet
  if (dwbp == howmanybytesinpacket ) { //this can happend, but it must be last byte in packet, otherwise we will rewrite data in packet row
    dwbp = 0;
    if (serialDebug) Serial.println(F("dwbp overflow"));//put this out, just to know,
  }
  //grabing_SPI = 0;//we are safe to manipulate data in main loop, I just move this to enableInteruptOnCLK function to part wich indicate end of packet transfer
  attachInterrupt(digitalPinToInterrupt(displaySTATUS), enableInteruptOnCLK, RISING);// enable RISING interupt on STATE line, indicating start of transmition of data
}

void readCLK()
{
  // if (serialDebug) Serial.println(digitalRead(displayDATA),DEC);
  if (digitalRead(displayDATA)) {
    // if (DATA_IS_HIGH) {
    //if (serialDebug) Serial.print(1);
    _byte = (_byte << 1) | 1;
  } else {
    // if (serialDebug) Serial.print(0);
    _byte = (_byte << 1);
  }
}

// called by interrupt service routine when incoming data arrives
void receiveEvent (int howMany)
{
  //if (serialDebug) Serial.print(F("grabing i2c: wdp: ")); if (serialDebug) Serial.print(wdp);// if (serialDebug) Serial.print(F(" howmany: "); if (serialDebug) Serial.println(howMany);
  reading_i2c = 1;
  data[wdp][0] = howMany;
  for (uint8_t i = 0; i < howMany; i++) {

    data[wdp][i + 1] = Wire.read();
    //if (serialDebug) Serial.print(data[wdp][i + 1], HEX);
  }

  wdp++;
  if (wdp == howmanypackets) wdp = 0;
  reading_i2c = 0;
}  // end of receiveEvent



void sendI2C (uint8_t data[howmanybytesinpacket]) {
  decode_i2c(data);
  //  int timeout_us = 5000;
  //  while (!i2c_start((I2C_7BITADDR << 1) | I2C_WRITE) && timeout_us > 0) {
  //    delayMicroseconds(20);
  //    timeout_us -= 20;
  //  }
  //
  //  if (timeout_us <= 0) { // start transfer
  //    if (serialDebug) Serial.println(F("I2C device busy");
  //    return;
  //  }

  SWire.beginTransmission(MY_ADDRESS); // transmit to device

  // if (serialDebug) Serial.print(F("size: ");
  // if (serialDebug) Serial.println(data[0]);

  for (byte i = 0 ; i < data[0]; i++) {
    //i2c_write(data[i + 1]);
    while (!SWire.write(data[i + 1])) {
      delay(10);
    }              // sends one byte

    //if (serialDebug) Serial.print(data[i + 1], HEX);
    //if (serialDebug) Serial.print(F(" ");
  }
  //i2c_stop(); // send stop condition
  SWire.endTransmission();    // stop transmitting
  //if (serialDebug) Serial.println();
  // decode_i2c(data);
}

void decode_i2c(uint8_t data[howmanybytesinpacket]) {
  //dump_data(data);
  uint8_t increments = 1; //at least 1 iteration of next FOR must run...
  if (data[1] > 0xf) {
    //autoincrement of subaddress:
    //packet is subbadress+data, data[0] is packet size , so number of incrementation is "packet_size - 1"
    increments = data[0] - 1;
  }
  for (uint8_t i = 0; i < increments; i++) {
    uint8_t subaddress = (data[1] & 0xf) + i;//subbadress is always lower 4bits of 2nd field in array plus increment
    uint8_t c = data[i + 2];//0th byte -> size, 1st byte->subaddress
    //if (serialDebug) Serial.println(subaddress);
    switch (subaddress) {
      case 0:
        { // input selector
          //muteVolume(4); //works here but it is too slow, display data are much faster
          if (input == TP) muteVolume(5); //goig from TP to what ever we going to play
          if (serialDebug) Serial.print(F("Input selector: "));
          //if (serialDebug) Serial.print(c,HEX);
          //if (serialDebug) Serial.print(F(" ");
          switch (c & B0000111) {
            case 1:
              input = TAPE;
              muteVolume(55);
              if (serialDebug) Serial.println(F("TAPE selected (IN2)"));
              break;
            case 2:
              input = RADIO;
              if (serialDebug) Serial.println(F("FM / AM selected (IN1) "));
              break;
            case 3:
              input = TP;
              muteVolume(6);
              if (serialDebug) Serial.println(F("TP selected (AM mono)"));
              break;

          }
          switch (c & B01000111) {
            case 0:
              input = CDCHANGER;
              if (serialDebug) Serial.println(F("CD selected (0dB diferential input gain (IN3))"));
              break;
            case 40:
              input = CDCHANGER;
              if (serialDebug) Serial.println(F("CD selected (-6dB diferential input gain (IN3))"));
              break;

          }
          switch (c & B00011000) {
            case 0:
              if (serialDebug) Serial.println(F("11.25dB gain"));
              break;
            case 1:
              if (serialDebug) Serial.println(F("7.5dB gain"));
              break;
            case 2:
              if (serialDebug) Serial.println(F("3.75dB gain"));
              break;
            case 3:
              if (serialDebug) Serial.println(F("0dB gain"));
              break;
          }
        }
        break;
      case 1:
        { // loudness
          if (serialDebug) Serial.print(F("Loudness: "));
          if (c > 0xf) {
            if (serialDebug) Serial.println(F("OFF"));
          } else {
            if (serialDebug) Serial.print(F(" - "));
            if (serialDebug) Serial.print(((c & 0xF) * 1.25), DEC);
            if (serialDebug) Serial.println(F("dB"));
          }
        }
        break;
      case 2:
        { // volume
          if (serialDebug) Serial.print(F("volume "));
          //          if (serialDebug) Serial.println(c);
          float _volume = 0;
          //          if (serialDebug) Serial.println((c & B00000011) * (-0.31));
          //          if (serialDebug) Serial.println(((c >> 2) & B00000111) * (-1.25));
          //          if (serialDebug) Serial.println(20 - (((c >> 5) & B00000111) * 10));
          _volume = (20 - (((c >> 5) & B00000111) * 10)) + (((c >> 2) & B00000111) * (-1.25)) + ((c & B00000011) * (-0.31));
          if (serialDebug) Serial.print(_volume);
          if (serialDebug) Serial.print(F("dB"));
          if (serialDebug) Serial.print(F(" ( "));
          if (serialDebug) Serial.print(c, HEX);
          if (serialDebug) Serial.println(F(" )"));
        }
        break;
      case 3: // bass, treble
        {
          if (serialDebug) Serial.print(F("Bass: "));
          //Bass
          switch (c >> 4) {
            case 0:
              if (serialDebug) Serial.print(-14);
              break;
            case 1:
              if (serialDebug) Serial.print(-12);
              break;
            case 2:
              if (serialDebug) Serial.print(-10);
              break;
            case 3:
              if (serialDebug) Serial.print(-8);
              break;
            case 4:
              if (serialDebug) Serial.print(-6);
              break;
            case 5:
              if (serialDebug) Serial.print(-4);
              break;
            case 6:
              if (serialDebug) Serial.print(-2);
              break;
            case 7:
            case 15:
              if (serialDebug) Serial.print(0);
              break;
            case 8:
              if (serialDebug) Serial.print(14);
              break;
            case 9:
              if (serialDebug) Serial.print(12);
              break;
            case 10:
              if (serialDebug) Serial.print(10);
              break;
            case 11:
              if (serialDebug) Serial.print(8);
              break;
            case 12:
              if (serialDebug) Serial.print(6);
              break;
            case 13:
              if (serialDebug) Serial.print(4);
              break;
            case 14:
              if (serialDebug) Serial.print(2);
              break;
          }
          if (serialDebug) Serial.print(F("dB, Treble: "));
          //treble
          switch (c & 0xF) {
            case 0:
              if (serialDebug) Serial.print(18);
              break;
            case 1:
              if (serialDebug) Serial.print(16);
              break;
            case 2:
              if (serialDebug) Serial.print(-10);
              break;
            case 3:
              if (serialDebug) Serial.print(-8);
              break;
            case 4:
              if (serialDebug) Serial.print(-6);
              break;
            case 5:
              if (serialDebug) Serial.print(-4);
              break;
            case 6:
              if (serialDebug) Serial.print(-2);
              break;
            case 7:
            case 15:
              if (serialDebug) Serial.print(0);
              break;
            case 8:
              if (serialDebug) Serial.print(14);
              break;
            case 9:
              if (serialDebug) Serial.print(12);
              break;
            case 10:
              if (serialDebug) Serial.print(10);
              break;
            case 11:
              if (serialDebug) Serial.print(8);
              break;
            case 12:
              if (serialDebug) Serial.print(6);
              break;
            case 13:
              if (serialDebug) Serial.print(4);
              break;
            case 14:
              if (serialDebug) Serial.print(2);
              break;
          }
          if (serialDebug) Serial.println(F("dB"));
        }
        break;
      case 4: // Speaker Attenuator left front
        if (serialDebug) Serial.print(F("Speaker Attenuator left front: "));
        spk_atten(c);
        break;
      case 5: // Speaker Attenuator left rear
        if (serialDebug) Serial.print(F("Speaker Attenuator left rear: "));
        spk_atten(c);
        break;
      case 6: // Speaker Attenuator right front
        if (serialDebug) Serial.print(F("Speaker Attenuator right front: "));
        spk_atten(c);
        break;
      case 7: // Speaker Attenuator left rear
        if (serialDebug) Serial.print(F("Speaker Attenuator left rear: "));
        spk_atten(c);
        break;
      case 8: // mute
        if (serialDebug) Serial.print(F("Mute: "));
        //0th and 1st bits
        switch (c & B00000011) {
          case 1:
            if (serialDebug) Serial.println(F("Soft Mute with fast slope (I = Imax)"));
            break;
          case 3:
            if (serialDebug) Serial.println(F("Soft Mute with slow slope (I = Imin)"));
            break;
        }
        //3th bit
        if ((c >> 3) & 1) if (serialDebug) Serial.println(F("Direct Mute"));
        //2nd and 4th bit
        if (!((c >> 5) & 1)) {
          if (serialDebug) Serial.print(F("Zero Crossing Mute "));
          if ((c >> 2) & 1) {
            if (serialDebug) Serial.println(F("On"));
          } else {
            if (serialDebug) Serial.println(F("Off"));
          }
        }
        //5th and 6th bit
        switch ((c >> 5) & B00000011) {
          case 0:
            if (serialDebug) Serial.println(F("160mV ZC Window Threshold (WIN = 00)"));
            break;
          case 1:
            if (serialDebug) Serial.println(F("80mV ZC Window Threshold (WIN = 01)"));
            break;
          case 2:
            if (serialDebug) Serial.println(F("40mV ZC Window Threshold (WIN = 10)"));
            break;
          case 3:
            if (serialDebug) Serial.println(F("20mV ZC Window Threshold (WIN = 11)"));
            break;
        }
        switch ((c >> 7) & B00000011) {
          case 0:
            if (serialDebug) Serial.println(F("Nonsymmetrical Bass Cut"));
            break;
          case 1:
            if (serialDebug) Serial.println(F("Symmetrical Bass Cut"));
            break;
        }
        break;
    }
  }
}

void spk_atten(uint8_t c) {
  if ((c & B00011111) == 0x1F)
  {
    if (serialDebug) Serial.println(F("Muted"));
  } else {

    float low = (c & B00000111) * 1.25;
    uint8_t high = ((c >> 3) & B00000011) * 10;
    if (serialDebug) Serial.print(-(low + high));
    if (serialDebug) Serial.println(F("dB"));
  }
}

void decode_button_push(uint8_t data) {
  // if (serialDebug) Serial.print(data,HEX);
  switch (data) {
    case PANEL_1:
      if (input == RADIO) muteVolume(7);
      if (serialDebug) Serial.println(F(" 1"));
      break;
    case PANEL_2:
      if (input == RADIO) muteVolume(8);
      if (serialDebug) Serial.println(F(" 2"));
      break;
    case PANEL_3:
      if (input == RADIO) muteVolume(9);
      if (serialDebug) Serial.println(F(" 3"));
      break;
    case PANEL_4:
      if (input == RADIO) muteVolume(10);
      if (serialDebug) Serial.println(F(" 4"));
      break;
    case PANEL_5:
      if (input == RADIO) muteVolume(11);
      if (serialDebug) Serial.println(F(" 5"));
      break;
    case PANEL_6:
      if (input == RADIO) muteVolume(12);
      if (serialDebug) Serial.println(F(" 6"));
      break;
    case PANEL_SEEK_UP:
      if (input == RADIO) muteVolume(13); //not sure
      if (serialDebug) Serial.println(F(" Seek > "));
      break;
    case PANEL_TP:
      //if (input==RADIO|CDCHANGER|TAPE) muteVolume(14);
      if (serialDebug) Serial.println(F(" TP"));
      break;
    case PANEL_RDS:
      if (serialDebug) Serial.println(F(" RDS"));
      break;
    case PANEL_CPS:
      if (serialDebug) Serial.println(F(" CPS"));
      break;
    case PANEL_MODE:
      muteVolume(15);
      if (serialDebug) Serial.println(F(" MODE"));
      break;
    case PANEL_RD:
      if (serialDebug) Serial.println(F(" RD(random ? )"));
      break;
    case PANEL_PREVIOUS_TRACK:
      if (input == TAPE) muteVolume(16); //not sure
      if (serialDebug) Serial.println(F(" << "));
      break;
    case PANEL_FADE:
      if (serialDebug) Serial.println(F(" FAD"));
      break;
    case PANEL_BALANCE:
      if (serialDebug) Serial.println(F(" BALANCE"));
      break;
    case PANEL_BASS:
      if (serialDebug) Serial.println(F(" BASS"));
      break;
    case PANEL_AM:
      if (serialDebug) Serial.println(F(" AM"));
      break;
    case PANEL_DOLBY:
      if (serialDebug) Serial.println(F(" Dolby"));
      break;
    case PANEL_NEXT_TRACK:
      if (input == TAPE) muteVolume(17); //not sure
      if (serialDebug) Serial.println(F(" >>"));
      break;
    case PANEL_TREBLE:
      if (serialDebug) Serial.println(F(" TREB"));
      break;
    case PANEL_AS:
      if (serialDebug) Serial.println(F(" AS"));
      break;
    case PANEL_SCAN:
      if (serialDebug) Serial.println(F(" SCAN"));
      break;
    case PANEL_FM:
      if (serialDebug) Serial.println(F(" FM"));
      break;
    case PANEL_SEEK_DOWN:
      if (input == RADIO) muteVolume(18);
      if (serialDebug) Serial.println(F(" Seek < "));
      break;
    case PANEL_REVERSE:
      if (input == TAPE) muteVolume(19); //not sure
      if (serialDebug) Serial.println(F(" REV"));
      break;
    case PANEL_KNOB_UP:
      if (serialDebug) Serial.println(F(" Knob + "));
      break;
    case PANEL_KNOB_DOWN:
      if (serialDebug) Serial.println(F(" Knob - "));
      break;
    case PANEL_CODE_IN:
      if (serialDebug) Serial.println(F(" Code in (TP + RDS)"));
      break;
    case PANEL_EJECT:
      if (input == TAPE) muteVolume(20);
      if (serialDebug) Serial.println(F("eject"));
      break;
    case PANEL_BUTTON_RELEASE:
      if (serialDebug) Serial.println(F("button release"));
      break;
    default:
      if (serialDebug) Serial.print(F(" unknown")); if (serialDebug) Serial.println(data, HEX);
      break;
  }
}
