/*
ALS-U 1U 2 CH Corrector Power Supply

Build 05/07/2024 Initial release
Build 09/23/2024 Remove read 1-wire temp sensors at turn-on




D. Bergman

Libraries are placed in /root/Arduino/libraries when installed by the library manager or may be
placed manually in ~/Arduino install directory/libraries/

Need to install the following libraries:
1. UIPEthernet
   SdFat
2. Create directory "OneWire" in /Arduino install directory/libraries/, and put the following files in it:
      DS2482.cpp, DS2482.h, DS18B20_DS2482.cpp, DS18B20_DS2482.h
      Need to comment out blockTillConversionComplete in DS18B20_DS2482::requestTemperatures(), line 336 in DS18B20_DS2482.cpp 

3. Need modified pins_arduino.h file to use non-Arduino digital IO on Atmega chip. Put in 
C:\Program Files (x86)\Arduino\hardware\arduino\avr\variants\mega\. Restart Arduino IDE.



*/

//#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>
//#include <stdint.h>

#include <avr/wdt.h>
#include <Wire.h>
#include <SdFat.h> // need this to use fgets() for line parsing

#include <DS2482.h>
#include <DS18B20_DS2482.h> //This includes one-wire functions
DS2482 DS2482(0);   // 0 = 0x18 Default Address, 1 = 0x19, 2 = 0x1A, 3 = 0x1b
DS18B20_DS2482 DS18B20_Device(&DS2482); //Pass DS18B20_DS2482 to DS2482 library
DeviceAddress Addr; // 8 byte data type DeviceAddress for 1-wire devices



#include <UIPEthernet.h>
//byte mac[] = {0x00,0x01,0x02,0x03,0x04,0x05};
// test board 192.168.0.6
// SN002 192.168.0.7
// SN004 192.168.0.8
//IPAddress ip(192,168,0,8);
IPAddress remote_IP;
int remote_port;

unsigned int port = 5000;
char packetBuffer[100]; // message from client to initiate UDP sending of housekeeping data
char datagram[2000];// housekeeping data packet UDP datagram
char scratch1[10]; // scratchpad for building datagram


// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

EthernetServer server(80);

//1-wire variables
int SensorCount = 0;
int DS2482_Num = 0;
//uint8_t sensor_1[8] = {0x28, 0xE5, 0xB8, 0x58, 0x09, 0x00, 0x00, 0xE7};
//uint8_t sensor_2[8] = {0x28, 0x37, 0x2D, 0x57, 0x09, 0x00, 0x00, 0xB2};
//uint8_t sensor_1[8] = {0x28, 0x7B, 0x3B, 0x58, 0x09, 0x00, 0x00, 0xC0};
//uint8_t sensor_2[8] = {0x28, 0xFA, 0x4F, 0x57, 0x09, 0x00, 0x00, 0xF5};
 
//

//fan pwm function
void pwm25k(int val6, int val7, int val8)
{
    OCR4A = val6; // pin 6
    OCR4B = val7; // pin 7
    OCR4C = val8; // pin 8
}


long clamp(long x, long a, long b)
{
  long z;
  if(x < a) z = a;
  else if(x > b) z = b;
  else(z = x);

  return z;
}

uint8_t ipOctet[4];
byte mac[6];
uint8_t sensor_1[8]; 
uint8_t sensor_2[8];
float hall_cal[] = {1.0, 1.0}; // one correction for each PWM power daughterboard
float mod_serial = 6202004;
float buildnumber = 240923.2; // .2 = set fan duty > 2000. .1 = don't set fan duty//////////////

int readSD(void) {

const int sdCS = 16; // chip select pin for SD module
char *p1;
char *pEnd; // dummy output of strtol()
char s1[30];
char hdr1[30];
char* delim1 = ":";
char* delim2 = ".";
char* delim3 = ",";
char buf[128]; // buffer for each line of file
int line=1;
size_t n;

SdFat sd;
SdFile file;
//File file;

Serial.print("Initializing SD card...");
if(!sd.begin(sdCS)){
  Serial.println("SD initialization failed. Halting...");
  //while(true); // halt bootup
  return(-1);
}
Serial.println("SD initialization done.");

Serial.println("Reading SD Card...");
if(!file.open("config.txt", O_READ)) {
   Serial.print("Can't open SD card config.txt file! Halting...");
   //while(true);
   return(-1);
   }

while( (n = file.fgets(buf, 100)) > 0) { // one line from file each loop
  
  p1 = strtok(buf, delim1); // first call to strtok returns pointer to buf with length 
                            // up to delim1
  strcpy(hdr1, p1);
  Serial.print(hdr1);
  Serial.print(": "); 
  
  switch(line){
    case 1:
    //while(p1 != NULL){
    for(int k=0; k<4; k++){
    p1 = strtok(NULL, delim2); // returns pointer starting after delim1 with length
                   // up to first delim2
    strcpy(s1, p1);
    ipOctet[k] = atoi(s1);
    Serial.print(ipOctet[k]);
    Serial.print('.');
    }
    break;

    case 2:
    //while(p1 != NULL){
    for(int k=0; k<6; k++){
    p1 = strtok(NULL, delim3); // returns pointer starting after delim1 with length
                   // up to first delim3
    strcpy(s1, p1);
    //mac[k] = atoi(s1);
    mac[k] = byte(strtol(s1, &pEnd, 16));
    Serial.print(mac[k],HEX);
    Serial.print(' ');
    }
    break;
    
    case 3:
    //while(p1 != NULL){
    for(int k=0; k<8; k++){
    p1 = strtok(NULL, delim3); // returns pointer starting after delim1 with length
                   // up to first delim3
    strcpy(s1, p1);
    sensor_1[k] = strtol(s1, NULL, 16);
    Serial.print(sensor_1[k],HEX);
    Serial.print(" ");
    }
    break;

    case 4:
    //while(p1 != NULL){
    for(int k=0; k<8; k++){
    p1 = strtok(NULL, delim3); // returns pointer starting after delim1 with length
                   // up to first delim3
    strcpy(s1, p1);
    sensor_2[k] = strtol(s1, NULL, 16);
    Serial.print(sensor_2[k], HEX);
    Serial.print(" ");
    }
    break;

    case 5:
    //while(p1 != NULL){
    for(int k=0; k<2; k++){
    p1 = strtok(NULL, delim3); // returns pointer starting after delim1 with length
                   // up to first delim3
    strcpy(s1, p1);
    //hall_cal[k] = strtof(s1, NULL, 16);
    hall_cal[k] = atof(s1);
    Serial.print(hall_cal[k]);
    Serial.print(" ");
    }
    break;

    case 6:
    //while(p1 != NULL){
    for(int k=0; k<1; k++){
    p1 = strtok(NULL, delim3); // returns pointer starting after delim1 with length
                   // up to first delim3
    strcpy(s1, p1);
    //hall_cal[k] = strtof(s1, NULL, 16);
    mod_serial = atof(s1);
    Serial.print(mod_serial,0);
    Serial.print(" ");
    }
    break;
    
     
  } // end switch
    
Serial.println("");
line++;
}

file.close();
Serial.println("...Done Reading SD Card");
Serial.print("\n");

return(0);
}
//end readSD()


void scanI2C()
{
  for (byte address = 1; address < 127; address++)
  {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    
    if (error == 0)
    {
    Serial.print("I2C device found at address 0x");
    if (address<16)
      Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(" ");
      DS2482_Num++;
    }
    else if (error ==4)
    {
      Serial.print("Error at address 0x");
      if(address <16)
        Serial.print("0");
      Serial.println(address,HEX);
    }
  }
  if (DS2482_Num == 0) Serial.println("No I2C Device Detected ");
  
Serial.println("");
}


void printAddress(DeviceAddress deviceAddress) // prints 1-wire device ID codes
{
  for (uint8_t i = 0; i < 8; i++) //8 bytes, 64 bits
  {
    Serial.print("0x");
    if (deviceAddress[i] < 0x10) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
    if (i < 7) Serial.print(", "); 
  }
  Serial.println(" ");
}



float getTemperature(DeviceAddress deviceAddress)
{
  float TempC = DS18B20_Device.getTempC(deviceAddress); 
  return TempC;
}



int PMBusAddr = 0x50;
uint8_t PMBusData[10] = {};
//uint8_t PMBusNumbytes;
uint8_t PMBusCmd_SetIoutMax5[] = {0xCC, 0x32, 0x00, 0x5B}; //Set IoutMax to 5 A. Use address 0xA0 to compute checksum.
uint8_t PMBusCmd_SetIoutMax2[] = {0xCC, 0x14, 0x00, 0x8B}; //Set IoutMax to 2 A
uint8_t PMBusCmd_SetIoutMax12[] = {0xCC, 0x7D, 0x00, 0xC3};//Set Ioutmax to 12.5 A
uint8_t PMBusCmd_SetPage[][3] = {
  {0x00, 0x00, 0x48},
  {0x00, 0x01, 0x4F},
  {0x00, 0x02, 0x46},
  {0x00, 0x03, 0x41},
  {0x00, 0x04, 0x54} };

uint16_t status_data;
uint8_t PMBusCmd_ClearFaults[] = {0x03, 0x11};
uint8_t PMBusCmd_SetOperation[] = {0x01, 0x80, 0xd4};
uint8_t PMBusCmd_SetFan1_30pct[] = {0x3b, 0x2c, 0x01, 0xa7};
uint8_t PMBusCmd_SetFan2_30pct[] = {0x3c, 0x2c, 0x01, 0xb1};


//unsigned long td1 = 30; // minimum ms delay needed after changing Delta PS page before query

float HKdata[60] = {};

char cmd;
unsigned long i = 0; // loop index
unsigned long i1 = 0; // loop index used for measuring loop timing
unsigned long i2 = 0; // loop index used for measuring loop timing
unsigned uptime_H; // uptime count high word
unsigned long uptime_L0; //uptime count low word previous loop 
unsigned long uptime_L1; // uptime count low word current loop
unsigned long uptime_s; // uptime in seconds
int j = 0;// i % 16 for ADCs
int k = 1; // 1 to 4 counter for Delta PS page changes
int n=0;
int resetFlag;
int flipclk=0;
float T1=-99; // sampled every 2 s
//float T1_fan[2] = {}; // sampled once every 30 s for fan control
float T2=-99;
//float T2_fan[2] = {}; 

float T1_avg;
float T2_avg;
float T1_buf[] = {0, 0, 0, 0, 0};
float T2_buf[] = {0, 0, 0, 0, 0};

//temperature resolution is 0.03125 deg C using avg of two 1-wire readings.
//Min to max pwm over an error range of 3.125 deg C gives 100 discrete PWM settings.
float T1_err = 0.0;
float T2_err = 0.0;
float T_errmax = 6.25; //degrees C
float T_set = 37.0; // degrees C
int Dmax = 200; // pwm duty cycle out of 320
int Dmin = 1;

//status 
int ovc_CH1 = 0; // overcurrent
int ovc_CH2 = 0;
int hall_mismatch_CH1 = 0;
int hall_mismatch_CH2 = 0;
int ovt1 = 0; // over temperature
int ovt2 = 0;
int sumfault_CH1; // summary fault
int sumfault_CH2;

int sts_mod[]={LOW,LOW,LOW,LOW}; // one low for every Delta module
int ON1cmdCH1[]={LOW,LOW};
int ON1cmdCH2[]={LOW,LOW};
int CH1_ON[]={0,0}; // CH1 state
int CH2_ON[]={0,0}; // CH2 state
int heartbeat=1;


//Channels 0-3 and 8-11 are Hall current sensors. Channels 4 and 12 are Vout. 5 and 13 are Vcap.
//Channels 6,7, 14, and 15 are temperature sensors.
//int chan_list[] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15};
int chan_list[] = {A0, A1, A2, A3, A8, A9, A10, A11}; // bridge 1, 2, 3, 4
int ADCs[8];
float ADC_[8];
float I_hall[8];
float I_hall_avg[8];
float I_hall_buf[8][10]; // running average buffer, 8 channels, up to 10 avgs per channel
float I_hall_buf1[8][2]; // buffer for IIR filter
//int tmr1[4] = {0, 0, 0, 0};
//int tmr2[4] = {0, 0, 0, 0};
int tmr1; // channel 1 OVC fault delay
int tmr2; // channel 1 mismatch fault delay 
int tmr3; // channel 2 OVC fault delay
int tmr4; // channel 2 mismatch fault delay
int tmr_ON1fltCH1=5000; // initial countdown value for detection of i2c 24V on with status low
int tmr_ON1fltCH2=5000; // initial countdown value for detection of i2c 24V on with status low
int tmr_ON1CH1turnOFF=40; //countdown when ON1 pulses go away
int tmr_ON1CH2turnOFF=40; 
int tmr_ON1CH1turnON=0; //count up when ON1 pulses start
int tmr_ON1CH2turnON=0;
//int tmr_ON1 = 0; // delay timers for deglitching om/off command
//int tmr_OFF1 = 0;
//int tmr_ON2 = 0;
//int tmr_OFF2 = 0;

unsigned long fan_ctrl; // parameter for computing fan pwm
int N = 256;
int ctr_buf = 0;
int pwm6;
int pwm7;
int pwm8;
uint32_t PSMODSTAT;
uint32_t PSFLTSTAT;
unsigned long tk=0; // time in milliseconds updated once each loop
//unsigned long tk1=0;
//unsigned long tk2=0;
//unsigned long tk3=0;
//unsigned long clk1 = 0; // updates every ? s for housekeeping comm
//unsigned long clk2 = 0; // updates every ? s for circular buffer
int HBrate = 1000; // HB rate = 1000 ms if SD card reads good
unsigned long tk_HBrate = 0; //millis() tick value updated at rate set by HBrate
unsigned long tk_halfsec = 0; //millis() tick value updated every 0.5 s
unsigned long tk_1s = 0; //millis() tick value updated every 1 s
unsigned long tk_2s = 0; //millis() tick value updated every 2 s
unsigned long tk_onCH1 = 0; // millis() value at Channel 1 turn-on
unsigned long tk_onCH2 = 0; // millis() value at Channel 2 turn-on
//unsigned long clk6 = 0; // updates every ? s to stop and re-begin UDP
//unsigned long clk7 = 0; // updates every ? s to get loop index
//unsigned long clk8 = 0; // updates every ? s for web server
//unsigned long clk9 = 0; // updates every ? s for Temp sensors for fan control
//unsigned long clk10 = 0; // updates every ? s for fast heartbeat
//unsigned long clk11 = 0; // updtes every second for uptime computation
uint16_t ctr_packet = 0;
int turning_onCH1 = LOW;
int turning_onCH2 = LOW;
int changingpage = LOW;
int SDfail = 0;
int ON1faultCH1 = 0;
int ON1faultCH2 = 0;
int OWconverting = LOW;
int flag1=0;

//Interlock limits
float OVC_limits[4] = {-30.0, 30.0, -30.0, 30.0}; //CH1low,CH1high,CH2low,CH2high /////////////////////////////////////////
float OVT_limits[2] = {67.0, 67.0};//Heatsink 1 and 2. Max heatsink temperature at 24 A in termperature controlled rack
                                   // at 24 deg C is 42 deg C with fan at 75 % ////////////////

// pin assignments
int pin_global_inh = 23; // output to 24 V power supply
int pin_on1_cmd_CH1 = 22; // input from PSC
int pin_reset_cmd_CH1 = 25; // input from PSC
int pin_spare_cmd_CH1 = 24;
int pin_on1_cmd_CH2 = 36; // input from PSC
int pin_reset_cmd_CH2 = 39; // input from PSC
int pin_spare_cmd_CH2 = 38;
int pin_mod1_inh = 30; // output to 24 V power supply
int pin_mod2_inh = 33; // output to 24 V power supply
int pin_mod3_inh = 44; // output to 24 V power supply
int pin_mod4_inh = 47; // output to 24 V power supply
int pin_pwr_good_mod1 = 32; // input from 24 power supply
int pin_pwr_good_mod2 = 35; // input from 24 power supply
int pin_pwr_good_mod3 = 46; // input from 24 power supply
int pin_pwr_good_mod4 = 49; // input from 24 power supply
int pin_on_sts_CH1 = 26; // output to PSC
int pin_flt1_sts_CH1 = 29; // output to PSC
int pin_flt2_sts_CH1 = 28; // output to PSC
int pin_spare_sts_CH1 = 31; 
int pin_on_sts_CH2 = 40; // output to PSC
int pin_flt1_sts_CH2 = 43; // output to PSC
int pin_flt2_sts_CH2 = 42; // output to PSC
int pin_spare_sts_CH2 = 45;
int pin_HB = 48; // output to LED
int pin_L = 13;
int pin_flipclk = 9;
int pin_park1 = 5;
int pin_park2 = 2;
int pin_pwm_en1 = 10;
int pin_pwm_en2 = 11;
//missing pin_fanpwrsel12
int pin_park3 = 3;
int pin_park4 = 4;
int pin_pwm_en3 = 12;
int pin_pwm_en4 = 13;
int pin_fanpwrsel34 = 37;
int pulse =  LOW; // test pulse for A12 output


void setup() {
// put your setup code here, to run once:

pinMode(pin_global_inh, OUTPUT);
pinMode(pin_mod1_inh, OUTPUT);
pinMode(pin_mod2_inh, OUTPUT);
pinMode(pin_mod3_inh, OUTPUT);
pinMode(pin_mod4_inh, OUTPUT);
pinMode(pin_HB, OUTPUT);
pinMode(pin_L, OUTPUT);
pinMode(pin_pwr_good_mod1, INPUT);
pinMode(pin_pwr_good_mod2, INPUT);
pinMode(pin_pwr_good_mod3, INPUT);
pinMode(pin_pwr_good_mod4, INPUT);
pinMode(pin_on1_cmd_CH1, INPUT);
pinMode(pin_reset_cmd_CH1, INPUT);
pinMode(pin_spare_cmd_CH1, INPUT);
pinMode(pin_on1_cmd_CH2, INPUT);
pinMode(pin_reset_cmd_CH2, INPUT);
pinMode(pin_spare_cmd_CH2, INPUT);
pinMode(pin_on_sts_CH1, OUTPUT);
pinMode(pin_flt1_sts_CH1, OUTPUT);
pinMode(pin_flt2_sts_CH1, OUTPUT); 
pinMode(pin_spare_sts_CH1, OUTPUT);
pinMode(pin_on_sts_CH2, OUTPUT);
pinMode(pin_flt1_sts_CH2, OUTPUT);
pinMode(pin_flt2_sts_CH2, OUTPUT);
pinMode(pin_spare_sts_CH2, OUTPUT);
pinMode( 6, OUTPUT); // pwm output
pinMode( 7, OUTPUT); // pwm output
pinMode( 8, OUTPUT); // pwm output
pinMode(pin_flipclk, OUTPUT);
pinMode(pin_park1, OUTPUT);
pinMode(pin_park2, OUTPUT);
pinMode(pin_pwm_en1, OUTPUT);
pinMode(pin_pwm_en2, OUTPUT);
pinMode(pin_park3, OUTPUT);
pinMode(pin_park4, OUTPUT);
pinMode(pin_pwm_en3, OUTPUT);
pinMode(pin_pwm_en4, OUTPUT);
pinMode(pin_fanpwrsel34, OUTPUT);
pinMode(A12, OUTPUT); // DIO test pin for measuring loop rate (TP1)

//PWM Registers Setup
TCCR4A = 0; // timer/counter control register
TCCR4B = 0;
TCNT4  = 0;
// Mode 10: phase correct PWM with ICR4 as Top (= F_CPU/2/25000). 
// OC4C as Non-Inverted PWM output
ICR4   = (F_CPU/25000)/2; // Top value is timer rollover count.
OCR4C  = ICR4/2;                    // output compare value. default: about 50:50
OCR4A  = ICR4/2;
OCR4B  = ICR4/2;
//TCCR4A = _BV(COM4C1) | _BV(WGM41);
//TCCR4B = _BV(WGM43) | _BV(CS40);
TCCR4A = _BV(COM4A1) | _BV(COM4B1) | _BV(COM4C1);         // Enable the PWM outputs OC4A, OC4B and OC4C on digital pins 6, 7 and 8
//TCCR4B = _BV(WGM43) | _BV(CS41);                          // Set phase and frequency correct PWM and prescaler of 8 on timer 4
TCCR4B = _BV(WGM43) | _BV(CS40);

//Serial.begin(9600);
Serial.begin(115200);
Wire.begin();
Wire.setClock(400000);

Serial.println("");
Serial.println("");
Serial.println("2 Channel 864 W Bipolar Power Converter");
Serial.print("Firmware build: ");
Serial.println(buildnumber, 0);
Serial.println("");

if( readSD() ) // get MAC, IP address, 1-wire IDs, and Hall coefficients. If fail, sumfault and fast heartbeat
{
  SDfail = 1;
  HBrate = 200;
}

//Start the Ethernet and UDP
IPAddress ip(ipOctet[0],ipOctet[1],ipOctet[2],ipOctet[3]); // 
Ethernet.begin(mac,ip);
//Udp.begin(port);
//server.begin(); // web server

digitalWrite(pin_HB, heartbeat);

//Serial.print("IP Address: ");
//Serial.println(Ethernet.localIP());
//Serial.println("");


//I2C Stuff
scanI2C(); // look for devices on i2c bus

Serial.print("Discovering One-Wire Sensors... ");
DS18B20_Device.begin();
//delay(500);
SensorCount = DS18B20_Device.getDeviceCount(); // returns the number of devices found on the bus
Serial.print(SensorCount, DEC);
Serial.println(" detected One-Wire Sensors ");
for (int i = 0; i < SensorCount; i++) //Counts and prints out each sensor 64-bit ID
{
  Serial.print("Sensor");
  Serial.print(" ");
  Serial.print(i + 1);
  Serial.print(" Address: "); 
  DS18B20_Device.getAddress(Addr, i); //Send command to get addresses for sensors
  printAddress(Addr); //print address function below.
  //delay(1000);
}
Serial.print("\n");
Serial.println("Discovered 1-Wire IDs should match IDs on SD card!");
Serial.print("\n\n\n");

resetFlag=1;
uptime_H=0;
uptime_L0=0;
uptime_L1=0;

wdt_enable(WDTO_2S);

turning_onCH1 = LOW;
turning_onCH2 = LOW;

/*
digitalWrite(pin_pwm_en1, HIGH);
digitalWrite(pin_pwm_en2, HIGH);
digitalWrite(pin_pwm_en3, HIGH);
digitalWrite(pin_pwm_en4, HIGH);
digitalWrite(pin_park1, HIGH);
digitalWrite(pin_park3, HIGH);
digitalWrite(pin_mod1_inh, HIGH); 
digitalWrite(pin_mod2_inh, HIGH);
digitalWrite(pin_mod3_inh, HIGH); 
digitalWrite(pin_mod4_inh, HIGH);
*/

digitalWrite(pin_global_inh, LOW);

} // end setup()



void loop() {
// put your main code here, to run repeatedly:
//tk1 = tk;
tk = millis();
Udp.begin(port);

if(uptime_s > 3 && !flag1){
//Report firmware
Wire.beginTransmission(PMBusAddr);
Wire.write(0xc4); 
Wire.endTransmission(false);
Wire.requestFrom(PMBusAddr, 16);
for (int i=0; i<16; i++) PMBusData[i] = (uint8_t)Wire.read();
Serial.print("Frame Firmware: ");
for (int i=0; i<16; i++){
Serial.print(PMBusData[i],HEX);
Serial.print(" ");
}
Serial.print("    ");
for (int i=0; i<16; i++) Serial.print((char)PMBusData[i]);
Serial.println();
  
Wire.beginTransmission(PMBusAddr);
Wire.write(0xc5); 
Wire.endTransmission(false);
Wire.requestFrom(PMBusAddr, 16);
for (int i=0; i<16; i++) PMBusData[i] = (uint8_t)Wire.read();
Serial.print("Module Firmware:  ");
for (int i=0; i<16; i++) {
Serial.print(PMBusData[i],HEX);
Serial.print(" ");
}
Serial.print("    ");
for (int i=0; i<16; i++) Serial.print((char)PMBusData[i]);
Serial.println();

/*
//Report fan status
 Wire.beginTransmission(PMBusAddr);
Wire.write(0x81); 
Wire.endTransmission(false);
Wire.requestFrom(PMBusAddr, 2);
for (int i=0; i<2; i++) PMBusData[i] = (uint8_t)Wire.read();
Serial.print("Fan Status: ");
Serial.println(PMBusData[0],HEX);
*/
flag1 = 1; // lockout subsequent calls
}

/*
if(tk - clk8 > 1000){
// listen for incoming clients
  EthernetClient client = server.available();
  if (client) {
  tk2=millis();  
  webpage(client, HKdata);
  tk3=millis();
  }  
clk8 = tk;
}
*/

digitalWrite(A12, pulse); // output bit toggles every loop (TP1)
pulse = !pulse;

if (tk>5000) resetFlag=0; // clear flag 5 s after restart

//heartbeat pulse
if ( tk - tk_HBrate > HBrate ) {
wdt_reset();
if (heartbeat == LOW) heartbeat = HIGH;
else
heartbeat = LOW;
digitalWrite(pin_HB, heartbeat);
digitalWrite(pin_flt2_sts_CH1, heartbeat);
digitalWrite(pin_flt2_sts_CH2, heartbeat);
//digitalWrite(pin_L, heartbeat);
tk_HBrate = tk;
}

//if (flipclk == 1){
//  flipclk=0;
digitalWrite(pin_flipclk, LOW);
//}

//flipclk pulse
//delay start of flipclk 2 s after reboot to give Atemga time to assert Delta PS inhibit lines.
//if ( tk > 5000 && tk - tk_halfsec > 500 ) {
//flipclk = 1;
//digitalWrite(pin_flipclk, HIGH);
//tk_halfsec = tk;
//}


// To detect presence of ON1 pulse train, use two timers, one to detect rising edges,
// one to detect falling edges. Use millis() to record time instant of each edge.
// Look at time difference between rising and falling edges. ON1 is asserted as long
// as time difference is less than specified amount.
ON1cmdCH1[0] = ON1cmdCH1[1];
ON1cmdCH2[0] = ON1cmdCH2[1];
ON1cmdCH1[1] = digitalRead(pin_on1_cmd_CH1);
ON1cmdCH2[1] = digitalRead(pin_on1_cmd_CH2);

sts_mod[0] = digitalRead(pin_pwr_good_mod1);
sts_mod[1] = digitalRead(pin_pwr_good_mod2);
sts_mod[2] = digitalRead(pin_pwr_good_mod3);
sts_mod[3] = digitalRead(pin_pwr_good_mod4);

//If power converter is turned off by command or by fault, disable HBridges and DC supply. 
//Since Hbridges turn off first, magnet current will freewheel through DC supply acting as 
//a sink until the DC supply turns off. 
//Then remaining energy will be either given to cap or dissipated in TVS if cap voltage
//goes too high. 
//ON fault happens if DC supplies fail to turn on when ON1 command changes from OFF to ON or
//if DC supply is on when ON1 is in the OFF state.
//If DC supply status readbacks go low while power converter is on, power converter should
//take no action because it might be able to ride through power dip. PSC should turn power 
//converter off if current loop error goes out of bounds. Power converter needs to be able 
//to tolerate discharge/dynamic currents in cap during AC voltage disturbances.

// If ON1 command detected (rising edge, pulse train, etc) and no fault...
// unpark and pwm enable are active low
// turn on DC supplies before enabling PWM so PWM electronics isn't coming up with PWM enabled.

CH1_ON[0] = CH1_ON[1];
if(tmr_ON1CH1turnOFF != 0) tmr_ON1CH1turnOFF--; //decrement each loop until zero
if(tmr_ON1CH1turnOFF == 0) CH1_ON[1] = 0; // if counter reaches zero (no ON1 pulses), turn off
if(ON1cmdCH1[0]==LOW && ON1cmdCH1[1]==HIGH) tmr_ON1CH1turnOFF = 40; // if ON1 pulse train detected, reset to 40
if(ON1cmdCH1[0]==LOW && ON1cmdCH1[1]==HIGH && tmr_ON1CH1turnON < 5) tmr_ON1CH1turnON++;
if(tmr_ON1CH1turnON == 5) CH1_ON[1] = 1;
if(CH1_ON[1]) tmr_ON1CH1turnON = 0;


CH2_ON[0] = CH2_ON[1];
if(tmr_ON1CH2turnOFF != 0) tmr_ON1CH2turnOFF--; //decrement each loop until zero
if(tmr_ON1CH2turnOFF == 0) CH2_ON[1] = 0; // if counter reaches zero (no ON1 pulses), turn off
if(ON1cmdCH2[0]==LOW && ON1cmdCH2[1]==HIGH) tmr_ON1CH2turnOFF = 40; // if ON1 pulse train detected, reset to 40
if(ON1cmdCH2[0]==LOW && ON1cmdCH2[1]==HIGH && tmr_ON1CH2turnON < 5) tmr_ON1CH2turnON++;
if(tmr_ON1CH2turnON == 5) CH2_ON[1] = 1;
if(CH2_ON[1]) tmr_ON1CH2turnON = 0;



//if(ON1cmdCH1[0]==LOW && ON1cmdCH1[1]==HIGH && !sumfault_CH1) { // if channel 1 turned on...
  if(CH1_ON[0]==0 && CH1_ON[1]==1 && !sumfault_CH1) {
  digitalWrite(pin_mod1_inh, LOW);
  digitalWrite(pin_mod2_inh, LOW);
  tk_onCH1 = tk; // instant of CH1 turn-on
  turning_onCH1 = HIGH;
}
//if(ON1cmdCH2[0]==LOW && ON1cmdCH2[1]==HIGH && !sumfault_CH2) { // if channel 2 turned on...
  if(CH2_ON[0]==0 && CH2_ON[1]==1 && !sumfault_CH2) {
  digitalWrite(pin_mod3_inh, LOW);
  digitalWrite(pin_mod4_inh, LOW);
  tk_onCH2 = tk; // instant of CH2 turn-on
  turning_onCH2 = HIGH;
}

if(turning_onCH1 && tk - tk_onCH1 > 2000 && tk - tk_onCH1 < 2200){
  if(sts_mod[0] && sts_mod[1]){
  digitalWrite(pin_pwm_en1, LOW);
  digitalWrite(pin_pwm_en2, LOW); // single channel pwm board uses pwm1 and pwm2 for the two bridges for chasssis CH1
  }
}
if(turning_onCH1 && tk - tk_onCH1 > 4000 && tk - tk_onCH1 < 4200 && !ON1faultCH1)
digitalWrite(pin_park1, LOW); // single channel pwm board uses park1 for chassis CH1

if(tk - tk_onCH1 > 4200) turning_onCH1 = LOW;


if(turning_onCH2 && tk - tk_onCH2 > 2000 && tk - tk_onCH2 < 2200){
  if(sts_mod[2] && sts_mod[3]){
  digitalWrite(pin_pwm_en3, LOW);
  digitalWrite(pin_pwm_en4, LOW); // single channel pwm board uses pwm3 and pwm4 for the two bridges for chassis CH2
  }
}
if(turning_onCH2 && tk - tk_onCH2 > 4000 && tk - tk_onCH2 < 4200 && !ON1faultCH2)
digitalWrite(pin_park3, LOW); // single channel pwm board uses park3 for chassis CH2

if(tk - tk_onCH2 > 4200) turning_onCH2 = LOW;
//

//if ON1 level is low or fault, park and pwm off, modules off
//if(ON1cmdCH1[1]==LOW || sumfault_CH1){
if(CH1_ON[1]==0 || sumfault_CH1){
  CH1_ON[1] = 0; //////////////////////////////////////////////////////////////// latch sumfault
  turning_onCH1 = LOW; 
  digitalWrite(pin_mod1_inh, HIGH); 
  digitalWrite(pin_mod2_inh, HIGH);
  digitalWrite(pin_pwm_en1, HIGH);
  digitalWrite(pin_pwm_en2, HIGH);
  digitalWrite(pin_park1, HIGH);
  }

//if ON1 level is low or fault, park and pwm off, modules off
//if(ON1cmdCH2[1]==LOW || sumfault_CH2){
  if(CH2_ON[1]==0 || sumfault_CH2){
  CH2_ON[1] = 0;  /////////////////////////////////////////////////////////////// latch sumfault
  turning_onCH2 = LOW;
  digitalWrite(pin_mod3_inh, HIGH); 
  digitalWrite(pin_mod4_inh, HIGH);
  digitalWrite(pin_pwm_en3, HIGH);
  digitalWrite(pin_pwm_en4, HIGH);
  digitalWrite(pin_park3, HIGH);
  }

digitalWrite(pin_flipclk, HIGH);

// ON/Module fault
if( (HKdata[18]>23.0 && !sts_mod[0]) ||
    (HKdata[20]>23.0 && !sts_mod[1]) ||
    (!(sts_mod[0]&&sts_mod[1]) && CH1_ON[1]) && 
    tmr_ON1fltCH1 ) tmr_ON1fltCH1--;
else tmr_ON1fltCH1=5000;
if(!tmr_ON1fltCH1) ON1faultCH1=1;

if( (HKdata[22]>23.0 && !sts_mod[2]) ||
    (HKdata[24]>23.0 && !sts_mod[3]) ||
    (!(sts_mod[2]&&sts_mod[3]) && CH2_ON[1]) &&
    tmr_ON1fltCH2 ) tmr_ON1fltCH2--;
else tmr_ON1fltCH2=5000;
if(!tmr_ON1fltCH2) ON1faultCH2=1;


//write output status bits
if(sts_mod[0] == HIGH && sts_mod[1] == HIGH && CH1_ON[1] && !sumfault_CH1) digitalWrite(pin_on_sts_CH1, HIGH);
if(sts_mod[0] == LOW || sts_mod[1] == LOW || !CH1_ON[1] || sumfault_CH1) digitalWrite(pin_on_sts_CH1, LOW);
if(sts_mod[2] == HIGH && sts_mod[3] == HIGH && CH2_ON[1] && !sumfault_CH2) digitalWrite(pin_on_sts_CH2, HIGH);
if(sts_mod[2] == LOW || sts_mod[3] == LOW || !CH2_ON[1] || sumfault_CH2) digitalWrite(pin_on_sts_CH2, LOW);

digitalWrite(pin_flt1_sts_CH1, sumfault_CH1);
digitalWrite(pin_flt1_sts_CH2, sumfault_CH2);


// only reset fault if ON1 command pulse timer has timed out /////////////////////////// added 6/9/23
if ( (digitalRead(pin_reset_cmd_CH1) == HIGH) && (tmr_ON1CH1turnOFF == 0) ) {
  ovc_CH1 = 0;
  hall_mismatch_CH1 = 0;
  ovt1 = 0;
  ON1faultCH1=0;
  tmr_ON1fltCH1=5000;
}

// only reset fault if ON1 command pulses have stopped /////////////////////////// added 6/9/23
if ( (digitalRead(pin_reset_cmd_CH2) == HIGH) && (tmr_ON1CH2turnOFF == 0) ) {
  ovc_CH2 = 0;
  hall_mismatch_CH2 = 0;
  ovt2 = 0;
  ON1faultCH2=0;
  tmr_ON1fltCH2=5000;
}

//ADCs
for (int j=0; j<8; j++){
ADCs[j] = analogRead(chan_list[j]);
 
// if channel is off, set HALL sensor and Vout readbacks to zero. This will avoid getting a full scale negative
// readback when channel is off.
if (j < 4){
ADC_[j] = (float)ADCs[j]*1.0;  
I_hall[j] = ( (float)(ADC_[j])*hall_cal[0]-512.0)*0.04883;
if (sts_mod[0] == LOW && sts_mod[1] == LOW) I_hall[j] = 0; 
} 

if (j >= 4 && j < 8){
ADC_[j] = (float)ADCs[j]*1.0;  
I_hall[j] = ( (float)(ADC_[j])*hall_cal[1]-512.0)*0.04883;
if (sts_mod[2] == LOW && sts_mod[3] == LOW) I_hall[j] = 0; 
} 
}

/*
//compute I_hall moving averages
//put I_hall[] values into shift register
for (int j=0; j<8; j++){
for (int i=1; i<5; i++) I_hall_buf[j][i]=I_hall_buf[j][i-1]; //move each element into next higher index
I_hall_buf[j][0] = I_hall[j]; // put new sample in index 0
}

//take mean of shift register values
for (int i=0; i<8; i++) I_hall_avg[i]=0.0; // initialize mean totalizer to zero
for (int j=0; j<8; j++){
for (int i=0; i<5; i++) I_hall_avg[j] = I_hall_avg[j] + I_hall_buf[j][i];
}  

for (int j=0; j<8; j++){
I_hall_avg[j] = I_hall_avg[j]/5.0;
}
*/

for(int j=0; j<8; j++){
I_hall_buf1[j][0] = I_hall_buf1[j][1]; //buf1[j][0] get last sample
I_hall_buf1[j][1] = I_hall[j]*0.01 + I_hall_buf1[j][0]*0.99; // multiply last sample by exp(-Ts/tau), e.g. exp(-0.001/0.1)=0.99
}



//OVC
// tmr1 is for CH1 OVC, tmr2 is for CH1 mismatch
// tmr3 is for CH2 OVC, tmr4 is for CH2 mismatch
if ( (I_hall[0]+I_hall[2]) > OVC_limits[1] || (I_hall[0]+I_hall[2]) < OVC_limits[0]) tmr1 +=1;
else tmr1=0;
if ( abs(I_hall[0]-I_hall[2]) > 4.0 || abs(I_hall[1]-I_hall[3]) > 4.0 ) tmr2 +=1; // 4 A mismatch
else tmr2=0;

if ( (I_hall[4]+I_hall[6]) > OVC_limits[3] || (I_hall[4]+I_hall[6]) < OVC_limits[2]) tmr3 +=1;
else tmr3=0;
if ( abs(I_hall[4]-I_hall[6]) > 4.0 || abs(I_hall[5]-I_hall[7]) > 4.0 ) tmr4 +=1; // 4 A mismatch
else tmr4=0;

if (tmr1 > 2) ovc_CH1 = 1;
if (tmr2 > 50) {
  ovc_CH1 = 1;
  hall_mismatch_CH1 = 1;
  }
if (tmr3 > 2) ovc_CH2 = 1;
if (tmr4 > 50) {
  ovc_CH2 = 1;
  hall_mismatch_CH2 = 1;
  }


/*
if ( tk - clk2 > 2000) {

if (T_CH1_fan[1] < 24) pwm6 = 180;
if (T_CH1_fan[1] >= 24 && T_CH1_fan[0] < 24) pwm6 = 150; // if > 24 and rising
if (T_CH1_fan[1] >= 32 && T_CH1_fan[0] < 32) pwm6 = 80;
if (T_CH1_fan[1] >= 40 && T_CH1_fan[0] < 40) pwm6 = 40;
if (T_CH1_fan[1] >= 48 ) pwm6 = 3;
if (T_CH1_fan[1] <= 44 && T_CH1_fan[0] > 44) pwm6 = 40;
if (T_CH1_fan[1] <= 36 && T_CH1_fan[0] > 36) pwm6 = 80;
if (T_CH1_fan[1] <= 28 && T_CH1_fan[0] > 28) pwm6 = 120;
if (T_CH1_fan[1] == -99 ) pwm6 = 150; // if no 1-wire sensor

if (T_CH2_fan[1] < 24) pwm7 = 180;
if (T_CH2_fan[1] >= 24 && T_CH2_fan[0] < 24) pwm7 = 150; // if > 24 and rising
if (T_CH2_fan[1] >= 32 && T_CH2_fan[0] < 32) pwm7 = 80;
if (T_CH2_fan[1] >= 40 && T_CH2_fan[0] < 40) pwm7 = 40;
if (T_CH2_fan[1] >= 48 ) pwm7 = 3;
if (T_CH2_fan[1] <= 44 && T_CH2_fan[0] > 44) pwm7 = 40;
if (T_CH2_fan[1] <= 36 && T_CH2_fan[0] > 36) pwm7 = 80;
if (T_CH2_fan[1] <= 28 && T_CH2_fan[0] > 28) pwm7 = 120;
if (T_CH2_fan[1] == -99 ) pwm7 = 150; // if no 1-wire sensor

pwm8 = 319;
pwm25k(pwm6, pwm7, pwm8); 
clk2 = tk;
}
*/

//OVT
if (T1 > OVT_limits[0]) ovt1 = 1; 
if (T2 > OVT_limits[1]) ovt2 = 1;

sumfault_CH1 = ovc_CH1 + ovt1 + SDfail + ON1faultCH1;
sumfault_CH2 = ovc_CH2 + ovt2 + SDfail + ON1faultCH2;
if(sumfault_CH1) sumfault_CH1=1;
if(sumfault_CH2) sumfault_CH2=1;

//1 second stuff
if(tk - tk_1s > 1000){

// Uptime update
uptime_L0 = uptime_L1;
uptime_L1 = micros();
if (uptime_L1 < uptime_L0) uptime_H++; // if rollover, increment high word
uptime_s = (uptime_H*pow(2, 32) + uptime_L1)/1e6;

//get loop index for measuring loop rate
i1=i2;
i2=i;


//I2C stuff
//Delta PS PMBus data

//delay(20);
//Report temperature //36-38
Wire.beginTransmission(PMBusAddr);
Wire.write(0x8D); 
Wire.endTransmission(false);
Wire.requestFrom(PMBusAddr, 3);
for (int i=0; i<3; i++) PMBusData[i] = (uint8_t)Wire.read();
HKdata[35] = (float)((PMBusData[0] + 256*PMBusData[1])/10.0);

//Report fan speed // 39-41
Wire.beginTransmission(PMBusAddr);
Wire.write(0x90); 
Wire.endTransmission(false);
Wire.requestFrom(PMBusAddr, 3);
for (int i=0; i<3; i++) PMBusData[i] = (uint8_t)Wire.read();
HKdata[38] = PMBusData[0] + 256*PMBusData[1];

//Change page. CH1 is pages 3 & 4. CH2 is pages 1 & 2.
Wire.beginTransmission(PMBusAddr);
for (int i=0; i<3; i++) Wire.write(PMBusCmd_SetPage[k][i]);
Wire.endTransmission(true);

changingpage = HIGH;
tk_1s = tk;
}

if(changingpage == HIGH && (tk - tk_1s) > 100) {
//delay(td1);  // without delay stuck on page 0
switch(k){
  case 1:
  //Report page 1 output voltage
  Wire.beginTransmission(PMBusAddr);
  Wire.write(0x8B); 
  Wire.endTransmission(false);
  Wire.requestFrom(PMBusAddr, 3);
  for (int i=0; i<3; i++) PMBusData[i] = (uint8_t)Wire.read();
  HKdata[24] = (PMBusData[0] + 256*PMBusData[1])/10.0;
  //Report page 1 output current
  Wire.beginTransmission(PMBusAddr);
  Wire.write(0x8C); 
  Wire.endTransmission(false);
  Wire.requestFrom(PMBusAddr, 3);
  for (int i=0; i<3; i++) PMBusData[i] = (uint8_t)Wire.read();
  HKdata[25] = (PMBusData[0] + 256*PMBusData[1])/10.0;
  k=2; // prepare for next case
  break;
      
  case 2:
  //Report page 2 output voltage
  Wire.beginTransmission(PMBusAddr);
  Wire.write(0x8B); 
  Wire.endTransmission(false);
  Wire.requestFrom(PMBusAddr, 3);
  for (int i=0; i<3; i++) PMBusData[i] = (uint8_t)Wire.read();
  HKdata[22] = (PMBusData[0] + 256*PMBusData[1])/10.0;
  //Report page 2 output current
  Wire.beginTransmission(PMBusAddr);
  Wire.write(0x8C); 
  Wire.endTransmission(false);
  Wire.requestFrom(PMBusAddr, 3);
  for (int i=0; i<3; i++) PMBusData[i] = (uint8_t)Wire.read();
  HKdata[23] = (PMBusData[0] + 256*PMBusData[1])/10.0;
  k=3; // prepare for next case
  break;
     
  case 3:
  //Report page 3 output voltage
  Wire.beginTransmission(PMBusAddr);
  Wire.write(0x8B); 
  Wire.endTransmission(false);
  Wire.requestFrom(PMBusAddr, 3);
  for (int i=0; i<3; i++) PMBusData[i] = (uint8_t)Wire.read();
  HKdata[20] = (PMBusData[0] + 256*PMBusData[1])/10.0;
  //Report page 3 output current
  Wire.beginTransmission(PMBusAddr);
  Wire.write(0x8C); 
  Wire.endTransmission(false);
  Wire.requestFrom(PMBusAddr, 3);
  for (int i=0; i<3; i++) PMBusData[i] = (uint8_t)Wire.read();
  HKdata[21] = (PMBusData[0] + 256*PMBusData[1])/10.0;  
  k=4; // prepare for next case
  break;
 
  case 4:
  //Report page 4 output voltage
  Wire.beginTransmission(PMBusAddr);
  Wire.write(0x8B); 
  Wire.endTransmission(false);
  Wire.requestFrom(PMBusAddr, 3);
  for (int i=0; i<3; i++) PMBusData[i] = (uint8_t)Wire.read();
  HKdata[18] = (PMBusData[0] + 256*PMBusData[1])/10.0;
  //Report page 4 output current
  Wire.beginTransmission(PMBusAddr);
  Wire.write(0x8C); 
  Wire.endTransmission(false);
  Wire.requestFrom(PMBusAddr, 3);
  for (int i=0; i<3; i++) PMBusData[i] = (uint8_t)Wire.read();
  HKdata[19] = (PMBusData[0] + 256*PMBusData[1])/10.0;  
  k=1; // prepare for next case
  break;

  default :
  k = 1;
} // end switch(k)   

changingpage = LOW;

if(!CH1_ON[1] && !CH2_ON[1] && uptime_s>3.5){
//Set fan
Wire.beginTransmission(PMBusAddr);
for (int i=0; i<4; i++) Wire.write(PMBusCmd_SetFan2_30pct[i]);
Wire.endTransmission(false);

Serial.println("Setting fan1 duty");
}
}
//end Delta PS PMBus data



//2 second stuff
if(tk - tk_2s > 2000) {
//I2C 1-Wire stuff
//trigger 1-wire data conversions every 2 s
//DS18B20_Device.requestTemperaturesByAddress(sensor_1);
//DS18B20_Device.requestTemperaturesByAddress(sensor_2);
DS18B20_Device.requestTemperatures();
OWconverting = HIGH;
tk_2s = tk;

}

// retrieve 1-wire data
//if(tk - tk_2s > 1500 && OWconverting == HIGH || turning_onCH1 || turning_onCH2) {
if(tk - tk_2s > 1500 && OWconverting == HIGH) {
  T1 = getTemperature(sensor_1);
  T2 = getTemperature(sensor_2); 
  OWconverting = LOW;

//put T1, T2 values into shift register
for (int i=1; i<2; i++){
  T1_buf[i]=T1_buf[i-1];
  T2_buf[i]=T2_buf[i-1];
}
T1_buf[0] = T1;
T2_buf[0] = T2;

//take mean of shift register values
T1_avg = 0;
T2_avg = 0;
for (int i=0; i<2; i++){
  T1_avg = T1_avg + T1_buf[i];
  T2_avg = T2_avg + T2_buf[i];
}
T1_avg = T1_avg/2.0;
T2_avg = T2_avg/2.0;


//PWM fan control
T1_err = T1_avg - T_set;
pwm6 = Dmax - ceil( (float)(Dmax - Dmin)/T_errmax*T1_err );
if(T1_err < 0.0) pwm6 = Dmax;
if(T1_err > T_errmax) pwm6 = Dmin;


T2_err = T2_avg - T_set;
pwm7 = Dmax - ceil( (float)(Dmax - Dmin)/T_errmax*T2_err );
if(T2_err < 0.0) pwm7 = Dmax;
if(T2_err > T_errmax) pwm7 = Dmin;

if(turning_onCH1) pwm6 = 80;
if(turning_onCH2) pwm7 = 80;
pwm8 = Dmax;
pwm25k(pwm6, pwm7, pwm8); 
  
}


// End I2C Stuff

/*
//every 30 s
if(tk - clk9 > 30000){
  T_CH1_fan[0] = T_CH1_fan[1];// put temperature reading from 30 s ago into [0]
  T_CH1_fan[1] = T_CH1;// put current temperature reading into [1]
  T_CH2_fan[0] = T_CH2_fan[1];
  T_CH2_fan[1] = T_CH2;
  clk9 = tk;
}
*/

PSMODSTAT= sts_mod[0] + sts_mod[1]*2 + sts_mod[2]*4 + sts_mod[3]*8;
PSFLTSTAT= sumfault_CH1 + sumfault_CH2*2 +
           ovc_CH1*16 + ovc_CH2*32 + hall_mismatch_CH1*256 + hall_mismatch_CH2*512 +
           ovt1*4096 + ovt2*8192 +
           SDfail*32768 + heartbeat*65536 + resetFlag*131072 +
           ON1faultCH1*262144 + ON1faultCH2*524288;



HKdata[0] = 1000.0;
HKdata[1] = I_hall_buf1[0][1];
HKdata[2] = I_hall_buf1[1][1];
HKdata[3] = I_hall_buf1[2][1];
HKdata[4] = I_hall_buf1[3][1];
HKdata[5] = I_hall_buf1[4][1];
HKdata[6] = I_hall_buf1[5][1];
HKdata[7] = I_hall_buf1[6][1];
HKdata[8] = I_hall_buf1[7][1];
HKdata[9] = 0;
HKdata[10] = 0;
HKdata[11] = 0;
HKdata[12] = 0;
HKdata[13] = 0;
HKdata[14] = 0;
HKdata[15] = 0;
HKdata[16] = 0;
HKdata[17] = 0;

HKdata[44] = T1_avg;
HKdata[45] = T2_avg;

HKdata[48] = (float)(Dmax-pwm6)/Dmax*100.0;
HKdata[49] = (float)(Dmax-pwm7)/Dmax*100.0;

HKdata[52] = PSMODSTAT;
HKdata[53] = PSFLTSTAT;

HKdata[54] = mod_serial; // PS Model, S/N
HKdata[55] = buildnumber; // firmware version

HKdata[56] = i2-i1; // loops/s
HKdata[57] = ctr_packet;
HKdata[58] = uptime_s;
HKdata[59] = 1001.0;

/*
//Send Housekeeping Packet to Serial Port
if ( (tk - clk1) >= 200 ){
for (int i=0; i<86; i++){
Serial.print(HKdata[i]);
Serial.print(",");
}
Serial.print("\n");
clk1 = tk;
}
*/

//Send Housekeeping Packet to Ethernet with UDP protocol
int packetSize = Udp.parsePacket();

//if packet received
if(packetSize){
   Udp.read(packetBuffer, 100);
  //Serial.print(packetBuffer);
  
  // if message = Loop, send out housekeeping data
  packetBuffer[4] = '\0'; //terminate string after 4 characters to exclude UDP message padding
  if(strcmp(packetBuffer, "Loop")==0){
  //tk2=millis();
remote_IP = Udp.remoteIP();
remote_port = Udp.remotePort();

  Udp.beginPacket(remote_IP, remote_port );
  //Udp.write(datagram);
  //recast array name HKdata from pointer to float array to pointer to char buffer
  Udp.write((char*)HKdata, sizeof(HKdata) );
  Udp.endPacket();
  //Serial.print("Sending UDP data to ");
  //Serial.print(remote_IP);
  //Serial.print(":");
  //Serial.println(remote_port);
  //Serial.println(datagram);
  
  //tk3=millis();
  //HBfast = !HBfast;
  n++;
  ctr_packet++;
  }
  
  memset(packetBuffer, 0, 100); //Clear out the buffer array
  Udp.stop();
  
}


if (Serial.available() > 0) {
  cmd = Serial.read();
  }

if (cmd == '1') {
  Wire.beginTransmission(PMBusAddr);
  for (int i=0; i<2; i++) Wire.write(PMBusCmd_ClearFaults[i]);
  Wire.endTransmission(true);
  delay(50);
  Serial.println("'Clear Faults' command sent");
  cmd = 0;
  }

if (cmd == '2') {
  digitalWrite(pin_global_inh, LOW);
  cmd = 0;
  }

if (cmd == '3') {
  digitalWrite(pin_mod1_inh, HIGH);
  digitalWrite(pin_mod2_inh, HIGH);
  cmd = 0;
  }

if (cmd == '4') {
  digitalWrite(pin_mod1_inh, LOW);
  digitalWrite(pin_mod2_inh, LOW);
  cmd = 0;
  }

if (cmd == '5') {
  digitalWrite(pin_mod3_inh, HIGH);
  digitalWrite(pin_mod4_inh, HIGH);
  cmd = 0;
  }

if (cmd == '6') {
  digitalWrite(pin_mod3_inh, LOW);
  digitalWrite(pin_mod4_inh, LOW);
  cmd = 0;
  }

// clear ovc faults
if (cmd == '7') {
  ovc_CH1 = 0;
  ovc_CH2 = 0;
  cmd = 0;
  }



i++;

}
