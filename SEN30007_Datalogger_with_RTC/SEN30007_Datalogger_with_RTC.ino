/***************************************************************************
* File Name: SEN30007_Datalogger_with_RTC.ino
* Processor/Platform: Arduino Uno R3 (tested)
* Development Environment: Arduino 1.8.3
*
* Designed for use with with Playing With Fusion Quad MAX31856 thermocouple
* Arduino shield, SEN-30007 (any TC type, configurable) and micro SD shield
* with RTC, IFB-11001. 
*
* Copyright © 2016-18 Playing With Fusion, Inc.
* SOFTWARE LICENSE AGREEMENT: This code is released under the MIT License.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
* DEALINGS IN THE SOFTWARE.
* **************************************************************************
* REVISION HISTORY:
* Author		      Date	      Comments
* J. Steinlage		2016May30   First revision
* J. Steinlage    2016Sep23   Added interval control
* J. Steinlage    2018Jul10   Remove reference to DR/FLT
*
* Playing With Fusion, Inc. invests time and resources developing open-source
* code. Please support Playing With Fusion and continued open-source
* development by buying products from Playing With Fusion!
* **************************************************************************
* ADDITIONAL NOTES:
* This file contains functions to initialize and run an Arduino Uno R3 in
* order to communicate with a MAX31856 single channel thermocouple breakout
* board. Funcionality is as described below:
*	- Configure Arduino to broadcast results via UART
* - Configure I2C for RTC communication
* - Configure SPI for MAX31856 and SD card communication
* - Call libraries to read thermocouple channels periodically, log results to SD card
*	- Broadcast results to COM port as they are logged
*  Circuit:
*    Arduino Uno   Arduino Mega  -->  Shield / Function
*    D10           D10           -->  SEN-30007 / Chip Select, TC0
*    D9            D9            -->  SEN-30007 / Chip Select, TC1
*    D8            D8            -->  SEN-30007 / Chip Select, TC2
*    D7            D7            -->  SEN-30007 / Chip Select, TC3
*    D6            D6            -->  IFB-11001 / Card Detect - SD
*    D5            D5            -->  IFB-11001 / Chip Select, SD
*    D4            D4            -->  unused
*    D3            D3            -->  unused
*    D2            D2            -->  unused
*    MOSI: pin 11  MOSI: pin 51  -->  Both Shields / SDI via ICSP 
*    MISO: pin 12  MISO: pin 50  -->  Both Shields / SDO via ICSP 
*    SCK:  pin 13  SCK:  pin 52  -->  Both Shields / SCLK via ICSP 
*    SCL           SCL           -->  IFB-11001    / SCL
*    SDA           SDA           -->  IFB-11001    / SDA
*    GND           GND           -->  GND
*    5V            5V            -->  5V (supply with same voltage as Arduino I/O, 5V)
*    VCCIO         VCCIO         -->  Vcc (supply with same voltage as Arduino I/O, 3.3-5V)
*    3.3V          3.3V          --> 3.3V (this is 3.3V output from on-board LDO. DO NOT POWER THIS PIN!
*    
*    ***** note, DRDY resistors must be removed from the MAX31856 Shield pins D3-D6, R0-R3
***************************************************************************/
#include "PlayingWithFusion_MAX31856.h"
#include "PlayingWithFusion_MAX31856_STRUCT.h"

#include "Wire.h"
#include "SD.h"
#include "RTClib.h"     // must be PWFusion version - includes MCP7941X chip on IFB-11001
#include "SPI.h"
#include "string.h"

// First things first... declare timing interval (time between logged samples)
// Settings is a number of 20ms loops... logger_interval = time(seconds) / 0.020
// minimum config should be 20 (0.2 seconds) for the MAX31856. Maximum timer setting 
// available is 655.65 seconds (65535 = 655.35 seconds)
uint16_t logger_interval = 500;    // setting of 500 logs one sample every 5 seconds

// MAX31856 shield pin definitions
uint8_t TC0_CS  = 10;
uint8_t TC1_CS  =  9;
uint8_t TC2_CS  =  8;
uint8_t TC3_CS  =  7;

// SD / RTC Logger shield pin definitions
uint8_t CS_SD   =  5;
uint8_t CD_SD   =  6;

// Thermocouple channel config (pass chip select to init function)
PWF_MAX31856  thermocouple0(TC0_CS);
PWF_MAX31856  thermocouple1(TC1_CS);
PWF_MAX31856  thermocouple2(TC2_CS);
PWF_MAX31856  thermocouple3(TC3_CS);

// RTC interface
RTC_MCP79410 rtc;

void setup()
{
  delay(1000);                            // give chip a chance to stabilize
  Serial.begin(115200);                   // set baudrate of serial port
  Serial.println("Playing With Fusion: Quad Thermocouple Data");
  Serial.println("Logger Example using SEN-30007 and IFB-11001");

  // setup for the the SPI library:
  SPI.begin();                            // begin SPI
  SPI.setClockDivider(SPI_CLOCK_DIV32);   // SPI speed to SPI_CLOCK_DIV32 (500kHz)
  SPI.setDataMode(SPI_MODE3);             // MAX31856 is a MODE3 device

  /**********    THERMOCOUPLE CHANNEL SETUP    **********/
  // call config command... options can be seen in the PlayingWithFusion_MAX31856.h file
  thermocouple0.MAX31856_config(K_TYPE, CUTOFF_60HZ, AVG_SEL_4SAMP, CMODE_AUTO);
  thermocouple1.MAX31856_config(K_TYPE, CUTOFF_60HZ, AVG_SEL_4SAMP, CMODE_AUTO);
  thermocouple2.MAX31856_config(K_TYPE, CUTOFF_60HZ, AVG_SEL_4SAMP, CMODE_AUTO);
  thermocouple3.MAX31856_config(K_TYPE, CUTOFF_60HZ, AVG_SEL_4SAMP, CMODE_AUTO);

  /**********    REAL-TIME CLOCK SETUP    **********/
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC, halting program");
    while (1);  // No RTC, stop. Could light LED here to indicate failure
  }

  // Check to see if RTC is running. If not, set the date to settings below. If it is, assume
  // that it is set correctly, and doesn't need to be adjusted. REMOVE THIS CHECK IF YOU WANT 
  // TO OVERRIDE THE CURRENT RTC SETTINGS!
  if (! rtc.isrunning()) {
    Serial.println("Setting RTC date/time");
    // following line sets the RTC to the date & time this sketch was compiled
    //                    Y   M  D   H   M   S    enable battery backup
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)), MCP7941X_BATT_BKUP_EN);
    // To set the RTC with an explicit date & time, see below
    // For May 31, 2016 at 1:23:16 : 
    //                      Y   M  D   H   M   S    enable battery backup
    //rtc.adjust(DateTime(2016, 5, 31, 1, 23, 16), MCP7941X_BATT_BKUP_EN);
  }
  else{Serial.println("RTC running, no time adjust");}
  
  /**********    SD CARD SETUP    **********/
  SPI.setDataMode(SPI_MODE0);             // SD card is a Mode 0 device
  delay(1000);
  pinMode(6, INPUT);
  uint8_t CD_val;
  // Read Card Detect input
  CD_val = digitalRead(6);
  if(1 == CD_val){Serial.println("SD Card installed");}
  else{Serial.println("SD Card not installed");}
  Serial.print("SD card Init...");
  // see if the card is present and can be initialized. Check both SD init and Card Detect
  if (!SD.begin(CS_SD)) {
    Serial.println("Card init failed");
    // don't do anything more:
    while (1);  // initialization failed, stop. Could light LED here to indicate failure
  }
  Serial.println("Begin log file");
  
  // Build data string to write to log file
  String dataString = "Time,TC0_Stat,TC0_Tint,TC0_Temp,TC1_Stat,TC1_Tint,TC1_Temp,TC2_Stat,TC2_Tint,TC2_Temp,TC3_Stat,TC3_Tint,TC3_Temp";
  //  String dataString2 = "Int Temp Scaling, 0.015625";
  //  String dataString3 = "TC Scaling, 0.0078125";
  // Log new data to SD card
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  
  File dataFile = SD.open("DL06.csv", FILE_WRITE);
  
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    //    dataFile.println(dataString2);
    //    dataFile.println(dataString3);
    // print to the serial port too:
    Serial.println(dataString);
    //    Serial.println(dataString2);
    //    Serial.println(dataString3);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening file");
    dataFile.close();
    while(1);       // file couldn't be opened, stop. Could light LED here to indicate failure
  }
  pinMode(5, OUTPUT);
  digitalWrite(5,HIGH);
  // need to clock out 8 bits so the SD card will release the MISO line
  SPI.transfer(0xAA);
  // timer interrupt setup
  timer_interval_config();
  
}

// timer variables (_tick is cleared by by ISR, count_main keeps track of timing interval)
volatile uint8_t _tick = 0;
volatile uint8_t count_main = 0;

void loop()
{
  // wait for next timer event to trigger (20mS timer interval expected)
  while(_tick);
  // start by re-setting tick
  _tick = 1;
  
  if(count_main < logger_interval) // next main step not met, increment count and wait
  {
    count_main++;
  }
  else // next step met, run code
  {
    // start by clearing the count
    count_main = 0;

    // Get current time
    DateTime now = rtc.now();
  
    // swtich back to Mode 3 for MAX31856
    SPI.setDataMode(SPI_MODE3);             // MAX31856 is a MODE3 device
    
    // Get new temperature readings
    static struct var_max31856 TC_CH0, TC_CH1, TC_CH2, TC_CH3;
    double tmp;
    
    struct var_max31856 *tc_ptr;
  
    // Read CH 0
    tc_ptr = &TC_CH0;                             // set pointer
    thermocouple0.MAX31856_update(tc_ptr);        // Update MAX31856 channel 0
    // Read CH 1
    tc_ptr = &TC_CH1;                             // set pointer
    thermocouple1.MAX31856_update(tc_ptr);        // Update MAX31856 channel 1
    // Read CH 2
    tc_ptr = &TC_CH2;                             // set pointer
    thermocouple2.MAX31856_update(tc_ptr);        // Update MAX31856 channel 2
    // Read CH 3
    tc_ptr = &TC_CH3;                             // set pointer
    thermocouple3.MAX31856_update(tc_ptr);        // Update MAX31856 channel 3
      
    // Make sure SD card is still inserted
    uint8_t CD_val;
    CD_val = digitalRead(6);
    if(0 == CD_val){
      while(1);   // if card is uninstalled, stop. Could light LED here to indicate failure
    }
    // switch back to Mode 0 for SD card
    SPI.setDataMode(SPI_MODE0);             // SD card is a Mode 0 device
  
    // Build data string to write to log file
    // format of string:
    // Timestamp,TC0_Status,TC0_IntTemp,TC1_Temp,TC1_Status,TC1_IntTemp,TC2_Temp,TC2_Status,TC2_IntTemp,TC3_Temp,TC3_Status,TC0_IntTemp,TC3_Temp
    String dataString = "";
    // add current time to datastring
    dataString += String(now.year());
    dataString += "/";
    dataString += String(now.month());
    dataString += "/";
    dataString += String(now.day());
    dataString += " ";
    dataString += String(now.hour());
    dataString += ":";
    dataString += String(now.minute());
    dataString += ":";
    dataString += String(now.second());
    dataString += ",";
    // add Thermocouple data to dataString
    dataString += String(TC_CH0.status);
    dataString += ",";
    dataString += String(TC_CH0.ref_jcn_temp);  // must scale by 0.015625 in spreadsheet to get result
    dataString += ",";
    dataString += String(TC_CH0.lin_tc_temp);   // must scale by 0.0078125 in spreadsheet to get value
    dataString += ",";
    dataString += String(TC_CH1.status);
    dataString += ",";
    dataString += String(TC_CH1.ref_jcn_temp);  // must scale by 0.015625 in spreadsheet to get result
    dataString += ",";
    dataString += String(TC_CH1.lin_tc_temp);   // must scale by 0.0078125 in spreadsheet to get value
    dataString += ",";
    dataString += String(TC_CH2.status);
    dataString += ",";
    dataString += String(TC_CH2.ref_jcn_temp);  // must scale by 0.015625 in spreadsheet to get result
    dataString += ",";
    dataString += String(TC_CH2.lin_tc_temp);   // must scale by 0.0078125 in spreadsheet to get value
    dataString += ",";
    dataString += String(TC_CH3.status);
    dataString += ",";
    dataString += String(TC_CH3.ref_jcn_temp);  // must scale by 0.015625 in spreadsheet to get result
    dataString += ",";
    dataString += String(TC_CH3.lin_tc_temp);   // must scale by 0.0078125 in spreadsheet to get value
    
    // Log new data to SD card
    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    File dataFile = SD.open("DL06.csv", FILE_WRITE);
  
    // if the file is available, write to it:
    if (dataFile) {
      dataFile.println(dataString);
      dataFile.close();
      // print to the serial port too:
      Serial.println(dataString);
    }
    // if the file isn't open, pop up an error:
    else {
      Serial.println("error opening file");
      while(1);       // uncomment this line to stop execution if file couldn't be opened
    }
    // Get SD card to release data line.... write 0xAA to SPI
    digitalWrite(5,HIGH);
    SPI.transfer(0xAA);
  } // end of logging loop

}

void timer_interval_config(void)
{
  cli();
  //set timer2 interrupt at 100Hz (0.01 sec)
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 8khz increments
  OCR2A = 155;// = (16*10^6) / (100*1024) - 1 (must be <256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM01);
  // Set CS20 abd CS22 bits for 1024 prescaler
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);
  sei();//allow interrupts for timer to work
}

// Interrupt handler for TIMER2 (base 'tick' handler)
ISR(TIMER2_COMPA_vect){
   //the only thing we do here is clear the 'tick' variable
  _tick = 0;
}
