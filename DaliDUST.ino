//  License:
//    Copyright (C) 2017-2019, Emmanuel Kellner
//
//    This file is part of Dust
//
//    Dust is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    Dust is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with Dust.  If not, see <http://www.gnu.org/licenses/>.
//
//======================================================================

#include <NMEAGPS.h>
#include <GPSport.h> // choose the GPS module serial port
#include <Streamers.h> // common set of printing and formatting routines for GPS data in csv, printed to debug output device

#include <stdio.h>
#include <SPI.h>
#include "SdFat.h"

#include <BME280Spi.h>
#define DEVICE_PIN PA4

#define LED_BUILTIN PC13 // User LED pin definition

SdFat SD;
const uint8_t chipSelect = PB0; // SD Chip Select pin

long int logcount = 0; // log number
int ledstate = 0;

static NMEAGPS  gps; // parse received characters into gps.fix() data structure
static gps_fix  fix; // define set of GPS fix information. Will hold on to various pieces as received from RMC sentence

BME280Spi::Settings settings(DEVICE_PIN); // Default : forced mode, standby time = 1000 ms
                                          //           Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,
BME280Spi bme(settings);

//  Print the 32-bit integer degrees *as if* they were high-precision floats
static void printL( Print & outs, int32_t degE7 );
static void printL( int32_t degE7 )
{
  char degbuffer [20] = "";
  if (degE7 < 0) {  // Extract and print negative sign
    degE7 = -degE7;
//    outs.print( '-' );
    sprintf(degbuffer, "-");
  }
  int32_t deg = degE7 / 10000000L;   // Whole degrees
  sprintf(degbuffer, "%s%d.", degbuffer, deg);
//  outs.print( deg );
//  outs.print( '.' );
  degE7 -= deg*10000000L;   // Get fractional degrees
  int32_t factor = 1000000L;   // Print leading zeroes, if needed
  while ((degE7 < factor) && (factor > 1L)){
//    outs.print( '0' );
    sprintf(degbuffer, "%s0", degbuffer);
    factor /= 10L;
  }
//  outs.print( degE7 );   // Print fractional degrees
}


// WORK FUNCTION, ALL TIME CONSUMiNG TASKS GO THERE
static void doSomeWork()
{
  File logfile = SD.open("dust.txt", O_RDWR | O_APPEND);  // open logfile
  if (!logfile){                                          // if file doesn't exist
    logfile = SD.open("dust.txt", O_RDWR | O_CREAT);

    if(!logfile){                                         // Failed logfile opening/creation.
      if(Serial) Serial.println( F("Failed to open dust.txt") ); // Send message. ADD LED BLINKING
      ledstate = 1; // LED OFF
      digitalWrite(LED_BUILTIN, ledstate);
    }
      logfile.println("Line,Time,Satellites,Latitude,Longitude,Temperature[C],Relative_Humidity[%RH],Pressure[Pa],PM2.5, PM10,state");
  }

  logcount++; // increment logfile line number

  char buffer [128] =""; // buffer
  sprintf(buffer, "%d", logcount);

  if (fix.valid.location){
    int hour = 0;
    sprintf(buffer, "%s,20%d-%02d-%02dT%02d:%02d:%02d",  // append time data to buffer
            buffer,
            fix.dateTime.year,
            fix.dateTime.month,
            fix.dateTime.date,
            (fix.dateTime.hours + 8)%24,
            fix.dateTime.minutes,
            fix.dateTime.seconds
    );
    sprintf(buffer, "%s,%d,%f,%f", buffer, fix.satellites,fix.latitude(), fix.longitude() );
  } else {
    sprintf(buffer, "%s,,,,", buffer);
    ledstate = 1;
    digitalWrite(LED_BUILTIN, ledstate);
  }


  float temp(NAN), hum(NAN), pres(NAN);
  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);
  bme.read(pres, temp, hum, tempUnit, presUnit);
  sprintf(buffer, "%s,%.2f,%.2f,%f", buffer, temp, hum, pres);

  uint8_t mData = 0;
  uint8_t mPkt[10] = {0};
  uint8_t mCheck = 0;
  int pm25 = 111;
  int pm10 = 111;
  for( int i=0; i<10; ++i ) {
    mPkt[i] = Serial3.read();
  }
  if( 0xC0 == mPkt[1] ) {      // Check
    uint8_t sum = 0;
    for( int i=2; i<=7; ++i ) {
      sum += mPkt[i];
    }
    if( sum == mPkt[8] ) {
      uint8_t pm25Low   = mPkt[2];
      uint8_t pm25High  = mPkt[3];
      uint8_t pm10Low   = mPkt[4];
      uint8_t pm10High  = mPkt[5];
      pm10 = (int)round((( pm10High * 256.0 ) + pm10Low ) / 10.0);
      pm25 = (int)round((( pm25High * 256.0 ) + pm25Low ) / 10.0);
    } else {
      pm10 = 0;
      pm25 = 0;
    }
  } else {
    pm10 = 0;
    pm25 = 0;
  }

//  float pm25_corr, pm10_corr;
//  pm25_corr = pm25 / ( 1.0 + 0.48756*pow( ( hum / 100.0 ), 8.60068 ) );
//  pm10_corr = pm10 / ( 1.0 + 0.81559*pow( ( hum / 100.0 ), 5.83411 ) );
  
//  sprintf(buffer, "%s,%d,%d,%.2f,%.2f" , buffer, pm25, pm10, pm25_corr, pm10_corr);

  sprintf(buffer, "%s,%d,%d,%d" , buffer, pm25, pm10,ledstate);
  digitalWrite(LED_BUILTIN,ledstate);

  Serial.println(buffer);
  if( Serial1.available() ) Serial.println("oui");
 // Serial1.println("oui");
  logfile.println(buffer);
  logfile.close();

} // doSomeWork



// MAIN GPS PARSING LOOP
static void GPSloop() 
{
  while (gps.available( gpsPort )) {
    fix = gps.read();
    ledstate = 0; // LED ON
    digitalWrite(LED_BUILTIN, ledstate);
    doSomeWork();
  }
} // GPSloop


//---------- SETUP ------------//
void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin( 9600 ); // USB Serial
//  Serial1.begin( 9600 ); // Bluetooth
  Serial3.begin( 9600 ); // SDS018
  gpsPort.begin( 115200 ); // Serial2
  delay(2000);
Serial.println("OOOOO");
  gpsPort.write("$PMTK220,1000*1F\r\n");
  gpsPort.flush();
  digitalWrite(LED_BUILTIN, LOW); // LED ON

  if( !SD.begin(chipSelect) ){
    Serial.println( F("SD card corrupt or missing!") );
  } else {
    Serial.println( F("SD card OK!") );
  }

  SPI.begin();
  while(!bme.begin())
  {
      Serial.println("Could not find BME280 sensor!");
      delay(1000);
  }
  switch(bme.chipModel())
  {
    case BME280::ChipModel_BME280:
      Serial.println("Found BME280 sensor! Success.");
      break;
    case BME280::ChipModel_BMP280:
      Serial.println("Found BMP280 sensor! No Humidity available.");
      break;
    default:
      Serial.println("Found UNKNOWN sensor! Error!");
  }
  Serial.flush();
}

//--------------------------

void loop()
{
  GPSloop();
}
