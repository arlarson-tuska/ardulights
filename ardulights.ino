//  Copyright (c) Alex Larson 2015
//  This program is based off of the work of Scott Simpson's MavSky 

//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  A copy of the GNU General Public License is available at <http://www.gnu.org/licenses/>.
//    
//
//  For Teensy 3.1 support
//    Connect:
//      
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <GCS_MAVLink.h>
#include <FastLED.h>

#include "MavSky.h"
#include "Logger.h"
#include "MavLink.h"

#define PRODUCT_STRING  "MAVSky Version 1.1.0"

#define DEBUG_SERIAL    Serial
#define MAVLINK_SERIAL  Serial2
#define LEDPIN          13
#define DATA_PIN 6
#define NUM_LEDS 53
CRGB leds[NUM_LEDS];
uint32_t frontColor,backColor;

uint32_t pTime,pastTime;
int ledSwitch,quadMode;
bool armed = 0,failsafe = 0,ledSwitchEvent = 0,LEDState = 0;
uint8_t LEDMODE;
uint16_t rcThrottle,batteryVoltage;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Auto Pilot modes
// ----------------
#define STABILIZE 0                     // hold level position
#define ACRO 1                          // rate control
#define ALT_HOLD 2                      // AUTO control
#define AUTO 3                          // AUTO control
#define GUIDED 4                        // AUTO control
#define LOITER 5                        // Hold a single location
#define RTL 6                           // AUTO control
#define CIRCLE 7                        // AUTO control
#define LAND 9                          // AUTO control
#define OF_LOITER 10                    // Hold a single location using optical flow sensor
#define DRIFT 11                        // DRIFT mode (Note: 12 is no longer used)
#define SPORT 13                        // earth frame rate control
#define FLIP        14                  // flip the vehicle on the roll axis
#define AUTOTUNE    15                  // autotune the vehicle's roll and pitch gains
#define POSHOLD     16                  // position hold with manual override
#define NUM_MODES   17

#define FAILSAFE_THRESHOLD 975          //threshold for failsafe based on raw RC throttle reading
#define LEDSWITCH_THRESHOLD 1000        //threshold for switching lights on based on raw RC channel 7 reading


//

#define FC_STABILIZE 0x0000FF
#define BC_STABILIZE 0xFF0000

#define FC_LOITER 0x00BBFF
#define BC_LOITER 0xFF0000

#define FC_RTL 0x00FF00
#define BC_RTL 0xFF0000

#define FC_ALT_HOLD 0x00CCAA
#define BC_ALT_HOLD 0xFF0000

#define FC_AUTO 0x0000FF
#define BC_AUTO 0xFFAA00

void console_print(char* fmt, ...) {
    char formatted_string[256];
    va_list argptr;
    va_start(argptr, fmt);
    vsprintf(formatted_string, fmt, argptr);
    va_end(argptr);
    DEBUG_SERIAL.print(formatted_string);
}

void setup()  {
  //debug_init();
  delay(5000);
  FastLED.addLeds<WS2811, DATA_PIN, RGB>(leds, NUM_LEDS);
  pinMode(LEDPIN, OUTPUT);
  //console_print("%s\nStarting\n]", PRODUCT_STRING);
  mavlink_init();
}

uint32_t next_200_loop = 0L;
uint32_t next_100_loop = 0L;

void loop()  {
  uint16_t len;
  uint32_t current_milli;
  
  process_mavlink_packets();

  updateLED(20);
  
  current_milli = millis();
  
}

void updateLED(int waitTime)
{
  static uint32_t prevLEDUpdate = 0; 

  if(millis() - prevLEDUpdate >= waitTime){

    setLEDColor();

    if(checkFailsafe() || checkLEDSwitch()){
      if(ledSwitchEvent == true)
        wipeLEDS(10);
      if(armed == false)
        fadeLEDS();
      else
        setLEDSSolidColor();
    }
    else{
      FastLED.setBrightness(0);
    }
    FastLED.show();
  }
}

bool checkFailsafe()
{
  if(rcThrottle <= FAILSAFE_THRESHOLD)
  {
    failsafe = true; 
    return true;
  }
  else
  {
    failsafe = false;
    return false;
  }
}

bool checkLEDSwitch()
{
  static bool prevLEDSwitch = 0;

  if(ledSwitch <= LEDSWITCH_THRESHOLD)
  {
    LEDState = true;
    if(prevLEDSwitch == false)
      ledSwitchEvent = true;
    prevLEDSwitch = true;
    return true;
  }
  else
  {
    LEDState = false;
    prevLEDSwitch = false;
    ledSwitchEvent = false;
    return false;
  }
}

void setLEDColor()
{
  switch(quadMode){
    case STABILIZE:{
      frontColor = FC_STABILIZE;
      backColor = BC_STABILIZE;
      break;
    }
    case LOITER:{
      frontColor = FC_LOITER;
      backColor = BC_LOITER;
      break;
    }
    case ALT_HOLD:{
      frontColor = FC_ALT_HOLD;
      backColor = BC_ALT_HOLD;
      break;
    }
    case RTL:{
      frontColor = FC_RTL;
      backColor = BC_RTL;
      break;
    }
    case AUTO:{
      frontColor = FC_AUTO;
      backColor = BC_AUTO;
      break;
    }
  }
}

void setLEDSSolidColor()
{
  for(int i = 0; i < NUM_LEDS/2; i++) 
       leds[i] = frontColor;
    
     for(int i = NUM_LEDS/2; i < NUM_LEDS; i++) 
       leds[i] = backColor; 
}

void fadeLEDS()
{
  setLEDSSolidColor();
  uint8_t x = (millis()/30); 
  FastLED.setBrightness(sin8(x));
}

bool wipeLEDS(uint8_t waitTime)
{
  static uint8_t LEDIndex = 0;
  static uint32_t pTime = 0;

  if(millis() - pTime > waitTime)
  {
    leds[LEDIndex] = frontColor;
    leds[NUM_LEDS/2 + LEDIndex++] = backColor;
  }
  if(LEDIndex >= NUM_LEDS/2)
  {
    LEDIndex = 0;
    ledSwitchEvent = false;
    return 1;
  }
  else
    return 0;
}









