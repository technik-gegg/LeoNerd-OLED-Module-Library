/**
 * Basic example for LeoNerd's OLED Module Library
 *
 * Copyright (C) 2020 Technik Gegg
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
 
 /*
  * Please notice: Even though this example has been written and compiled
  * for the Arduino Nano, it won't leave much storage left for anything else
  * than that.
  * If you really need such expensive interface, you should be probably using 
  * a beefier MCU, such as the ATMEGA2560.
  */
  
// Basic includes
#include <Arduino.h>
#include "U8g2lib.h"
#include "LeoNerdEncoder.h"

#define DISPLAY_ADDRESS     0x3C    // use the default address
#define ENCODER_ADDRESS     0x3D    // use the default address

// Fonts used in this program.
// Be aware that each font used will reduce the amount of
// available Flash memory!
#define BASE_FONT           u8g2_font_6x12_t_symbols
#define SMALL_FONT          u8g2_font_6x10_mr

// simple wrapper for determining the real size of an array in bytes
#define ArraySize(arr)    (sizeof(arr)/sizeof(arr[0]))

// forward declaration makes every compiler happy
void        setupDisplay();
void        setupEncoder();
void        draw();
void        drawStatus();
void        scanI2CDevices();
const char* PROGMEM xlateBtn(ButtonState state);
void        __debug(const char* fmt, ...);
void        playTune();

// Runtime instances for display and encoder
LeoNerdEncoder                      encoder(ENCODER_ADDRESS);

// For the display we're using the famous U8G2 library because of the already 
// integrated support for SH1106 but you're free to use any other library
// that supports this display type.
// Please notice: The rotation (U8G2_R2 - 180°) has to be set for this module! 
U8G2_SH1106_128X64_NONAME_F_HW_I2C  display(U8G2_R2, /* reset=*/ U8X8_PIN_NONE); 

// runtime variables
ButtonState wheelBtn,
            mainBtn,
            leftBtn,
            rightBtn;
int16_t     encoderPos = 0, lastEncoderPos = 0;

const uint16_t tune[] PROGMEM = {
  // Frequency, Duration, Pause
  1760,  90,  90,
  1975,  90,  90,
  2093,  90,  90,
  1975,  90,  90,
  1760, 200,  50,
     0,   0,   0
};


// set everything up
void setup() {
    Serial.begin(57600);
    __debug(PSTR("[ Start ]"));
    // I2C scan must show (at least) 2 devices (address 0x3c and 0x3d).
    // If there are no devices reported, check your wiring.
    // If there are more than two devices, this might point to  
    // insufficient termination resistors on your SDA/SCL lines.
    scanI2CDevices();
    // initialize the display (U8G2 library)
    setupDisplay();
    // initialize the LeoNerd's encoder
    setupEncoder();
    // enable encoder wheel to beep on rotation only
    encoder.setKeyBeep(1400, 10);
    encoder.setKeyBeepMask(BEEP_WHEEL_ROTATION);
    draw();
}

void loop() {
    // service the encoder status (must be called from within this loop!)
    encoder.loop();
    // read the wheel position and store it
    int8_t v = encoder.getValue();
    //__debug(PSTR("Enc: %d"),v);
    encoderPos += v;
    // read the status of each button and stor it
    wheelBtn   = encoder.getButton(WheelButton);
    mainBtn    = encoder.getButton(MainButton);
    leftBtn    = encoder.getButton(LeftButton);
    // right button plays a tune if held
    if((rightBtn = encoder.getButton(RightButton)) == LongClicked)
      playTune();
    
    // if the encoder has been turned
    if(encoderPos != lastEncoderPos) {
      lastEncoderPos = encoderPos;
      draw();
      // set LEDs of Main button to off when wheel position = 0
      if(encoderPos == 0) {
          encoder.setLED(LED_GREEN, false);
          encoder.setLED(LED_RED, false);
      }
      // turn green LED on if wheel position is positive 
      if(encoderPos > 0) {
          encoder.setLED(LED_GREEN, true);
      }
      // turn red LED on if wheel position is negative 
      if(encoderPos < 0) {
          encoder.setLED(LED_RED, true);
      }
    }
    if(wheelBtn != Open || mainBtn != Open || leftBtn != Open || rightBtn != Open) { 
      // draw button states and encoder position on the display
      draw();
    }
}

// Function used to draw the data on display
void draw() {
    display.firstPage();
    do {
        drawStatus();
    } while(display.nextPage());
}

// Function used to initialize the display
void setupDisplay() {
    display.begin();
    display.enableUTF8Print();
    display.clearDisplay();
}

// Function used to initialize the encoder
void setupEncoder() {
    encoder.begin();
    uint8_t ver = encoder.queryVersion();
    // Please notice: This library needs at least version 2!
    __debug(PSTR("LeoNerd's encoder version is: %d"), ver);
    // check the options 
    uint8_t opt = encoder.queryOptions();
    __debug(PSTR("Options set: 0x%02X"), opt);
}

// modify this routine to meet your needs
void drawStatus() {
    char tmp[20];
    display.setFont(BASE_FONT);
    display.setFontMode(0);
    display.setDrawColor(1);
    sprintf_P(tmp, PSTR("WHEEL"));          display.drawStr(4,  10, tmp);
    sprintf_P(tmp, PSTR("MAIN"));           display.drawStr(99, 10, tmp);
    sprintf_P(tmp, PSTR("LEFT"));           display.drawStr(4,  52, tmp);
    sprintf_P(tmp, PSTR("RIGHT"));          display.drawStr(92, 52, tmp);
    sprintf_P(tmp, PSTR("POS"));            display.drawStr(42, 28, tmp);
    sprintf(tmp, "%d", encoderPos);         display.drawStr(68, 28, tmp);
    
    display.setFont(SMALL_FONT);
    sprintf_P(tmp, xlateBtn(wheelBtn));     display.drawStr(4,  23, tmp);  
    sprintf_P(tmp, xlateBtn(mainBtn));      display.drawStr(99, 23, tmp);
    sprintf_P(tmp, xlateBtn(leftBtn));      display.drawStr(4,  40, tmp);
    sprintf_P(tmp, xlateBtn(rightBtn));     display.drawStr(99, 40, tmp);
}

// Function used to translate the button states into readable text
const char* PROGMEM xlateBtn(ButtonState state) {
    switch(state) {
        case Open:          return PSTR("None");
        case Clicked:       return PSTR("Click");
        case LongClicked:   return PSTR("Hold");
        default:            return PSTR("");
    }
}

// Function used to scan for all available I2C/TWI devices on the bus
void scanI2CDevices() {
  Wire.begin();
  for(uint8_t i2cAdr = 1; i2cAdr < 127; i2cAdr++)
  {
    // request a transmission for the device address
    Wire.beginTransmission(i2cAdr);
    // if something answered, the device is basically available
    if (Wire.endTransmission() == 0) {
      __debug(PSTR("I2C device found at address 0x%2x"), i2cAdr);
    }
  }
}

// Function that plays a tune (melody)
void playTune() {
  uint16_t f=0, d=0, p=0;
  uint8_t n=0;
    
  while(1) {
    f = pgm_read_word((tune+(n*3)));
    d = pgm_read_word((tune+(n*3))+1);
    p = pgm_read_word((tune+(n*3))+2);
    n++;
    if(f==0 && d==0)  // end of tune condition
      break;
    // if frequency and duration are set, play the tone
    if(f && d) {
      encoder.playFrequency(f, d);
      delay(p ? p : d);
    }
  }
}

// Function used to print out formatted debug information
void __debug(const char* fmt, ...) {
    char _tmp[60];
    va_list arguments;
    va_start(arguments, fmt); 
    vsnprintf_P(_tmp, ArraySize(_tmp)-1, fmt, arguments);
    va_end (arguments); 
    Serial.print(F("Debug: "));
    Serial.println(_tmp); 
}