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
#include <Arduino.h>
#include <Wire.h>
#include "U8g2lib.h"
#include "LeoNerdEncoder.h"

#define DISPLAY_ADDRESS     0x3C    // use the default address
#define ENCODER_ADDRESS     0x3D    // use the default address

#define BASE_FONT           u8g2_font_6x12_t_symbols
#define SMALL_FONT          u8g2_font_6x10_mr

#define ArraySize(arr)    (sizeof(arr)/sizeof(arr[0]))

// forward declaration to keep the compiler happy
void setupDisplay();
void setupEncoder();
void draw();
void drawStatus();
void scanI2CDevices();
const char* PROGMEM xlateBtn(ButtonState state);
void __debug(const char* fmt, ...);

// instances of display and encoder
U8G2_SH1106_128X64_NONAME_F_HW_I2C  display(U8G2_R2, /* reset=*/ U8X8_PIN_NONE); 
LeoNerdEncoder                      encoder(ENCODER_ADDRESS);

// runtime vars
ButtonState wheelBtn,
            mainBtn,
            leftBtn,
            rightBtn;
int16_t     encoderPos = 0;

void setup() {
    Serial.begin(57600);
    __debug(PSTR("[ Start ]"));
    // must show (at least) 2 devices (0x3c and 0x3d)
    scanI2CDevices();
    setupDisplay();
    setupEncoder();
}

void loop() {
    
    encoder.loop();
    encoderPos += encoder.getValue();
    wheelBtn   = encoder.getButton(WheelButton);
    mainBtn    = encoder.getButton(MainButton);
    leftBtn    = encoder.getButton(LeftButton);
    rightBtn   = encoder.getButton(RightButton);
    draw();
    delay(250);
}

void draw() {
    display.firstPage();
    do {
        drawStatus();
    } while(display.nextPage());
}

void setupDisplay() {
    display.begin();
    display.enableUTF8Print();
    display.clearDisplay();
}

void setupEncoder() {
    encoder.begin();
    uint8_t ver = encoder.queryVersion();
    __debug(PSTR("LeoNerd's encoder version is: %d"), ver);
}

void drawStatus() {
    char tmp[40];
    display.setFont(BASE_FONT);
    display.setFontMode(0);
    display.setDrawColor(1);
    display.drawStr(4,  10, "WHEEL");
    display.drawStr(99, 10, "MAIN");
    display.drawStr(4,  52, "LEFT");
    display.drawStr(92, 52, "RIGHT");
    display.drawStr(42, 28, "POS:");
    sprintf(tmp, "%d", encoderPos);
    display.drawStr(68, 28, tmp);
    display.setFont(SMALL_FONT);
    sprintf_P(tmp, "%s", xlateBtn(wheelBtn));
    display.drawStr(4,  23, tmp);
    sprintf_P(tmp, "%s", xlateBtn(mainBtn));
    display.drawStr(99, 23, tmp);
    sprintf_P(tmp, "%s", xlateBtn(leftBtn));
    display.drawStr(4,  40, tmp);
    sprintf_P(tmp, "%s", xlateBtn(rightBtn));
    display.drawStr(99, 40, tmp);
}

const char* PROGMEM xlateBtn(ButtonState state) {
    switch(state) {
        case Open:          return PSTR("None");
        case Clicked:       return PSTR("Click");
        case LongClicked:   return PSTR("LngClick");
        default:            return PSTR("---");
    }
}

void scanI2CDevices() {
  Wire.begin();
  for(uint8_t i2cAdr = 1; i2cAdr < 127; i2cAdr++)
  {
    Wire.beginTransmission(i2cAdr);
    if (Wire.endTransmission() == 0) {
      __debug(PSTR("I2C device found at address 0x%2x"), i2cAdr);
    }
  }
}

void __debug(const char* fmt, ...) {
    char _tmp[128];
    va_list arguments;
    va_start(arguments, fmt); 
    vsnprintf_P(_tmp, ArraySize(_tmp)-1, fmt, arguments);
    va_end (arguments); 
    Serial.print(F("Debug: "));
    Serial.println(_tmp); 
}