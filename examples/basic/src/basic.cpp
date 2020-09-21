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
#include "U8g2lib.h"
#include "LeoNerdEncoder.h"

#define DISPLAY_ADDRESS     0x3C    // use the default address
#define ENCODER_ADDRESS     0x3D    // use the default address

#define BASE_FONT           u8g2_font_6x12_t_symbols
#define ICONIC_FONT         u8g2_font_open_iconic_check_2x_t
#define SYMBOL_FONT         u8g2_font_unifont_t_symbols

#define ArraySize(arr)    (sizeof(arr)/sizeof(arr[0]))


U8G2_SH1106_128X64_NONAME_F_HW_I2C  display(U8G2_R2, /* reset=*/ U8X8_PIN_NONE); 
LeoNerdEncoder                      encoder(ENCODER_ADDRESS);

ButtonState wheel, main, left, right;
int16_t     encoderPos = 0;

void __debug(const char* fmt, ...) {
    char _tmp[256];
    va_list arguments;
    va_start(arguments, fmt); 
    vsnprintf_P(_tmp, ArraySize(_tmp)-1, fmt, arguments);
    va_end (arguments); 
    Serial.print(F("Debug: "));
    Serial.println(_tmp); 
}

void setupDisplay() {
    display.begin();
    display.enableUTF8Print();
    display.clearDisplay();
    display.setFont(BASE_FONT);
    display.setFontMode(0);
    display.setDrawColor(1);
}

void setupEncoder() {
    encoder.begin();
    uint8_t ver = encoder.queryVersion();
    __debug(PSTR("Encoder version is: %d"), ver);
}

void drawStatus() {
    char tmp[60];
    display.setFontMode(0);
    display.setDrawColor(1);
    display.drawStr(display.getDisplayWidth() - display.getStrWidth(tmp) - 10, 14, tmp);
    display.drawStr(display.getDisplayWidth() - display.getStrWidth(tmp) - 10, 14, tmp);
}

void setup() {
    Serial.begin(57600);
    setupDisplay();
    setupEncoder();
}

void loop() {
    encoder.loop();
    encoderPos += encoder.getValue();
    wheel   = encoder.getButton(WheelButton);
    main    = encoder.getButton(MainButton);
    left    = encoder.getButton(LeftButton);
    right   = encoder.getButton(RightButton);

    display.firstPage();
    do {
        drawStatus();
    } while(display.nextPage());
}