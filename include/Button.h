/**
 * Library for LeoNerd's OLED Module
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
#pragma once

#ifndef _BUTTON_H
#define _BUTTON_H 1

#include <Arduino.h>
#include "ButtonState.h"

class Button {
public:
    Button() { _which = 0; _state = Open; }
    Button(uint8_t which) { _which = which; _state = Open; _lastButtonPressed = 0L; _lastButtonReleased = 0L; _doubleClickEnabled = false; _clickCount = 0; }

    void            setButtonState(ButtonState state) { _state = state; }
    void            resetButtonState() { _state = Open; }
    ButtonState     getButtonState() { return _state; }
    void            setLastButtonPressed(unsigned long ticks) { _lastButtonPressed = ticks; }
    void            setLastButtonReleased(unsigned long ticks) { _lastButtonReleased = ticks; }
    unsigned long   getLastButtonPressed() { return _lastButtonPressed; }
    unsigned long   getLastButtonReleased() { return _lastButtonReleased; }
    void            setDoubleClickEnabled(bool enable) { _doubleClickEnabled = enable; }
    bool            getDoubleClickEnabled() { return _doubleClickEnabled; }
    void            incrementClickCount() { _clickCount++; }
    void            resetClickCount() { _clickCount = 0; }
    uint8_t         getClickCount() { return _clickCount; }
    uint8_t         which() { return _which; }

private:
    volatile uint8_t         _which;
    volatile ButtonState     _state;
    unsigned long           _lastButtonPressed;
    unsigned long           _lastButtonReleased;
    volatile uint8_t        _doubleClickTicks;     
    bool                    _doubleClickEnabled;  
    volatile uint8_t        _clickCount;
};
#endif
