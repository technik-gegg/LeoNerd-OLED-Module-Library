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

#ifndef _LEONERD_ENCODER_H
#define _LEONERD_ENCODER_H 1

#include <stdint.h>
#if defined (__AVR__)
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#endif
#if defined (__STM32F1__)
#include <wirish.h>
#include <libmaple/gpio.h>
#endif
#include "Arduino.h"
#include <Wire.h>
#include "ButtonState.h"

// Registers for I2C
#define REG_EVENT               0x01    // Implements an 8-level deep FIFO of input events. Each event is formed of a single byte. 
#define REG_RELEASEMASK         0x02    // Controls whether buttons will send event reports on release as well as press (bits set to 1), or on press only (bits set to 0).
#define REG_DEBOUNCE_TIME       0x03    // Sets the duration in milliseconds for key debounce detection
#define REG_BTNHOLD_TIME        0x04    // Sets the duration in centiseconds for reporting a "button held" event (75 by default)
#define REG_KEYBEEP_DURATION    0x10    // Controls the duration of the autonomous keybeep in centiseconds
#define REG_KEYBEEP_MASK        0x11    // Controls which button events cause an autonomous keybeep
#define REG_BEEP_DURATION       0x12    // Sets the duration of the beep tone in centiseconds
#define REG_BEEP_TONE           0x13    // Sets the frequency of the beep tone, in units of 10 Hz
#define REG_BEEP_FREQH          0x14    // Sets the frequency of the beep tone, in units of 1 Hz (upper 8 bit)
#define REG_BEEP_FREQL          0x15    // Sets the frequency of the beep tone, in units of 1 Hz (lower 8 bit)
#define REG_LED1_PWM            0x20    // Sets the brightness of LED1, which is connected to the red LED in the main button
#define REG_LED2_PWM            0x21    // Sets the brightness of LED2, which is connected to the green LED in the main button
#define REG_GPIO_DIR            0x30    // Sets the drive direction on each of the GPIO lines. 0 = input; 1 = output
#define REG_GPIO_IO             0x31    // Sets the output state for any GPIO line currently set as output
#define REG_GPIO_PULLUP         0x32    // Enables pullup resistors on any of the GPIO lines with bits set high (1)
#define REG_GPIO_EVENTMASK      0x33    // Sets a mask used for level change event detection on the GPIO lines
#define REG_EEPROM              0xC0    // EEPROM storage for encoder I2C address (default: 0x3d)
#define REG_EEPROM_OPTIONS      0xC1    // EEPROM storage for misc. options (default: 0x00)
#define REG_EEPROM_DEBOUNCE     0xC2    // EEPROM storage for debounce time (default: 0x14)
#define REG_EEPROM_HOLDTIME     0xC3    // EEPROM storage for hold time (default: 0x4b)
#define REG_EEPROM_BTN_MAPPING  0xC4    // EEPROM storage for mapping button to GPIO (default: 0x00; 0x04 for SMuFF)
#define REG_EEPROM_BTN_POLARITY 0xC5    // EEPROM storage for mapping button to GPIO polarity (default: 0x00; 0x04 for SMuFF)
#define REG_SW_VERSION          0xF0    // Reports the current firmware version (default: 0x02)

// Release Mask 
#define EVENT_RELEASE_IGNORE    0x01 
#define EVENT_RELEASE_WHEEL     0x02
#define EVENT_RELEASE_MAIN      0x04
#define EVENT_RELEASE_LEFT      0x08
#define EVENT_RELEASE_RIGHT     0x10
#define RELEASE_ALL_KEYS        EVENT_RELEASE_WHEEL | EVENT_RELEASE_MAIN | EVENT_RELEASE_LEFT | EVENT_RELEASE_RIGHT

// Key Beep Mask
#define BEEP_NONE               0x00
#define BEEP_WHEEL_ROTATION     0x01
#define BEEP_WHEEL_BUTTON       0x02
#define BEEP_MAIN_BUTTON        0x04
#define BEEP_LEFT_BUTTON        0x08
#define BEEP_RIGHT_BUTTON       0x10
#define BEEP_ALL_KEYS           BEEP_WHEEL_ROTATION | BEEP_WHEEL_BUTTON | BEEP_MAIN_BUTTON | BEEP_LEFT_BUTTON | BEEP_RIGHT_BUTTON

// Events
// The top three bits of the event classify it into a category, 
// the remaining bits are specific to that category
#define EVENT_NONE                      0x00
#define EVENT_WHEEL_DOWN                0x21
#define EVENT_WHEEL_UP                  0x22
#define EVENT_BUTTON_RELEASE_WHEEL      0x40
#define EVENT_BUTTON_PRESS_WHEEL        0x41
#define EVENT_BUTTON_HOLD_WHEEL         0x42
#define EVENT_BUTTON_RELEASE_MAIN       0x44
#define EVENT_BUTTON_PRESS_MAIN         0x45
#define EVENT_BUTTON_HOLD_MAIN          0x46
#define EVENT_BUTTON_RELEASE_LEFT       0x48
#define EVENT_BUTTON_PRESS_LEFT         0x49
#define EVENT_BUTTON_HOLD_LEFT          0x4A
#define EVENT_BUTTON_RELEASE_RIGHT      0x4C
#define EVENT_BUTTON_PRESS_RIGHT        0x4D
#define EVENT_BUTTON_HOLD_RIGHT         0x4E
#define EVENT_GPIO_CHANGE               0x60

#define MAX_LEDS                2       // there ain't no more
#define MAX_BUFFER              8       // buffer size of the FIFO
#define DOUBLECLICK_TIME        600     // second click must be within 600ms (not supported yet)
#define LONG_CLICK_TIME         1200    // hold time for long click

// Buttons
typedef enum _Buttons {
    NoButton = 0,
    WheelButton,
    MainButton,
    LeftButton,
    RightButton
} Buttons;

// Event types
typedef enum _EventType {
    None = -1,
    WheelEvent = 0,
    WheelButtonEvent,
    MainButtonEvent,
    LeftButtonEvent,
    RightButtonEvent,
    GpioEvent
} EventType;

class LeoNerdEvent {
public:
    LeoNerdEvent() { EvtType = None; State = Open; Value = 0;}
    LeoNerdEvent(EventType event, ButtonState state, uint8_t value = 0) { EvtType = event; State = state; Value = value; }
    volatile EventType      EvtType;
    volatile ButtonState    State;
    volatile uint8_t        Value;
};

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

class LeoNerdEncoder {
public:
    LeoNerdEncoder(uint8_t address, int intPin = -1, void (*interruptHandler)(void) = NULL, void (*eventHandler)(LeoNerdEvent) = NULL, bool keyBeep = false);
    ~LeoNerdEncoder();

    void            begin(void);
    void            service(void);
    void            loop(void);
    void            flushFifo(void);
    int16_t         getValue(void) { int16_t val = _wheelPos; _wheelPos = 0; return val; };

    ButtonState     getButton(uint8_t which = WheelButton);
    void            resetButton(uint8_t which = WheelButton);
    void            resetButtons(void);
    void            setKeyBeep(int frequency, int duration);
    void            setKeyBeepDuration(uint8_t duration);
    void            setKeyBeepMask(uint8_t mask = BEEP_ALL_KEYS);
    void            setDebounceTime(uint8_t time = 20);
    void            setButtonHoldTime(uint8_t time = 75);
    void            setButtonReleaseMask(uint8_t mask = RELEASE_ALL_KEYS);
    void            setAccelerationEnabled(bool state) { _accelEnabled = state; }       // dummy; maybe someday in the future
    bool            getAccelerationEnabled() { return _accelEnabled; }                  // dito
    void            playTone(int frequency, int duration);
    void            playFrequency(int frequency, int duration);
    void            muteTone(void);
    void            setDoubleClickEnabled(bool enabled, uint8_t which = WheelButton);   // dummy; maybe someday in the future
    bool            busy(void) { return _isBusy; }

    void            setMaxBrightness(uint8_t brightness) { _maxBrightness = brightness; }
    void            setLED(uint8_t which, bool state);
    void            toggleLED(uint8_t which);
#if defined(__STM32F1__)
    void            setGPIOMode(uint8_t which, WiringPinMode mode);
#else
    void            setGPIOMode(uint8_t which, int mode);
#endif
    void            setGPIO(uint8_t which, bool state);
    bool            getGPIO(uint8_t which);
    void            setEepromValue(uint8_t address, uint8_t value);

    uint8_t         queryReleaseMask(void);
    uint8_t         queryKeyBeepDuration(void);
    uint8_t         queryKeyBeepMask(void);
    uint8_t         queryBeepDuration(void);
    uint8_t         queryBeepTone(void);
    uint8_t         queryLed1Pwm(void);
    uint8_t         queryLed2Pwm(void);
    uint8_t         queryGpioDir(void);
    uint8_t         queryGpioIo(void);
    uint8_t         queryGpioPullup(void);
    uint8_t         queryGpioEventMask(void);
    uint8_t         queryEncoderAddress(void);
    uint8_t         queryDebounceTime(void);
    uint8_t         queryHoldTime(void);
    uint8_t         queryButtonMapping(Buttons button);
    uint8_t         queryButtonMappingPolarity(Buttons button);
    uint8_t         queryVersion(void);

private:
    void            parseData(uint8_t reg, uint8_t data);
    void            parseEvent(uint8_t data);
    void            setButtonEvent(Button* instance, ButtonState state);
    uint8_t         queryRegister(uint8_t reg, uint8_t* buffer, uint8_t size);
    uint8_t         queryRegister(uint8_t reg);
    void            waitBusy(void) { while(_isBusy); };
    void            (*_eventHandler)(LeoNerdEvent);
    void            (*_interruptHandler)(void);

    uint8_t         _address;
    Button          _encoderButton;
    Button          _mainButton;
    Button          _leftButton;
    Button          _rightButton;
    uint8_t         _maxBrightness;
    uint8_t         _gpioDir;
    uint8_t         _gpioVal;
    uint8_t         _leds[MAX_LEDS];
    volatile int16_t _wheelPos;
    int8_t          _intPin;
    bool            _accelEnabled;
    volatile bool   _isBusy;
};
#endif