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
#include "../include/LeoNerdEncoder.h"


LeoNerdEncoder::LeoNerdEncoder(uint8_t address, int intPin, void (*interruptHandler)(void), void (*eventHandler)(LeoNerdEvent), bool keyBeep) {
    _address = address;
    
    if(keyBeep) {
        setKeyBeep(330, 100);
    }
    if(intPin != -1) {
        _intPin = intPin;
        pinMode(intPin, INPUT_PULLUP);
    }
    _interruptHandler = interruptHandler;
    _eventHandler = eventHandler;
}

/**
 * Destructor for clean up
 */
LeoNerdEncoder::~LeoNerdEncoder() {
    if(_intPin != -1)
        detachInterrupt(_intPin);
    Wire.end();
}

/**
 * Initialize the encoder library
 */
void LeoNerdEncoder::begin() {
    Wire.begin();
    _wheelPos = 0;
    _isBusy = false;
    flushFifo();
    if(_intPin != -1 && _interruptHandler != NULL)
        attachInterrupt(_intPin, _interruptHandler, FALLING);
    // set release mask on all buttons by default
    setButtonReleaseMask();
}

/**
 * Flushes the FIFO of the encoder
 */
void LeoNerdEncoder::flushFifo() {
    // flush FIFO, just in case
    if(_intPin != -1) {
        while(digitalRead(_intPin) == LOW) {
            queryRegister(REG_EVENT);
        } 
    }
}

/**
 * Simple wrapper for service()
 * Call this from somwhere in your main loop()
 */
void LeoNerdEncoder::loop() {
    service();
}

/**
 * Main routine for reading the events coming from the encoder
 */
void LeoNerdEncoder::service() {
    uint8_t data = 0;
    uint8_t buf[MAX_BUFFER];
    if(_isBusy)
        return;
    memset(buf, 0xaa, MAX_BUFFER);
    interrupts();
    // if an interrupt pin is assigned and its state is HIGH, there are no events pending
    if(_intPin != -1 && digitalRead(_intPin) == HIGH) {
        return;
    }
    _isBusy = true;
    uint8_t cnt = queryRegister(REG_EVENT, buf, MAX_BUFFER);
    for(int i=0; i<cnt; i++) {
        // parse all data except 0xFF
        if(buf[i] != 0xff)
            parseData(REG_EVENT, buf[i]);
    }
    _isBusy = false;
}

/**
 * Set the button state according to the event received
 * 
 * @param instance      the button instance
 * @param state         the new button state @see ButtonState  
 */
void LeoNerdEncoder::setButtonEvent(Button* instance, ButtonState state) {
    unsigned long now = millis();

    if(state == Open) {
        unsigned long last = instance->getLastButtonReleased();
        if(now - last > DOUBLECLICK_TIME*3) {
            instance->setButtonState(Open);
        }
    }
    if(state == Pressed) {
        instance->setLastButtonPressed(now);
        instance->setButtonState(Pressed);
    }
    if(state == Held) {
        instance->setButtonState(LongClicked);
        instance->resetClickCount();
        if(_eventHandler != NULL)
            _eventHandler(LeoNerdEvent((EventType)instance->which(), instance->getButtonState()));
        instance->setLastButtonReleased(now);
    }
    if(state == Released) {
        if(instance->getButtonState() == Pressed) {
            instance->resetClickCount();
            instance->setButtonState(Clicked);
            instance->setLastButtonReleased(now);
            if(_eventHandler != NULL)
                _eventHandler(LeoNerdEvent((EventType)instance->which(), instance->getButtonState()));
        }
    }
}

/**
 * Parse the event and act sccordingly
 * 
 * @param data      the event received
 */
void LeoNerdEncoder::parseEvent(uint8_t data) {
    unsigned long now = millis();
    
    switch(data) {
        case EVENT_NONE:
            _wheelPos = 0;
            setButtonEvent(&_encoderButton, Open);
            setButtonEvent(&_mainButton, Open);
            setButtonEvent(&_leftButton, Open);
            setButtonEvent(&_rightButton, Open);
            break;
        case EVENT_WHEEL_DOWN:
            _wheelPos--;
            if(_eventHandler != NULL)
                _eventHandler(LeoNerdEvent(WheelEvent, LeftTurn));
            break;
        case EVENT_WHEEL_UP:
            _wheelPos++;
            if(_eventHandler != NULL)
                _eventHandler(LeoNerdEvent(WheelEvent, RightTurn));
            break;
        case EVENT_BUTTON_RELEASE_WHEEL:
            setButtonEvent(&_encoderButton, Released);
            break;
        case EVENT_BUTTON_PRESS_WHEEL:
            setButtonEvent(&_encoderButton, Pressed);
            break;
        case EVENT_BUTTON_HOLD_WHEEL:
            setButtonEvent(&_encoderButton, Held);
            break;
        case EVENT_BUTTON_RELEASE_MAIN:
            setButtonEvent(&_mainButton, Released);
            break;
        case EVENT_BUTTON_PRESS_MAIN:
            setButtonEvent(&_mainButton, Pressed);
            break;
        case EVENT_BUTTON_HOLD_MAIN:
            setButtonEvent(&_mainButton, Held);
            break;
        case EVENT_BUTTON_RELEASE_LEFT:
            setButtonEvent(&_leftButton, Released);
            break;
        case EVENT_BUTTON_PRESS_LEFT:
            setButtonEvent(&_leftButton, Pressed);
            break;
        case EVENT_BUTTON_HOLD_LEFT:
            setButtonEvent(&_leftButton, Held);
            break;
        case EVENT_BUTTON_RELEASE_RIGHT:
            setButtonEvent(&_rightButton, Released);
            break;
        case EVENT_BUTTON_PRESS_RIGHT:
            setButtonEvent(&_rightButton, Pressed);
            break;
        case EVENT_BUTTON_HOLD_RIGHT:
            setButtonEvent(&_rightButton, Held);
            break;
        case EVENT_GPIO_CHANGE:
            _gpioVal = data & 0x0f;
            if(_eventHandler != NULL)
                _eventHandler(LeoNerdEvent(GpioEvent, Open, _gpioVal));
            break;
        case 0xff:
            break;
        default:
            break;
    }
}

/**
 * Parse the data read according to the addressed register
 */
void LeoNerdEncoder::parseData(uint8_t reg, uint8_t data) {
    switch(reg) {
        case REG_EVENT:
            parseEvent(data);
            break;
        case REG_RELEASEMASK:
            break;
        case REG_KEYBEEP_DURATION:
            break;
        case REG_KEYBEEP_MASK:
            break;
        case REG_BEEP_DURATION:
            break;
        case REG_BEEP_TONE:
            break;
        case REG_LED1_PWM:
            _leds[0] = data;
            break;
        case REG_LED2_PWM:
            _leds[1] = data;
            break;
        case REG_GPIO_DIR:
            _gpioDir = data;
            break;
        case REG_GPIO_IO:
            _gpioVal = data;
            break;
        case REG_GPIO_PULLUP:
            break;
        case REG_GPIO_EVENTMASK:
            break;
        default:
            break;
    }
}

/**
 * Enables/Disables the double click feature on button
 * 
 * @param enabled       the enabled state
 * @param which         the button
 *
 */
void LeoNerdEncoder::setDoubleClickEnabled(bool enabled, uint8_t which) {
    switch(which) {
        case WheelButton:   _encoderButton.setDoubleClickEnabled(enabled);  break;
        case MainButton:    _mainButton.setDoubleClickEnabled(enabled);     break;
        case LeftButton:    _leftButton.setDoubleClickEnabled(enabled);     break;
        case RightButton:   _rightButton.setDoubleClickEnabled(enabled);    break;
    }
}

/**
 * Get the state of the button
 * 
 * @param which     the button @see Button
 * @returns the current @see ButtonState
 */
ButtonState LeoNerdEncoder::getButton(uint8_t which) {
    ButtonState state;
    switch(which) {
        case WheelButton:
            state = _encoderButton.getButtonState(); 
            break;
        case MainButton:
            state = _mainButton.getButtonState();    
            break;
        case LeftButton:
            state = _leftButton.getButtonState();
            break;
        case RightButton:
            state = _rightButton.getButtonState();
            break;
        default: state = Open;
    }
    if(state != Pressed)
        resetButton(which);   
    return state;
}

/**
 * Resets the state of the button
 * 
 * @param which     the button to reset @see Buttons
 */
void LeoNerdEncoder::resetButton(uint8_t which) {
    switch(which) {
        case WheelButton:   _encoderButton.resetButtonState();  break;
        case MainButton:    _mainButton.resetButtonState();     break;
        case LeftButton:    _leftButton.resetButtonState();     break;
        case RightButton:   _rightButton.resetButtonState();    break;
    }
}

/**
 * Resets the state of all buttons
 */
void LeoNerdEncoder::resetButtons() {
    _encoderButton.resetButtonState();
    _mainButton.resetButtonState();
    _leftButton.resetButtonState();
    _rightButton.resetButtonState();
}

/**
 * Switch the given LED on or off
 * 
 * @param which     number of the LED 1 or 2
 * @param state     true (on) / false (off)
 */
void LeoNerdEncoder::setLED(uint8_t which, bool state) {
    if(which >= 1 && which <= MAX_LEDS) {
        waitBusy();
        Wire.beginTransmission(_address);
        Wire.write(which == 1 ? REG_LED1_PWM : REG_LED2_PWM);
        _leds[which-1] = state ? _maxBrightness : 0;
        Wire.write(_leds[which-1]);
        Wire.endTransmission();
    }
}

/**
 * Toggle the given LED on or off
 * 
 * @param which     number of the LED 1 or 2
 * @param state     true (on) / false (off)
 */
void LeoNerdEncoder::toggleLED(uint8_t which) {
    if(which >= 1 && which <= MAX_LEDS) {
        waitBusy();
        Wire.beginTransmission(_address);
        Wire.write(which == 1 ? REG_LED1_PWM : REG_LED2_PWM);
        _leds[which-1] = _leds[which-1]==0 ? _maxBrightness : 0;
        Wire.write(_leds[which-1]);
        Wire.endTransmission();
    }
}

/**
 * Set the given GPIO pin to either INPUT or OUTPUT
 * 
 * @param which     number of the GPIO pin (0-3)
 * @param mode      INPUT or OUTPUT
 */
#if defined(__STM32F1__)
void LeoNerdEncoder::setGPIOMode(uint8_t which, WiringPinMode mode)
#else
void LeoNerdEncoder::setGPIOMode(uint8_t which, int mode)
#endif
{
    waitBusy();
    Wire.beginTransmission(_address);
    Wire.write(REG_GPIO_DIR);
    _gpioDir = (mode == INPUT) ? _gpioDir & ~(1<<which) : _gpioDir | (1<<which);
    Wire.write(_gpioDir &0x0f);
    Wire.endTransmission();
}

/**
 * Set the given GPIO pin state
 * 
 * @param which     number of the GPIO pin (0-3)
 * @param state     true (HIGH) / false (LOW)
 */
void LeoNerdEncoder::setGPIO(uint8_t which, bool state){
    waitBusy();
    Wire.beginTransmission(_address);
    Wire.write(REG_GPIO_IO);
    _gpioVal = (!state) ? _gpioVal & ~(1<<which) : _gpioVal | (1<<which);
    Wire.write(_gpioVal & 0x0f);
    Wire.endTransmission();
}

/**
 * Get the given GPIO pin state
 * 
 * @param which     number of the GPIO pin (0-3)
 * @returns true for HIGH, false for LOW
 */
bool LeoNerdEncoder::getGPIO(uint8_t which) {
    return (_gpioVal & (1<<which)) > 0;
}

/**
 * Play a tone 
 * 
 * @param frequency     the tone frequency
 * @param duration      the tone duration in milliseconds
 * 
 */
void LeoNerdEncoder::playTone(int frequency, int duration) {
    waitBusy();
    Wire.beginTransmission(_address);
    Wire.write(REG_BEEP_TONE);
    Wire.write((uint8_t)frequency/10);
    Wire.endTransmission();

    Wire.beginTransmission(_address);
    Wire.write(REG_BEEP_DURATION);
    Wire.write((uint8_t)duration/10);
    Wire.endTransmission();
}

/**
 * Play a frequency 
 * 
 * @param frequency     the tone frequency
 * @param duration      the tone duration in milliseconds
 * 
 */
void LeoNerdEncoder::playFrequency(int frequency, int duration) {
    waitBusy();
    Wire.beginTransmission(_address);
    Wire.write(REG_BEEP_FREQH);
    Wire.write((uint8_t)(frequency >> 8));
    Wire.write((uint8_t)(frequency & 0xFF));
    Wire.endTransmission();

    Wire.beginTransmission(_address);
    Wire.write(REG_BEEP_DURATION);
    Wire.write((uint8_t)duration/10);
    Wire.endTransmission();
}

/**
 * Mute buzzer 
 */
void LeoNerdEncoder::muteTone() {
    waitBusy();
    Wire.beginTransmission(_address);
    Wire.write(REG_BEEP_DURATION);
    Wire.write(0);
    Wire.endTransmission();
}

/**
 * Set the KeyBeep frequency and duration
 * 
 * @param frequency     the tone frequency
 * @param duration      the tone duration in milliseconds
 * 
 */
void LeoNerdEncoder::setKeyBeep(int frequency, int duration) {
    waitBusy();
    Wire.beginTransmission(_address);
    Wire.write(REG_BEEP_TONE);
    Wire.write((uint8_t)frequency/10);
    Wire.endTransmission();
    setKeyBeepDuration((uint8_t)duration/10);
    setKeyBeepMask();
}

/**
 * Set the KeyBeep duration
 * @param duration      the tone duration in milliseconds
 */
void LeoNerdEncoder::setKeyBeepDuration(uint8_t duration) {
    waitBusy();
    Wire.beginTransmission(_address);
    Wire.write(REG_KEYBEEP_DURATION);
    Wire.write(duration);
    Wire.endTransmission();
}

/**
 * Set the KeyBeep mask
 * @param mask      mask to set (which buttons will beep on action)
 */
void LeoNerdEncoder::setKeyBeepMask(uint8_t mask) {
    waitBusy();
    Wire.beginTransmission(_address);
    Wire.write(REG_KEYBEEP_MASK);
    Wire.write(mask);
    Wire.endTransmission();
}

/**
 * Set debounce time
 * @param time      time in milliseconds (0-255)
 */
void LeoNerdEncoder::setDebounceTime(uint8_t time) {
    waitBusy();
    Wire.beginTransmission(_address);
    Wire.write(REG_DEBOUNCE_TIME);
    Wire.write(time);
    Wire.endTransmission();
}

/**
 * Set the encoder button hold time for "Long Click"
 * @param time  time in centiseconds (0-255)
 */
void LeoNerdEncoder::setButtonHoldTime(uint8_t time) {
    waitBusy();
    Wire.beginTransmission(_address);
    Wire.write(REG_BTNHOLD_TIME);
    Wire.write(time);
    Wire.endTransmission();
}

/**
 * Set button release mask
 * @param mask  button release mask @see LeoNerdEncoder.h
 */
void LeoNerdEncoder::setButtonReleaseMask(uint8_t mask) {
    waitBusy();
    Wire.beginTransmission(_address);
    Wire.write(REG_RELEASEMASK);
    Wire.write(mask);
    Wire.endTransmission();
}

/**
 * Set EEPROM value
 * 
 * Be careful with this method and know what you're doing! 
 * It may render your encoder unusable!
 * 
 * @param eep_adr   the EEPROM address @see LeoNerdEncoder.h (REG_EEPROM*)
 * @param value     the value to be written
 */
void LeoNerdEncoder::setEepromValue(uint8_t eep_adr, uint8_t value) {
    if(eep_adr >= REG_EEPROM && eep_adr <= REG_EEPROM_BTN_POLARITY) {
        waitBusy();
        Wire.beginTransmission(_address);
        Wire.write(eep_adr);
        Wire.write(value);
        Wire.endTransmission();
    }
}


/**
 * Read the RELEASEMASK
 * 
 * @returns the current value
 */
uint8_t LeoNerdEncoder::queryReleaseMask() {
    waitBusy();
    return queryRegister(REG_RELEASEMASK);
}

/**
 * Read the KEYBEEP DURATION
 * 
 * @returns the current value
 */
uint8_t LeoNerdEncoder::queryKeyBeepDuration() {
    waitBusy();
    return queryRegister(REG_KEYBEEP_DURATION);
}

/**
 * Read the KEYBEEP MASK
 * 
 * @returns the current value
 */
uint8_t LeoNerdEncoder::queryKeyBeepMask() {
    waitBusy();
    return queryRegister(REG_KEYBEEP_MASK);
}

/**
 * Read the BEEP DURATION
 * 
 * @returns the current value
 */
uint8_t LeoNerdEncoder::queryBeepDuration() {
    waitBusy();
    return queryRegister(REG_BEEP_DURATION);
}

/**
 * Read the BEEP TONE
 * 
 * @returns the current value
 */
uint8_t LeoNerdEncoder::queryBeepTone() {
    waitBusy();
    return queryRegister(REG_BEEP_TONE);
}

/**
 * Read the PWM value for LED1
 * 
 * @returns the current value
 */
uint8_t LeoNerdEncoder::queryLed1Pwm() {
    waitBusy();
    return queryRegister(REG_LED1_PWM);
}

/**
 * Read the PWM value for LED2 
 * 
 * @returns the current value
 */
uint8_t LeoNerdEncoder::queryLed2Pwm() {
    waitBusy();
    return queryRegister(REG_LED2_PWM);
}

/**
 * Read the GPIO DIRECTION settings
 * 
 * @returns the current value
 */
uint8_t LeoNerdEncoder::queryGpioDir() {
    waitBusy();
    return queryRegister(REG_GPIO_DIR);
}

/**
 * Read the GPIO states
 * 
 * @returns the current value
 */
uint8_t LeoNerdEncoder::queryGpioIo() {
    waitBusy();
    return queryRegister(REG_GPIO_IO);
}

/**
 * Read the GPIO PULLUP settings
 * 
 * @returns the current value
 */
uint8_t LeoNerdEncoder::queryGpioPullup() {
    waitBusy();
    return queryRegister(REG_GPIO_PULLUP);
}

/**
 * Read the GPIO EVENTMASK settings
 * 
 * @returns the current value
 */
uint8_t LeoNerdEncoder::queryGpioEventMask() {
    waitBusy();
    return queryRegister(REG_GPIO_EVENTMASK);
}

/**
 * Query the ENCODER ADDRESS 
 * 
 * @returns the configured address
 */
uint8_t LeoNerdEncoder::queryEncoderAddress() {
    waitBusy();
    return queryRegister(REG_EEPROM);
}

/**
 * Read the DEBOUNCE TIME 
 * 
 * @returns the default time
 */
uint8_t LeoNerdEncoder::queryDebounceTime() {
    waitBusy();
    return queryRegister(REG_EEPROM_DEBOUNCE);
}

/**
 * Read the HOLD TIME 
 * 
 * @returns the default time
 */
uint8_t LeoNerdEncoder::queryHoldTime() {
    waitBusy();
    return queryRegister(REG_EEPROM_HOLDTIME);
}

/**
 * Read the BUTTON TO GPIO MAPPING
 * 
 * @returns the current GPIO ports it's mapped to
 */
uint8_t LeoNerdEncoder::queryButtonMapping(Buttons button) {
    waitBusy();
    uint8_t stat = queryRegister(REG_EEPROM_BTN_MAPPING);
    switch(button) {
        case WheelButton:
            stat = (stat & 1);
            break;
        case MainButton:
            stat = (stat & 2) >> 1;
            break;
        case LeftButton:
            stat = (stat & 4) >> 2;
            break;
        case RightButton:
            stat = (stat & 8) >> 3;
            break;
        default:
        stat = 0;
    }
    return stat;
}

/**
 * Read the BUTTON TO GPIO MAPPING POLARITY
 * 
 * @returns     the current GPIO ports it's mapped to
 */
uint8_t LeoNerdEncoder::queryButtonMappingPolarity(Buttons button) {
    waitBusy();
    uint8_t stat = queryRegister(REG_EEPROM_BTN_POLARITY);
    switch(button) {
        case WheelButton:
            stat = (stat & 1);
            break;
        case MainButton:
            stat = (stat & 2) >> 1;
            break;
        case LeftButton:
            stat = (stat & 4) >> 2;
            break;
        case RightButton:
            stat = (stat & 8) >> 3;
            break;
        default:
        stat = 0;
    }
    return stat;
}


/**
 * Read the VERSION info
 * 
 * @returns the current value
 */
uint8_t LeoNerdEncoder::queryVersion() {
    waitBusy();
    return queryRegister(REG_SW_VERSION);
}

/**
 * Query the value of the given (FIFO buffered) register
 * @param reg       the register in charge
 * @param buffer    the pointer to the result buffer
 * @param size      the max. size of the result buffer
 * 
 * @returns the number of bytes received from FIFO; buffer gets filled accordingly
 */
uint8_t LeoNerdEncoder::queryRegister(uint8_t reg, uint8_t* buffer, uint8_t size) {
    uint8 stat = 0;
    do {
        Wire.beginTransmission(_address);
        Wire.write(reg);
        stat = Wire.endTransmission();
    } while(stat > 1);
    if(stat == SUCCESS) {
        uint8 cnt = Wire.requestFrom(_address, size);
        while(!Wire.available())
            delayMicroseconds(10);
        int ndx = 0;
        while(ndx < cnt) {
            uint8_t response = Wire.read();
            if(ndx < size)
                *(buffer+ndx) = response;
            ndx++;
        }
        return cnt;
    }
    return 0;
}

/**
 * Query the value of the given register
 * @param reg   the register in charge
 * 
 * @returns the response read
 */
uint8_t LeoNerdEncoder::queryRegister(uint8_t reg) {
    uint8 stat = 0;
    do {
        Wire.beginTransmission(_address);
        Wire.write(reg);
        stat = Wire.endTransmission();
    } while(stat > 1);
    Wire.requestFrom(_address, 1);
    while(!Wire.available())
        delayMicroseconds(10);
    return Wire.read();
}

