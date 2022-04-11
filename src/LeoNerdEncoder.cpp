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
#include "LeoNerdEncoder.h"
// #include "../../include/Debug.h"

/**
 * @brief Constructor
 *        Call using (simple method):
 *              LeoNerdEncoder encoder = LeoNerdEncoder(0x3D);
 *        or (utilizing interrupt and event handlers):
 *              LeoNerdEncoder encoder = LeoNerdEncoder(0x3D, myIntPin, myIntHandler, myEventHandler, true);
 * 
 * @param address               the I2C address of this module
 * @param intPin                the GPIO pin which will receive the interrupt sinal if the FIFO buffer is set
 * @param interruptHandler      pointer to the interrupt handler function (can be omitted)
 * @param eventHandler          pointer to the event handler function (can be omitted)
 * @param keyBeep               flag whether or not to use beeps on encoder/buttons
 */
LeoNerdEncoder::LeoNerdEncoder(uint8_t address, pin_t intPin, void (*interruptHandler)(void), void (*eventHandler)(LeoNerdEvent), bool keyBeep) {
    _address = address;

    if(keyBeep) {
        setKeyBeep(330, 100);
    }
    _intPin = intPin;
    if(intPin != 0) {
        pinMode(intPin, INPUT_PULLUP);
    }
    _interruptHandler = interruptHandler;
    _eventHandler = eventHandler;
    _encoderButton = Button(WheelButton);
    _mainButton = Button(MainButton);
    _leftButton = Button(LeftButton);
    _rightButton = Button(RightButton);
    _maxBrightness = 255;
    _gpioDir = 0;
    _gpioVal = 0;
    _wheelPos = 0;
    _accelEnabled = false;
    _isBusy = false;
}

/**
 * @brief Destructor for clean up
 */
LeoNerdEncoder::~LeoNerdEncoder() {
    if(_intPin != 0)
        detachInterrupt(_intPin);
    #if !defined(__ESP32__) && !defined(ESP8266)
    if (_I2CBus != nullptr) 
        _I2CBus->end();
    #endif
}

/**
 * @brief Initialize the encoder library
 * 
 * @param bus       instance of the I2C driver (either HW or SW I2C)
 * @param initBus   flag which determines whether the bus must be initialized (i.e. begin() is being called)
 */
void LeoNerdEncoder::internalBegin(I2CBusBase* bus, bool initBus /* = true */) {
    if (bus == nullptr) 
        return;

    _I2CBus = bus;
    if(initBus)             // avoid re-init of I2C bus if initBus is false
        _I2CBus->begin();
    _wheelPos = 0;
    _isBusy = false;
    flushFifo();
    if(_intPin != 0 && _interruptHandler != nullptr)
        attachInterrupt(_intPin, _interruptHandler, FALLING);
    // set release mask on all buttons by default
    setButtonReleaseMask();
}

/**
 * @brief Flushes the FIFO of the encoder
 */
void LeoNerdEncoder::flushFifo() {
    // flush FIFO, just in case
    if(_intPin != 0) {
        while(digitalRead(_intPin) == LOW) {
            queryRegister(REG_EVENT);
        }
    }
}

/**
 * @brief   Checks is encoder is busy. This is the case if 
 *          the @see service() method is being processed. 
 * 
 * @return true if is busy
 */
bool LeoNerdEncoder::busy(void) {
    return _isBusy;
}

/**
 * @brief Simple wrapper for service()
 *        Call this from somwhere in your main loop()
 * 
 * @param autoParse     @see service()
 * 
 * @returns @see service()
 */
uint8_t LeoNerdEncoder::loop(bool autoParse /*=true*/) {
    return service(autoParse);
}

/**
 * @brief Main routine for reading the events coming from the encoder
 * 
 * @param autoParse     flag which determines if parsing events is being done automatically (preset by default)
 * 
 * @returns the number of bytes received (0 if no data was pending)
 */
uint8_t LeoNerdEncoder::service(bool autoParse /*=true*/) {
    if(busy())
        return 0;
    memset(_evtBuffer, 0xaa, MAX_BUFFER);
    interrupts();
    // if an interrupt pin is assigned and its state is HIGH, there are no events pending
    if(_intPin != 0 && digitalRead(_intPin) == HIGH) {
        return 0;
    }
    _isBusy = true;
    uint8_t cnt = queryRegister(REG_EVENT, _evtBuffer, MAX_BUFFER);
    if(autoParse)
        parseEvents(cnt);
    _isBusy = false;
    return cnt;
}

/**
 * @brief Parses all events received from the FIFO buffer
 * 
 * @param cnt   the amount of data bytes read
 */
void LeoNerdEncoder::parseEvents(uint8_t cnt) {
    for(int i=0; i<cnt; i++) {
        // parse all data except 0xFF
        if(_evtBuffer[i] != 0xff)
            parseData(REG_EVENT, _evtBuffer[i]);
    }
}

/**
 * @brief Sets the button state according to the event received
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
        if(_eventHandler != nullptr)
            _eventHandler(LeoNerdEvent((EventType)instance->which(), instance->getButtonState()));
        instance->setLastButtonReleased(now);
    }
    if(state == Released) {
        if(instance->getButtonState() == Pressed) {
            instance->resetClickCount();
            instance->setButtonState(Clicked);
            instance->setLastButtonReleased(now);
            if(_eventHandler != nullptr)
                _eventHandler(LeoNerdEvent((EventType)instance->which(), instance->getButtonState()));
        }
    }
}

/**
 * @brief Parse the event and act accordingly
 *
 * @param data      the event received
 */
void LeoNerdEncoder::parseEvent(uint8_t data) {

    // special handling for encoder wheel acceleration as implemented
    // in GMagician firmware
    uint8_t steps = 1;
    if(data > 0x20 && data < 0x3F) {
        steps = ((data & 0x1C) >> 2)+1;
	    data &= 0x23;
    }
    uint8_t gpio = 0;
    if(data >= 0x60 && data <= 0x6F) {
        gpio = (data & 0x0f);
	    data &= 0x60;
    }

    switch(data) {
        case EVENT_NONE:
            _wheelPos = 0;
            break;
        case EVENT_WHEEL_DOWN:
            _wheelPos -= steps;
            if(_eventHandler != nullptr)
                _eventHandler(LeoNerdEvent(WheelEvent, LeftTurn));
            break;
        case EVENT_WHEEL_UP:
            _wheelPos += steps;
            if(_eventHandler != nullptr)
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
            _gpioVal = gpio;
            if(_eventHandler != nullptr)
                _eventHandler(LeoNerdEvent(GpioEvent, Open, _gpioVal));
            break;
        case 0xff:
            break;
        default:
            break;
    }
}

/**
 * @brief Parse the data received according to the addressed register
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
 * @brief Enables/Disables the double click feature on button
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
 * @brief Reads the state of a button
 *
 * @param which     the button @see Button
 * 
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
 * @brief Resets the state of the button
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
 * @brief Resets the state of all buttons
 */
void LeoNerdEncoder::resetButtons() {
    _encoderButton.resetButtonState();
    _mainButton.resetButtonState();
    _leftButton.resetButtonState();
    _rightButton.resetButtonState();
}

/**
 * @brief Switches the given LED on or off
 *
 * @param which     number of the LED 1 or 2
 * @param state     true (on) / false (off)
 */
void LeoNerdEncoder::setLED(uint8_t which, bool state) {
    waitBusy();
    if(which >= 1 && which <= MAX_LEDS) {
        which--;
        _leds[which] = state ? _maxBrightness : 0;
        sendData(REG_LED1_PWM + which, _leds[which]);
    }
}

/**
 * @brief Toggles the given LED
 *
 * @param which     number of the LED 1 or 2
 * @param state     true (on) / false (off)
 */
void LeoNerdEncoder::toggleLED(uint8_t which) {
    waitBusy();
    if(which >= 1 && which <= MAX_LEDS) {
        which--;
        _leds[which] = _leds[which]==0 ? _maxBrightness : 0;
        sendData(REG_LED1_PWM + which, _leds[which]);
    }
}

/**
 * @brief Sets the given GPIO pin to a certian mode
 *
 * @param which     number of the GPIO pin (0-3)
 * @param mode      INPUT or OUTPUT
 */
#if defined(__STM32F1__) && defined(__LIBMAPLE__)
void LeoNerdEncoder::setGPIOMode(uint8_t which, WiringPinMode mode)
#else
void LeoNerdEncoder::setGPIOMode(uint8_t which, int mode)
#endif
{
    waitBusy();
    _gpioDir = (mode == INPUT) ? _gpioDir & ~(1<<which) : _gpioDir | (1<<which);
    sendData(REG_GPIO_DIR, _gpioDir & 0x0f);
}

/**
 * @brief Sets the given GPIO pin to a certain state
 *
 * @param which     number of the GPIO pin (0-3)
 * @param state     true (HIGH) / false (LOW)
 */
void LeoNerdEncoder::setGPIO(uint8_t which, bool state){
    waitBusy();
    _gpioVal = (!state) ? _gpioVal & ~(1<<which) : _gpioVal | (1<<which);
    sendData(REG_GPIO_IO, _gpioVal & 0x0f);
}

/**
 * @brief Gets the given GPIO pin state
 *
 * @param which     number of the GPIO pin (0-3)
 * 
 * @returns true for HIGH, false for LOW
 */
bool LeoNerdEncoder::getGPIO(uint8_t which) {
    waitBusy();
    return (_gpioVal & (1<<which)) > 0;
}

/**
 * @brief Plays a tone on the buzzer in units of 10 Hz
 *
 * @param frequency     the tone frequency
 * @param duration      the tone duration in milliseconds
 *
 */
void LeoNerdEncoder::playTone(int frequency, int duration) {
    waitBusy();
    sendData(REG_BEEP_TONE,     (uint8_t)(frequency/10));
    sendData(REG_BEEP_DURATION, (uint8_t)(duration/10));
}

/**
 * @brief Plays a frequency on the buzzer in units of 1 Hz
 *
 * @param frequency     the tone frequency as 16 bit value
 * @param duration      the tone duration in milliseconds
 *
 */
void LeoNerdEncoder::playFrequency(int frequency, int duration) {
    waitBusy();
    sendData(REG_BEEP_FREQH,    (uint8_t)(frequency >> 8), (uint8_t)(frequency & 0xFF));
    sendData(REG_BEEP_DURATION, (uint8_t)(duration/10));
}

/**
 * @brief Mutes the buzzer
 */
void LeoNerdEncoder::muteTone() {
    waitBusy();
    sendData(REG_BEEP_DURATION, 0);
}

/**
 * @brief Sets the KeyBeep frequency and duration
 *
 * @param frequency     the tone frequency
 * @param duration      the tone duration in milliseconds
 *
 */
void LeoNerdEncoder::setKeyBeep(int frequency, int duration) {
    waitBusy();
    sendData(REG_BEEP_TONE, (uint8_t)frequency/10);
    setKeyBeepDuration((uint8_t)duration/10);
    setKeyBeepMask();
}

/**
 * @brief Sets the KeyBeep duration
 * 
 * @param duration      the tone duration in milliseconds
 */
void LeoNerdEncoder::setKeyBeepDuration(uint8_t duration) {
    waitBusy();
    sendData(REG_KEYBEEP_DURATION, duration);
}

/**
 * @brief Sets the KeyBeep mask
 * 
 * @param mask      mask to set (which buttons will beep on action)
 */
void LeoNerdEncoder::setKeyBeepMask(uint8_t mask) {
    waitBusy();
    sendData(REG_KEYBEEP_MASK, mask);
}

/**
 * @brief Sets the debounce time of buttons
 * 
 * @param time      time in milliseconds (0-255)
 */
void LeoNerdEncoder::setDebounceTime(uint8_t time) {
    waitBusy();
    sendData(REG_DEBOUNCE_TIME, time);
}

/**
 * @brief Sets the encoder button hold time for "Long Click"
 * 
 * @param time  time in centiseconds (0-255)
 */
void LeoNerdEncoder::setButtonHoldTime(uint8_t time) {
    waitBusy();
    sendData(REG_BTNHOLD_TIME, time);
}

/**
 * @brief Sets the release mask of buttons
 * 
 * @param mask  button release mask @see LeoNerdEncoder.h
 */
void LeoNerdEncoder::setButtonReleaseMask(uint8_t mask) {
    waitBusy();
    sendData(REG_RELEASEMASK, mask);
}

/**
 * @brief Enables the EEPROM for writing
 *
 * This is a security measure to avoid accidentally overwriting the
 * EEPROM if there're some unwanted signals on the I2C bus.
 * This method is called for each setEepromValue() operation.
 */
void LeoNerdEncoder::unlockEepromWrite() {
    waitBusy();
    sendData(REG_EEPROM_UNLOCK, 0x12);
    sendData(REG_EEPROM_UNLOCK, 0x34);
}

/**
 * @brief Programs a value in the EEPROM
 *
 * Be careful with this method and know what you're doing!
 * It may render your encoder unusable!
 *
 * @param eep_adr   the EEPROM address @see LeoNerdEncoder.h (REG_EEPROM*)
 * @param value     the value to be written
 */
void LeoNerdEncoder::setEepromValue(uint8_t eep_adr, uint8_t value) {
    waitBusy();
    if(eep_adr >= REG_EEPROM && eep_adr <= REG_EEPROM_BTN_POLARITY) {
        unlockEepromWrite();
        sendData(eep_adr, value);
    }
}


/**
 * @brief Reads the RELEASEMASK
 *
 * @returns the current value
 */
uint8_t LeoNerdEncoder::queryReleaseMask() {
    waitBusy();
    return queryRegister(REG_RELEASEMASK);
}

/**
 * @brief Reads the KEYBEEP DURATION
 *
 * @returns the current value
 */
uint8_t LeoNerdEncoder::queryKeyBeepDuration() {
    waitBusy();
    return queryRegister(REG_KEYBEEP_DURATION);
}

/**
 * @brief Reads the KEYBEEP MASK
 *
 * @returns the current value
 */
uint8_t LeoNerdEncoder::queryKeyBeepMask() {
    waitBusy();
    return queryRegister(REG_KEYBEEP_MASK);
}

/**
 * @brief Reads the BEEP DURATION
 *
 * @returns the current value
 */
uint8_t LeoNerdEncoder::queryBeepDuration() {
    waitBusy();
    return queryRegister(REG_BEEP_DURATION);
}

/**
 * @brief Reads the BEEP TONE
 *
 * @returns the current value
 */
uint8_t LeoNerdEncoder::queryBeepTone() {
    waitBusy();
    return queryRegister(REG_BEEP_TONE);
}

/**
 * @brief Reads the PWM value for LED1
 *
 * @returns the current value
 */
uint8_t LeoNerdEncoder::queryLed1Pwm() {
    waitBusy();
    return queryRegister(REG_LED1_PWM);
}

/**
 * @brief Reads the PWM value for LED2
 *
 * @returns the current value
 */
uint8_t LeoNerdEncoder::queryLed2Pwm() {
    waitBusy();
    return queryRegister(REG_LED2_PWM);
}

/**
 * @brief Reads the GPIO DIRECTION settings
 *
 * @returns the current value
 */
uint8_t LeoNerdEncoder::queryGpioDir() {
    waitBusy();
    return queryRegister(REG_GPIO_DIR);
}

/**
 * @brief Reads the GPIO states
 *
 * @returns the current value
 */
uint8_t LeoNerdEncoder::queryGpioIo() {
    waitBusy();
    return queryRegister(REG_GPIO_IO);
}

/**
 * @brief Reads the GPIO PULLUP settings
 *
 * @returns the current value
 */
uint8_t LeoNerdEncoder::queryGpioPullup() {
    waitBusy();
    return queryRegister(REG_GPIO_PULLUP);
}

/**
 * @brief Reads the GPIO EVENTMASK settings
 *
 * @returns the current value
 */
uint8_t LeoNerdEncoder::queryGpioEventMask() {
    waitBusy();
    return queryRegister(REG_GPIO_EVENTMASK);
}

/**
 * @brief Queries the ENCODER ADDRESS
 *
 * @returns the configured address
 */
uint8_t LeoNerdEncoder::queryEncoderAddress() {
    waitBusy();
    return queryRegister(REG_EEPROM);
}

/**
 * @brief Reads the OPTIONS register
 *
 * @returns the options as set in EEPROM
 */
uint8_t LeoNerdEncoder::queryOptions() {
    waitBusy();
    return queryRegister(REG_EEPROM_OPTIONS);
}

/**
 * @brief Reads the DEBOUNCE TIME
 *
 * @returns the default time
 */
uint8_t LeoNerdEncoder::queryDebounceTime() {
    waitBusy();
    return queryRegister(REG_EEPROM_DEBOUNCE);
}

/**
 * @brief Reads the HOLD TIME
 *
 * @returns the default time
 */
uint8_t LeoNerdEncoder::queryHoldTime() {
    waitBusy();
    return queryRegister(REG_EEPROM_HOLDTIME);
}

/**
 * @brief Reads the BUTTON TO GPIO MAPPING
 *
 * @returns the current GPIO ports it's mapped to
 */
uint8_t LeoNerdEncoder::queryButtonMapping(Buttons button) {
    waitBusy();
    uint8_t stat = queryRegister(REG_EEPROM_BTN_MAPPING);
    return assignButton(button, stat);
}

/**
 * @brief Reads the BUTTON TO GPIO MAPPING POLARITY
 *
 * @returns     the current GPIO ports it's mapped to
 */
uint8_t LeoNerdEncoder::queryButtonMappingPolarity(Buttons button) {
    waitBusy();
    uint8_t stat = queryRegister(REG_EEPROM_BTN_POLARITY);
    return assignButton(button, stat);
}

/**
 * @brief Reads the WHEEL ACCELERATION time
 *
 * @returns the default time in ms
 */
uint8_t LeoNerdEncoder::queryWheelAcceleration() {
    waitBusy();
    return queryRegister(REG_EEPROM_ACCELERATION);
}

/**
 * @brief Reads the WHEEL DECELERATION time
 *
 * @returns the default time in ms
 */
uint8_t LeoNerdEncoder::queryWheelDeceleration() {
    waitBusy();
    return queryRegister(REG_EEPROM_DECELERATION);
}

/**
 * @brief Reads the firmware VERSION info
 *
 * @returns the current value
 */
uint8_t LeoNerdEncoder::queryVersion() {
    waitBusy();
    return queryRegister(REG_SW_VERSION);
}

/**
 * @brief Queries the value of the given (FIFO buffered) register
 * 
 * @param reg       the register in charge
 * @param buffer    the pointer to the result buffer
 * @param size      the max. size of the result buffer
 *
 * @returns the number of bytes received from FIFO; buffer gets filled accordingly
 */
uint8_t LeoNerdEncoder::queryRegister(uint8_t reg, uint8_t* buffer, uint8_t size) {
    if (_I2CBus == nullptr) 
        return 0;

    uint8_t stat = sendRequest(reg);
    if(stat == 0) {
        uint8_t cnt = _I2CBus->requestFrom(_address, size);
        while(!_I2CBus->available())
            ;
        uint8_t ndx = 0;
        while(ndx < cnt) {
            uint8_t response = _I2CBus->read();
            if(ndx < size)
                *(buffer+ndx) = response;
            ndx++;
        }
        return cnt;
    }
    return 0;
}

/**
 * @brief Queries the value of the given register
 * 
 * @param reg   the register in charge
 *
 * @returns the response received
 */
uint8_t LeoNerdEncoder::queryRegister(uint8_t reg) {
    if (_I2CBus == nullptr) 
        return 0;

    sendRequest(reg);
    _I2CBus->requestFrom(_address, (uint8_t)1);
    while(!_I2CBus->available())
        ;
    return _I2CBus->read();
}

/**
 * @brief Waits until the busy flag gets released
 */
void LeoNerdEncoder::waitBusy(void) {
    while(busy()) {
    }
}

/**
 * @brief Sends a register read request. Device will respond with data.
 * 
 * @param reg       the register to read
 * 
 * @returns the status of the transmission operation
*/
uint8_t LeoNerdEncoder::sendRequest(uint8_t reg) {
    if (_I2CBus == nullptr) 
        return 0;
    uint8_t stat = 0;
    do {
        _I2CBus->beginTransmission(_address);
        _I2CBus->write(reg);
        stat = _I2CBus->endTransmission();
    } while(stat > 1);
    return stat;
}

/**
 * @brief Sends one byte of data to the specified register
 * 
 * @param reg       the register to write to
 * @param data      the (8-bit) data to be written
 * 
 * @returns the status of the transmission operation
 */
uint8_t LeoNerdEncoder::sendData(uint8_t reg, uint8_t data) {
    if (_I2CBus == nullptr) 
        return 0;
    _I2CBus->beginTransmission(_address);
    _I2CBus->write(reg);
    _I2CBus->write((uint8_t)data);
    return _I2CBus->endTransmission();
}

/**
 * @brief Sends two bytes of data to the specified register
 * 
 * @param reg       the register to write to
 * @param data1     the first byte to be written
 * @param data2     the second byte to be written
 * 
 * @returns the status of the transmission operation
 */
uint8_t LeoNerdEncoder::sendData(uint8_t reg, uint8_t data1, uint8_t data2) {
    if (_I2CBus == nullptr) 
        return 0;
    _I2CBus->beginTransmission(_address);
    _I2CBus->write(reg);
    _I2CBus->write((uint8_t)data1);
    _I2CBus->write((uint8_t)data2);
    return _I2CBus->endTransmission();
}

/**
 * @brief Assigns button status (helper)
*/
uint8_t assignButton(uint8_t button, uint8_t stat) {
    switch(button) {
        case WheelButton:
            return (stat & 1);
        case MainButton:
            return (stat & 2) >> 1;
        case LeftButton:
            return (stat & 4) >> 2;
        case RightButton:
            return (stat & 8) >> 3;
        default:
            return 0;
    }
}
