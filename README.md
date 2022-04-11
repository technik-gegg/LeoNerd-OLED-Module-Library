# LeoNerd-OLED-Module-Library

This is a standalone library for LeoNerd's [OLED Front Panel Module](https://www.tindie.com/products/leonerd/oled-front-panel-module/) for Arduino projects.
![Image](https://cdn.tindiemedia.com/images/resize/vT8K8YJx4fqYTFgQj5vLBKUB6yM=/p/full-fit-in/2400x1600/i/46252/products/2019-02-09T14%3A52%3A38.181Z-IMG_6938.jpg)

This nice little module from designer **Paul "LeoNerd" Evans** comes with all the components needed for sophisticated Input/Output in your DIY projects by utilizing the I2C interface. This requires only 4 wires (**VCC**, **GND**, **SCL**, **SDA**) to connect it and to establish a communication with a decent speed.

This library is a simple wrapper for the I2C commands used to communicate with the device and which are described in detail in the [datasheet](https://d3s5r33r268y59.cloudfront.net/datasheets/16167/2020-09-09-16-08-39/driver.pdf) of the module.
It takes care of the handling of the I2C encoder, buttons, LEDs and buzzer, as well as the [U8G2 library from Oliver Kraus](https://github.com/olikraus/u8g2) for putting data onto the SH1106 OLED display.

## Basic usage

To use this library in your Arduino project, simply place a link to this github repository into your platformio.ini's *lib_deps*, i.e.:

```bash
lib_deps =  ...
            https://github.com/technik-gegg/LeoNerd-OLED-Module-Library.git
            ...
```

If you're going to compile for an STM32F1xx MCU using the **Maple** framework (*board_build.core=maple*), add the next two lines to your platformio.ini as well:

```bash
board_build.core   = maple
build_flags        = -D __LIBMAPLE__  # compile for Maple framework
```

The "basic" sample in the examples folder will show you how to use this library.

### Usage In A Nutshell

You have to create two object instances:

+ one for the U8G2 library (or any other library which supports an SH1106 I2C OLED display)
+ one for the LeoNerdEncoder library

I.e., by using these global instance variables:

```cpp
#define ENCODER_ADDRESS     0x3D    // use the default address

U8G2_SH1106_128X64_NONAME_F_HW_I2C  display(U8G2_R2, /* reset=*/ U8X8_PIN_NONE);
LeoNerdEncoder                      encoder(ENCODER_ADDRESS);

```

In your main *setup()* routine add the *begin()* method of each to initialize them:

```cpp
void setup() {
    ...
    display.begin();
    encoder.begin();
    ...
}
```

In the master *loop()* of your program call the **encoder.loop()** first, then read out button states and encoder value and react accordingly.

```cpp
ButtonState wheelBtn, mainBtn, leftBtn, rightBtn;
int16_t encoderPos;

void loop() {
    ...
    encoder.loop();
    encoderPos += encoder.getValue();
    wheelBtn   = encoder.getButton(WheelButton);
    mainBtn    = encoder.getButton(MainButton);
    leftBtn    = encoder.getButton(LeftButton);
    rightBtn   = encoder.getButton(RightButton);

    if(mainBtn == Clicked) {
        ...
    }
    ...
}
```

For sending data to the display, use the well known methods of the **U8G2 library**, [as explained here](https://github.com/olikraus/u8g2/wiki/u8g2reference). The simplest one is:

```cpp
    ...
    int x = 4, y = 10;

    display.firstPage();
    do {
        drawStr(x, y, text1);
        drawStr(x, y+10, text2);
        ...
    } while(display.nextPage());
    ...
```

Beside reading the encoder and the button states, you have these additional methods for handling the buzzer, the LEDs and the I/O pins of the module.

```cpp

void playFrequency(int frequency, int duration);
void muteTone(void);
void setLED(uint8_t which, bool state);
void toggleLED(uint8_t which);
void setGPIO(uint8_t which, bool state);
bool getGPIO(uint8_t which);
```

### Please notice

The I2C bus of this device is not terminated!
This means: If this module is the only I2C device on your bus, you have to terminate the bus by adding a **2.2K - 4.7K** pull-up resistor to the **SDA** as well as the **SCL** line.

If you have another I2C device connected to your MCU which comes with such pull-ups already applied to SDA/SCL, you **must not** add another set of resistors.

## Change History

**2022-04-10**

+ changed interrupt pin type to *pin_t* for use with STM32 Core (uint8_t on Arduino, uint32_t on STM32 Core)
+ moved all files into **src** folder and removed **include** folder (for easier debugging)
+ changed signature of *loop()* and *service()* and added optional **autoParse** flag
+ added *parseEvents()* method to be called from outside if *loop()* or *service()* have been called with *autoParse = false*
+ changed all occurances of *NULL* with *nullptr* to avoid compiler ambiguities
+ refactored library to reduce code and enhance readability; Added *sendData()* function for sending 1 or 2 bytes to the OLED Module and *sendRequest()* function for reading register values 
+ added **USE_SW_TWI** flag (needed for STM32 Core) if you'd like to use software instead of hardware TWI/I2C. You'll need to include the [SoftWire](https://github.com/stevemarple/SoftWire.git) (which also requires the [AsyncDelay](https://github.com/stevemarple/AsyncDelay.git)) library and set up an instance of SoftWire, which then needs to be passed to the library in the *begin()* method.  

**2022-03-20**

+ added conditional compilation for STM32F1xx. Using a STM32F1 MCU under the Maple framework (*board_build.core = maple*), needs adding the definition
**-D \_\_LIBMAPLE\_\_** to make the library compile properly
+ added *queryOptions()* method to read out the Options register
+ added encoder wheel acceleration handling as used in GMagicians alternative firmware version
+ updated basic example (added playing tune, decreased the memory footprint)
+ added conditional compiling for ESP32 since *Wire.end()* is unknown to the framework
