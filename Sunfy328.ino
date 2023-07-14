//   <!--------------------------------------------------------------------------------------------->
//   <!--    SUNFY328                                                                             -->
//   <!--    IRRIGATION CONTROL UNIT                                                              -->
//   <!--    Hardware and instructions at www.picapot.com                                         -->
//   <!--                                                                                         -->
//   <!--    This program is free software; you can redistribute it and/or                        -->
//   <!--    modify it under the terms of the GNU Lesser General Public                           -->
//   <!--    License as published by the Free Software Foundation; either                         -->
//   <!--    version 3.0 of the License, or (at your option) any later version.                   -->
//   <!--    Any re-distribution of this software, even if modified, MUST remain open-source      -->
//   <!--    and MUST show the original author full name and the GITHUB code source link          -->
//   <!--    and MUST inherit and apply the same GNU license.                                     -->
//   <!--                                                                                         -->
//   <!--    This library is distributed in the hope that it will be useful,                      -->
//   <!--    but WITHOUT ANY WARRANTY; without even the implied warranty of                       -->
//   <!--    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU                    -->
//   <!--    Lesser General Public License for more details.                                      -->
//   <!--------------------------------------------------------------------------------------------->
#include <avr/wdt.h>              // watchdog library https://www.nongnu.org/avr-libc/user-manual/group__avr__watchdog.html
#include <avr/sleep.h>            // sleep mode library https://www.nongnu.org/avr-libc/user-manual/group__avr__sleep.html
#include <avr/power.h>            // power control library https://www.nongnu.org/avr-libc/user-manual/group__avr__power.html 
#include <Wire.h>                 // I2C protocol library https://github.com/esp8266/Arduino/blob/master/libraries/Wire/Wire.h
#include <OneWire.h>              // 1-Wire protocol library https://github.com/PaulStoffregen/OneWire
#include <Dusk2Dawn.h>            // daily dusk and dawn library https://github.com/Ipposnif/Dusk2Dawn fork of https://github.com/dmkishi/Dusk2Dawn 
#include "SSD1306AsciiAvrI2c.h"   // SH1106/SSD1306/SSD1309 128x64 I2C screen library https://github.com/Ipposnif/SSD1306Ascii fork of https://github.com/greiman/SSD1306Ascii 
#include "SSD1306AsciiSpi.h"      // same library as above but working with the SPI interface

// SYSTEM SETTINGS ////////////////////////////////////////////////////////
#define SCREEN_INTERFACE 1            // 0=I2C screen connected to port P7 or port P3  1=SPI screen connected to port P8
#define SCREEN_TYPE &Adafruit128x64   // use &SH1106_128x64 for the SH1106 screen and &Adafruit128x64 for the SSD1306/SSD1309 screen
#define SCREEN_SLEEP_SEC 120          // turn OFF the screen after n seconds of inactivity. 0=screen always ON.
#define SCREEN_CONTRAST 230           // 0-255
#define INT_TEMP_COMP 0               // compensation in celsius applied to the internal atmega328 temperature
#define CLOCK_ADDRESS 0x68            // I2C address of the Real Time Clock DS1307
#define SCREEN_ADDRESS 0x3C           // I2C address of the I2C screen 
#define CATNIP_SENSOR_ADDRESS 0x20    // I2C address of the Catnip sensor 
#define BLINK_FREQ 200                // blinking frequency in ms for the edit mode
#define EDIT_FREQ 70                  // buttons processing frequency. It sets the speed for auto increase/decrease when the buttons 2 and 3 are kept pressed in edit mode.
#define DEBOUNCE_DELAY 25             // button debouncing delay in ms
#define SYSTEM_DELAY 5                // delay in ms after each wire bulk read/write
#define HEAT_MULTIPLIER 30            // percentage increase of the watering duration. It is applied when the user option HeatWave>0 and the max temperature of the day is equal to or greater than HeatWave

// DEFAULT VALUES OVERRIDE ////////////////////////////////////////////////////////
#define DEFAULT_LONGITUDE +14         // after a reset the system starts with this longitude value (Malta). To reset the control unit, restart it without the CR2032 backup battery.
#define DEFAULT_LATITUDE +35          // after a reset the system starts with this latitude value (Malta).
#define DEFAULT_TIMEZONE +4           // after a reset the system starts with this timezone value. The timezone is expressed counting the quarters d'hour present in the desired timezone. Default is +4 (+01.00 Berlin, Rome, Paris, Madrid, Warsaw, Malta). For example, Venezuela(-04:30) has value -18.
#define DEFAULT_DAYLIGHT_SAVING 1     // after a reset the system starts with this daylight saving value. 0=daylight saving OFF 1=daylight saving ON
#define DEFAULT_ALARM_STATUS 1        // 0=default setting is both alarms OFF. If the backup battery runs flat, after a black-out sunfy328 will not water and wait for a reset confirmation from the user. This option is to make installations safe during watering hours.
                                      // 1=default setting is both alarms ON. If the backup battery runs flat, after a black-out sunfy328 will automatically reset itself and it will water using the default settings (at a ramdomly wrong time). This is to avoid that Sunfy328 stops watering if no one changes the battery.
                                      
// PINS ////////////////////////////////////////////////////////
#define PIN_INT0 2                   // Interrupt 0 wired to the 1Hz square wave of the RTC
#define PIN_LED 0                    // red LED
#define PIN_ONE_WIRE A3              // I/O pin used to communicate with the humidity sensor using the 1-Wire protocol  
#define PIN_SPI_CS  9                // CS pin for the SPI screen
#define PIN_SPI_RST A1               // RST pin for the SPI screen
#define PIN_SPI_DC  10               // DC pin for the SPI screen    
#define PIN_BTN1 1                   // button 1 (left)
#define PIN_BTN2 4                   // button 2 (middle)
#define PIN_BTN3 3                   // button 3 (right)
#define PIN_OUT 8                    // Output signal wired to the MOSFET gate (opens the solenoid valve)    
#define PIN_BAT_CHECK A2             // Analog input pin used to check the battery's voltage

//MACROS ////////////////////////////////////////////////////////
#define SWAP(a, b) { int16_t t = a; a = b; b = t; }

//FONTS ////////////////////////////////////////////////////////
// Free Font LCD5x7 revisited using GLCD Font Creator https://www.mikroe.com/glcd-font-creator. The size is doubled when printed (10x14)
// Some chars have been changed to icons
// {=TOP ARROW  |=BOTTOM ARROW  }=WATER DROP  ~=DEGREE CELSIUS  ,=DEGREE FAHRENHEIT
// "=BATTERY BOTTOM FULL  ;=BOTTOM 2/3  _=BOTTOM 1/3  `=BOTTOM EMPTY  #=TOP FULL  '=TOP EMPTY
// Some chars are reserved and used as text formatting elements:
// &=PLACEHOLDER  [=1x PIXEL SPACE  ]=4x PIXEL SPACE ^=LINEFEED
GLCDFONTDECL(lcd5x7m) = {
  0x00, 0x00, 0x05, 0x07, 0x20, 0x60, // 0, 0, width, height, first char, char count(96)
  0x00, 0x00, 0x00, 0x00, 0x00,   0x00, 0x00, 0x4F, 0x00, 0x00,   0x7E, 0x42, 0x5A, 0x5A, 0x5A,   0x5A, 0x42, 0x66, 0x3C, 0x3C,   0x24, 0x2A, 0x7F, 0x2A, 0x12, // SPACE ! " # $
  0x23, 0x13, 0x08, 0x64, 0x62,   0x36, 0x49, 0x55, 0x22, 0x50,   0x42, 0x42, 0x66, 0x3C, 0x3C,   0x00, 0x1C, 0x22, 0x41, 0x00,   0x00, 0x41, 0x22, 0x1C, 0x00, // % & ' ( )
  0x14, 0x08, 0x3E, 0x08, 0x14,   0x08, 0x08, 0x3E, 0x08, 0x08,   0x02, 0x05, 0x7E, 0x14, 0x04,   0x08, 0x08, 0x08, 0x08, 0x08,   0x00, 0x60, 0x60, 0x00, 0x00, // * + , - .
  0x20, 0x10, 0x08, 0x04, 0x02,   0x3E, 0x51, 0x49, 0x45, 0x3E,   0x00, 0x42, 0x7F, 0x40, 0x00,   0x42, 0x61, 0x51, 0x49, 0x46,   0x21, 0x41, 0x45, 0x4B, 0x31, // / 0 1 2 3
  0x18, 0x14, 0x12, 0x7F, 0x10,   0x27, 0x45, 0x45, 0x45, 0x39,   0x3C, 0x4A, 0x49, 0x49, 0x30,   0x01, 0x71, 0x09, 0x05, 0x03,   0x36, 0x49, 0x49, 0x49, 0x36, // 4 5 6 7 8
  0x06, 0x49, 0x49, 0x29, 0x1E,   0x00, 0x00, 0x24, 0x00, 0x00,   0x7E, 0x42, 0x5A, 0x5A, 0x42,   0x08, 0x14, 0x22, 0x41, 0x00,   0x00, 0x14, 0x14, 0x14, 0x00, // 9 : ; < =
  0x00, 0x41, 0x22, 0x14, 0x08,   0x02, 0x01, 0x51, 0x09, 0x06,   0x30, 0x49, 0x79, 0x41, 0x3E,   0x7E, 0x11, 0x11, 0x11, 0x7E,   0x7F, 0x49, 0x49, 0x49, 0x36, // > ? @ A B
  0x3E, 0x41, 0x41, 0x41, 0x22,   0x7F, 0x41, 0x41, 0x22, 0x1C,   0x7F, 0x49, 0x49, 0x49, 0x41,   0x7F, 0x09, 0x09, 0x09, 0x01,   0x3E, 0x41, 0x49, 0x49, 0x7A, // C D E F G
  0x7F, 0x08, 0x08, 0x08, 0x7F,   0x00, 0x41, 0x7F, 0x41, 0x00,   0x20, 0x40, 0x41, 0x3F, 0x01,   0x7F, 0x08, 0x14, 0x22, 0x41,   0x7F, 0x40, 0x40, 0x40, 0x40, // H I J K L
  0x7F, 0x02, 0x0C, 0x02, 0x7F,   0x7F, 0x04, 0x08, 0x10, 0x7F,   0x3E, 0x41, 0x41, 0x41, 0x3E,   0x7F, 0x09, 0x09, 0x09, 0x06,   0x3E, 0x41, 0x51, 0x21, 0x5E, // M N O P Q
  0x7F, 0x09, 0x19, 0x29, 0x46,   0x46, 0x49, 0x49, 0x49, 0x31,   0x01, 0x01, 0x7F, 0x01, 0x01,   0x3F, 0x40, 0x40, 0x40, 0x3F,   0x1F, 0x20, 0x40, 0x20, 0x1F, // R S T U V
  0x3F, 0x40, 0x30, 0x40, 0x3F,   0x63, 0x14, 0x08, 0x14, 0x63,   0x07, 0x08, 0x70, 0x08, 0x07,   0x61, 0x51, 0x49, 0x45, 0x43,   0x00, 0x7F, 0x41, 0x41, 0x00, // W X Y Z [
  0x02, 0x04, 0x08, 0x10, 0x20,   0x00, 0x41, 0x41, 0x7F, 0x00,   0x04, 0x02, 0x01, 0x02, 0x04,   0x7E, 0x42, 0x5A, 0x42, 0x42,   0x7E, 0x42, 0x42, 0x42, 0x42, // \ ] ^ _ `
  0x20, 0x54, 0x54, 0x54, 0x78,   0x7F, 0x50, 0x48, 0x48, 0x30,   0x38, 0x44, 0x44, 0x00, 0x00,   0x38, 0x44, 0x44, 0x48, 0x7F,   0x38, 0x54, 0x54, 0x54, 0x18, // a b c d e
  0x08, 0x7E, 0x09, 0x01, 0x02,   0x0C, 0x52, 0x52, 0x52, 0x3E,   0x7F, 0x08, 0x04, 0x04, 0x78,   0x00, 0x44, 0x7D, 0x40, 0x00,   0x20, 0x40, 0x44, 0x3D, 0x00, // f g h i j
  0x7F, 0x10, 0x28, 0x44, 0x00,   0x00, 0x41, 0x7F, 0x40, 0x00,   0x78, 0x0C, 0x18, 0x0C, 0x78,   0x7C, 0x08, 0x04, 0x04, 0x78,   0x38, 0x44, 0x44, 0x44, 0x38, // k l m n o
  0x7C, 0x14, 0x14, 0x14, 0x08,   0x08, 0x14, 0x14, 0x18, 0x7C,   0x7C, 0x08, 0x04, 0x04, 0x08,   0x48, 0x54, 0x54, 0x54, 0x20,   0x04, 0x3F, 0x44, 0x40, 0x20, // p q r s t
  0x3C, 0x40, 0x40, 0x20, 0x7C,   0x1C, 0x20, 0x40, 0x20, 0x1C,   0x3C, 0x40, 0x30, 0x40, 0x3C,   0x44, 0x28, 0x10, 0x28, 0x44,   0x0C, 0x50, 0x50, 0x50, 0x3C, // u v w x y
  0x44, 0x64, 0x54, 0x4C, 0x44,   0x0C, 0x06, 0x7F, 0x06, 0x0C,   0x18, 0x30, 0x7F, 0x30, 0x18,   0x1C, 0x22, 0x27, 0x2E, 0x1C,   0x02, 0x05, 0x3A, 0x44, 0x44, // z { | } ~
  0x00, 0x00, 0x00, 0x00, 0x00
};

// Time Font. It is Arial Narrow revisited
GLCDFONTDECL(arialNarrow15x27) = {
0x00, 0x00, 0X0F, 0X1B, 0x2F, 0x0C, // 0, 0, width, height, first char, char count(12)
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //  Code for char / (changed to space for the blink effect during edit)  
0x00, 0x00, 0xC0, 0xE0, 0xF0, 0xF8, 0x78, 0x78, 0x78, 0xF8, 0xF0, 0xE0, 0xC0, 0x00, 0x00, 0x00, 0xFE, 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x1F, 0xFF, 0xFF, 0xFF, 0xE0, 0x80, 0x80, 0x80, 0xE0, 0xFF, 0xFF, 0xFF, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x07, 0x07, 0x07, 0x07, 0x07, 0x03, 0x01, 0x00, 0x00, 0x00,  //  Code for char 0  
0x00, 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xF8, 0xF8, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x07, 0x03, 0x01, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x00,  //  Code for char 1  
0x00, 0x80, 0xE0, 0xF0, 0xF0, 0xF8, 0x78, 0x78, 0x78, 0xF8, 0xF0, 0xF0, 0xE0, 0x80, 0x00, 0x00, 0x03, 0x03, 0x03, 0x03, 0x00, 0x00, 0x80, 0xC0, 0xF0, 0xFF, 0xFF, 0x7F, 0x0F, 0x00, 0x00, 0x00, 0xE0, 0xF0, 0xFC, 0xFE, 0xFF, 0x8F, 0x87, 0x83, 0x81, 0x80, 0x80, 0x80, 0x00, 0x00, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x00,  //  Code for char 2  
0x00, 0x00, 0xC0, 0xF0, 0xF0, 0xF8, 0x78, 0x78, 0x78, 0xF8, 0xF0, 0xF0, 0xE0, 0x80, 0x00, 0x00, 0x03, 0x03, 0x03, 0x03, 0x00, 0xF0, 0xF0, 0xF0, 0xF8, 0xFF, 0xFF, 0x9F, 0x07, 0x00, 0x00, 0x18, 0xF8, 0xF8, 0xF8, 0xE0, 0x80, 0x80, 0x80, 0xE1, 0xFF, 0xFF, 0xFF, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x07, 0x07, 0x07, 0x07, 0x07, 0x03, 0x03, 0x01, 0x00, 0x00,  //  Code for char 3  
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xE0, 0xF8, 0xF8, 0xF8, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x80, 0xE0, 0xF8, 0xFC, 0x3F, 0x0F, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x3F, 0x3F, 0x3F, 0x3F, 0x3C, 0x3C, 0x3C, 0x3C, 0xFF, 0xFF, 0xFF, 0xFF, 0x3C, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x07, 0x07, 0x07, 0x00, 0x00,  //  Code for char 4  
0x00, 0x00, 0xC0, 0xF8, 0xF8, 0xF8, 0x78, 0x78, 0x78, 0x78, 0x78, 0x78, 0x78, 0x00, 0x00, 0x00, 0xE0, 0xFF, 0xFF, 0x7F, 0x3F, 0x38, 0x3C, 0x3C, 0x7C, 0xF8, 0xF8, 0xF0, 0xC0, 0x00, 0x00, 0x30, 0xF8, 0xF8, 0xF8, 0xE0, 0x80, 0x80, 0x80, 0xE0, 0xFF, 0xFF, 0xFF, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x07, 0x07, 0x07, 0x07, 0x07, 0x03, 0x01, 0x00, 0x00, 0x00,  //  Code for char 5  
0x00, 0x00, 0x80, 0xE0, 0xF0, 0xF0, 0x78, 0x78, 0x78, 0xF8, 0xF8, 0xF0, 0xE0, 0x80, 0x00, 0x00, 0xFE, 0xFF, 0xFF, 0xFF, 0xF1, 0x78, 0x78, 0x78, 0xF8, 0xF1, 0xF1, 0xE1, 0x81, 0x00, 0x00, 0x1F, 0xFF, 0xFF, 0xFF, 0xC0, 0x80, 0x80, 0x80, 0xC0, 0xFF, 0xFF, 0xFF, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x07, 0x07, 0x07, 0x07, 0x07, 0x03, 0x03, 0x01, 0x00, 0x00,  //  Code for char 6  
0x00, 0x78, 0x78, 0x78, 0x78, 0x78, 0x78, 0x78, 0x78, 0xF8, 0xF8, 0xF8, 0xF8, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xF0, 0xFC, 0xFF, 0x3F, 0x0F, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xFE, 0xFF, 0xFF, 0x1F, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x07, 0x07, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //  Code for char 7  
0x00, 0xC0, 0xE0, 0xF0, 0xF0, 0xF8, 0x78, 0x78, 0x78, 0xF8, 0xF0, 0xF0, 0xE0, 0xC0, 0x00, 0x00, 0x07, 0x9F, 0xDF, 0xFF, 0xF8, 0xF0, 0xF0, 0xF0, 0xF8, 0xFF, 0xFF, 0x9F, 0x07, 0x00, 0x00, 0x7E, 0xFF, 0xFF, 0xFF, 0xC1, 0x80, 0x80, 0x80, 0xC1, 0xFF, 0xFF, 0xFF, 0x7E, 0x00, 0x00, 0x00, 0x01, 0x03, 0x03, 0x07, 0x07, 0x07, 0x07, 0x07, 0x03, 0x03, 0x01, 0x00, 0x00,  //  Code for char 8  
0x00, 0x80, 0xE0, 0xF0, 0xF0, 0xF8, 0x78, 0x78, 0x78, 0xF8, 0xF0, 0xE0, 0xC0, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xFF, 0xE0, 0xC0, 0xC0, 0xC0, 0xE0, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x70, 0xF0, 0xF1, 0xF1, 0xC3, 0x83, 0x83, 0x83, 0xF1, 0xFF, 0xFF, 0x7F, 0x0F, 0x00, 0x00, 0x00, 0x01, 0x03, 0x07, 0x07, 0x07, 0x07, 0x07, 0x03, 0x03, 0x01, 0x00, 0x00, 0x00,  //  Code for char 9  
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x0F, 0x1F, 0x1F, 0x1F, 0x0F, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xE0, 0xF0, 0xF0, 0xF0, 0xE0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x03, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00   //  Code for char :  
};

// GLOBAL CONSTANTS ////////////////////////////////////////////////////////
// Settings array with default values
// Each setting has a comment with: array position, DS1307 memory hex address, default value, lower-upper bounds, description and respective global variable name
const byte defSettings[50] PROGMEM = {
  0x00,                           // 0  0x08    (0)   [0-1]     Temperature measurement unit 0=Celsius 1=Fahrenheit [tempUnit]
  DEFAULT_TIMEZONE+48,            // 1  0x09    (52)  [0-104]   timeZone (-48 +56) [timezone] (-12.00 +14.00) 15 minutes precision. Default is 52 (+1.00 Berlin, Rome, Paris, Madrid, Warsaw, Malta)
  DEFAULT_LATITUDE+90,            // 2  0x0A    (125) [0-180]   latitude (-90 +90) [latitude] starts with Malta la=+35 lo=+14
  (DEFAULT_LONGITUDE+180)%256,    // 3  0x0B    (194) [0-255]   longitude (-180 +180) low byte [longitude]
  (DEFAULT_LONGITUDE+180)/256,    // 4  0x0C    (0)   [0-1]     longitude high byte
  0x03,                           // 5  0x0D    (3)   [0-16]    skip ice: skip watering when temperature is below the limit in celsius (0=OFF -4 +10) (default -2 celsius) [skpIce]
  0x01,                           // 6  0x0E    (1)   [0-3]     temperature/humidity sensor type: 0=NONE  1=ATMEGA328 TMP  2=I2C CATNIP ELEC  3=1WR DS18B20 2BY  4=1WR DS18B20 4BY [sensorType]
  DEFAULT_DAYLIGHT_SAVING,        // 7  0x0F    (1)   [0-1]     daylight saving enabled 0=no 1=yes [daySav] (it must be set accordingly with the country daylight saving rule)
  0x03,                           // 8  0x10    (3)   [0-8]     adjWater 0-8 = 0-400% [adjWater1] default=150%
  0xF0,                           // 9  0x11    (240) [0-255]   real time clock calibration (-240 +240) low byte  [clockCalibration]
  0x00,                           // 10 0x12    (0)   [0-1]     real time clock calibration high byte
  #if DEFAULT_ALARM_STATUS==1
  0x07,                           // 11 0x13    (7)   [0-8]     alarm 1 option(0=off 1=manual 2=dawn 3=dawn-90 4=dawn-60 5=dawn-30 6=dawn+30 7=dawn+60 8=dawn+90) [alm1Option]
  #else
  0x00,                           // 11 0x13    (0)   [0-8]     alarm 1 option(0=off 1=manual 2=dawn 3=dawn-90 4=dawn-60 5=dawn-30 6=dawn+30 7=dawn+60 8=dawn+90) [alm1Option]
  #endif
  0x07,                           // 12 0x14    (7)   [0-23]    alarm 1 hour [alm1DateTime]
  0x00,                           // 13 0x15    (0)   [0-59]    alarm 1 minute [alm1DateTime]
  0x05,                           // 14 0x16    (5)   [0-21]    alarm 1 duration (index of the array almDurValue) [alm1Duration] 
  0x14,                           // 15 0x17    (20)  [0-99]    alarm 1 last trig year [alm1DateTime]
  0x01,                           // 16 0x18    (1)   [0-12]    alarm 1 last trig month [alm1DateTime]
  0x01,                           // 17 0x19    (1)   [0-31]    alarm 1 last trig day [alm1DateTime]
  #if DEFAULT_ALARM_STATUS==1
  0x04,                           // 18 0x1A    (4)   [0-8]     alarm 2 option(0=off 1=manual 2=dusk 3=dusk-90 4=dusk-60 5=dusk-30 6=dusk+30 7=dusk+60 8=dusk+90) [alm2Option]
  #else
  0x00,                           // 18 0x1A    (0)   [0-8]     alarm 2 option(0=off 1=manual 2=dusk 3=dusk-90 4=dusk-60 5=dusk-30 6=dusk+30 7=dusk+60 8=dusk+90) [alm2Option]
  #endif
  0x13,                           // 19 0x1B    (19)  [0-23]    alarm 2 hour [alm2DateTime]
  0x00,                           // 20 0x1C    (0)   [0-59]    alarm 2 minute [alm2DateTime]
  0x02,                           // 21 0x1D    (2)   [0-21]    alarm 2 duration (index of the array almDurValue) [alm2Duration]
  0x14,                           // 22 0x1E    (20)  [0-99]    alarm 2 last trig year [alm2DateTime]
  0x01,                           // 23 0x1F    (1)   [0-12]    alarm 2 last trig month [alm2DateTime]
  0x01,                           // 24 0x20    (1)   [0-31]    alarm 2 last trig day [alm2DateTime]
  0x00,                           // 25 0x21    (0)   [0-6]     alarm 1 days to skip 0=OFF 1=AUTO 2-6=MAN [skpDays1]
  0x5C,                           // 26 0x22    (92)  [0-99]    alarm 1 skip humidity [skpHumidity1] 0=OFF
  0x4A,                           // 27 0x23    (74)  [0-255]   minimum daylight minutes of the year - low byte - Start with Malta 586 [minDaylightMinutes]
  0x02,                           // 28 0x24    (2)   [0-255]   minimum daylight minutes of the year - high byte
  0x68,                           // 29 0x25    (104) [0-255]   maximum daylight minutes of the year - low byte - Start with Malta 872 [maxDaylightMinutes]
  0x03,                           // 30 0x26    (3)   [0-255]   maximum daylight minutes of the year - high byte
  0x00,                           // 31 0x27    (0)   [0-100]   alarm 1 seasonal water adjustment, performed if option Water is ON [adjWaterResult]
  0x01,                           // 32 0x28    (1)   [0-23]    watering now duration [alm3Duration]
  0x07,                           // 33 0x29    (37)  [0-26]    heatwave celsius degrees 0=OFF (30-55)=(1-25)+29 [heatWave]
  0x14,                           // 34 0x2A    (20)  [0-99]    minimum temperature, last check year [minTempDateTime]
  0x01,                           // 35 0x2B    (1)   [0-12]    minimum temperature, last check month [minTempDateTime]
  0x01,                           // 36 0x2C    (1)   [0-31]    minimum temperature, last check day [minTempDateTime]
  0xFA,                           // 37 0x2D    (250) [0-31]    today minimum temperature recorded, zero celsius degrees=125 [-125 +125] [minTempValue]
  0x00,                           // 38 0x2E    (0)   [0-23]    today minimum temperature, hour of recording [minTempDateTime]
  0x00,                           // 39 0x2F    (0)   [0-59]    today minimum temperature, minute of recording [minTempDateTime]
  0x14,                           // 40 0x30    (20)  [0-99]    maximum temperature, last check year [minTempDateTime]
  0x01,                           // 41 0x31    (1)   [0-12]    maximum temperature, last check month [minTempDateTime]
  0x01,                           // 42 0x32    (1)   [0-31]    maximum temperature, last check day [minTempDateTime]
  0x00,                           // 43 0x33    (0)   [0-31]    today maximum temperature recorded, zero degrees=125 [-125 +125] [minTempValue]
  0x00,                           // 44 0x34    (0)   [0-23]    today maximum temperature, hour of recording [minTempDateTime]
  0x00,                           // 45 0x35    (0)   [0-59]    today maximum temperature, minute of recording [minTempDateTime]
  0x00,                           // 46 0x36    (0)   [0-6]     alarm 2 days to skip 0=OFF 1=AUTO 2-6=MAN [skpDays2]
  0x5C,                           // 47 0x37    (92)  [0-99]    alarm 2 skip humidity [skpHumidity2] 0=OFF
  0x03,                           // 48 0x38    (3)   [0-8]     alarm 2 adjWater 0-8 [adjWater2] default=150%
  0x00                            // 49 0x39    (0)   [0-255]   unused
};

// strings saved to flash memory
const char weekDay00[] PROGMEM = "SU";
const char weekDay01[] PROGMEM = "MO";
const char weekDay02[] PROGMEM = "TU";
const char weekDay03[] PROGMEM = "WD";
const char weekDay04[] PROGMEM = "TH";
const char weekDay05[] PROGMEM = "FR";
const char weekDay06[] PROGMEM = "SA";
const char *const weekDays[7] PROGMEM = {weekDay00, weekDay01, weekDay02, weekDay03, weekDay04, weekDay05, weekDay06};

const char month00[] PROGMEM = "JAN";
const char month01[] PROGMEM = "FEB";
const char month02[] PROGMEM = "MAR";
const char month03[] PROGMEM = "APR";
const char month04[] PROGMEM = "MAY";
const char month05[] PROGMEM = "JUN";
const char month06[] PROGMEM = "JUL";
const char month07[] PROGMEM = "AUG";
const char month08[] PROGMEM = "SEP";
const char month09[] PROGMEM = "OCT";
const char month10[] PROGMEM = "NOV";
const char month11[] PROGMEM = "DEC";
const char *const months[12] PROGMEM = {month00, month01, month02, month03, month04, month05, month06, month07, month08, month09, month10, month11};

const char sensorType00[] PROGMEM = "NONE";
const char sensorType01[] PROGMEM = "ATMEGA328";
const char sensorType02[] PROGMEM = "CATNIP";
const char sensorType03[] PROGMEM = "DS18B20";
const char sensorType04[] PROGMEM = "PICAPOT";
const char *const sensorTypes[5] PROGMEM = {sensorType00, sensorType01, sensorType02, sensorType03, sensorType04};

const char optionOff[] PROGMEM = "OFF";  // alarm 1 options
const char optionManual[] PROGMEM = "MANUAL";
const char alm1Opts02[] PROGMEM = "DAWN";
const char alm1Opts03[] PROGMEM = "DAWN+30]Mins";
const char alm1Opts04[] PROGMEM = "DAWN+1]Hour";
const char alm1Opts05[] PROGMEM = "DAWN+2]Hours";
const char alm1Opts06[] PROGMEM = "DAWN+3]Hours";
const char alm1Opts07[] PROGMEM = "DAWN+4]Hours";
const char alm1Opts08[] PROGMEM = "DAWN+5]Hours";
const char *const alm1Opts[9] PROGMEM = {optionOff, optionManual, alm1Opts02, alm1Opts03, alm1Opts04, alm1Opts05, alm1Opts06, alm1Opts07, alm1Opts08};

const char alm2Opts02[] PROGMEM = "DUSK";
const char alm2Opts03[] PROGMEM = "DUSK-30]Mins";
const char alm2Opts04[] PROGMEM = "DUSK-1]Hour";
const char alm2Opts05[] PROGMEM = "DUSK-2]Hours";
const char alm2Opts06[] PROGMEM = "DUSK-3]Hours";
const char alm2Opts07[] PROGMEM = "DUSK-4]Hours";
const char alm2Opts08[] PROGMEM = "DUSK-5]Hours";
const char *const alm2Opts[9] PROGMEM = {optionOff, optionManual, alm2Opts02, alm2Opts03, alm2Opts04, alm2Opts05, alm2Opts06, alm2Opts07, alm2Opts08};

const char almDurOpts00[] PROGMEM = "1]Min"; // alarm duration options
const char almDurOpts01[] PROGMEM = "2]Min";
const char almDurOpts02[] PROGMEM = "3]Min";
const char almDurOpts03[] PROGMEM = "4]Min";
const char almDurOpts04[] PROGMEM = "5]Min";
const char almDurOpts05[] PROGMEM = "6]Min";
const char almDurOpts06[] PROGMEM = "7]Min";
const char almDurOpts07[] PROGMEM = "8]Min";
const char almDurOpts08[] PROGMEM = "9]Min";
const char almDurOpts09[] PROGMEM = "10]Min";
const char almDurOpts10[] PROGMEM = "15]Min";
const char almDurOpts11[] PROGMEM = "20]Min";
const char almDurOpts12[] PROGMEM = "25]Min";
const char almDurOpts13[] PROGMEM = "30]Min";
const char almDurOpts14[] PROGMEM = "40]Min";
const char almDurOpts15[] PROGMEM = "50]Min";
const char almDurOpts16[] PROGMEM = "1]Hour";
const char almDurOpts17[] PROGMEM = "1.5]Hours";
const char almDurOpts18[] PROGMEM = "2]Hours";
const char almDurOpts19[] PROGMEM = "2.5]Hours";
const char almDurOpts20[] PROGMEM = "3]Hours";
const char *const almDurOpts[21] PROGMEM = {almDurOpts00, almDurOpts01, almDurOpts02, almDurOpts03, almDurOpts04, almDurOpts05, almDurOpts06, almDurOpts07, almDurOpts08, almDurOpts09, almDurOpts10, almDurOpts11, almDurOpts12, almDurOpts13, almDurOpts14, almDurOpts15, almDurOpts16, almDurOpts17, almDurOpts18, almDurOpts19, almDurOpts20};

// each battery level picture is formed by two characters
const char batteryChargePict00[] PROGMEM = "`'";
const char batteryChargePict01[] PROGMEM = "_'";
const char batteryChargePict02[] PROGMEM = ";'";
const char batteryChargePict03[] PROGMEM = "\"'";
const char batteryChargePict04[] PROGMEM = "\"#";
const char *const batteryChargePict[5] PROGMEM = {batteryChargePict00, batteryChargePict01, batteryChargePict02, batteryChargePict03, batteryChargePict04};


// GLOBAL VARIABLES ////////////////////////////////////////////////////////
struct dateTime {
  uint8_t second = 0;
  uint8_t minute = 0;
  uint8_t hour = 0;
  uint8_t dow = 4;
  uint8_t day = 1;
  uint8_t month = 1;
  uint8_t year = 20;
};
dateTime alm1DateTime, alm2DateTime; // hour/minute parts of the structure contain the alarm scheduling time, day/month/year parts contain the date of the last triggered alarm
dateTime dtm; // datetime of the clock, it is refreshed every interrupt
dateTime dtm_ds; // datetime of the clock including daylight saving
byte alm1Option, alm1Duration, alm2Option, alm2Duration, alm3Duration, daySav, skpIce, skpHumidity1, skpDays1, skpHumidity2, skpDays2, heatWave, batteryLevel, tempUnit, sensorType; //options variables
int timezone, latitude, longitude, minDaylightMinutes, maxDaylightMinutes, adjWater1, adjWaterResult, adjWater2, clockCalibration; //options variables
int temperatureSensorValue, temperatureSensorValueRaw, InternalVoltage;
unsigned int soilSensorValue, soilSensorValueRaw, soilSensorMin=0, soilSensorMax=65535;
bool soilSensorInverted=0;
//The user interface of Sunfy328 is organized in 8 screens.
const byte blinkPosCt[8] = {7, 4, 7, 7, 5, 1, 0, 0}; // total input cells of every screen. screen temperature and screen about have no edit mode and are set to zero.
dateTime minTempDateTime, maxTempDateTime; // hour/minute parts of the structure containing the recording time of the min/max temperature of the day, day/month/year parts containing the date of the last temperature check
int minTempValue, maxTempValue;
byte calibration[30] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // clock calibration 240 bit array
const int8_t alarmDDAdjust[9] = {0, 0, 0, 3, 6, 12, 18, 24, 30}; // values in minutes/10 of the alarm options (dusk and dawn adjustment)
const byte almDurValue[21] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 15, 20, 25, 30, 40, 50, 60, 90, 120, 150, 180}; // values in minutes of the alarm durations
unsigned int soilMoistureSample[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // the soil moisture sensor value is an average of the last 10 samples
int temperatureSample[10] = {25, 25, 25, 25, 25, 25, 25, 25, 25, 25}; // the temperature value is an average of the last 10 samples
bool blinkStatus = false; // this value is switched every BLINK_FREQ
bool manualStop = false; // User can stop watering pressing the button 1 for more than 2 seconds in the main screen
bool editMode = false; // start in view mode
bool btn1Keep = false, btn2Keep = false, btn3Keep = false; // become true when a button is kept pressed for more than 2 seconds
bool screenSleep = false, refreshNow = false; //sleep mode and refresh flag of the screen
byte AlmTrig = 0; // contains the currently firing alarm. 0=none  1=alarm-1  2=alarm-2  3=manual
bool btn1Status = true, btn2Status = true, btn3Status = true; // pressing status of the buttons
bool lastBtn1Status = false, lastBtn2Status = false, lastBtn3Status = false; // button debounce status
bool skipButtonProcess = false; // flag to skip process of the button that woke up from sleep mode
byte sensorErr=3; // count of consecutive communication errors with the sensor. Status is error until the first good sample.
byte btn1 = 1, btn2 = 1, btn3 = 1; // processing status of the buttons: 0=pressed  1=not pressed  2=press process completed
byte pressCt = 0, sleepCt = 0;
byte activeScreen = 1, blinkPos = 0; // current screen, current input cell of a screen in edit mode
unsigned long ms = 0, lastAlm = 0, lastBlink = 0; // used to keep track of the elapsed time in milliseconds
unsigned long lastPin1Debounce = 0, lastPin2Debounce = 0, lastPin3Debounce = 0; // button debounce timing
char cstr1[12]; // used to convert long type numbers when passed to the function Print. If more values are passed to the same function, each value must use a different cstr array
char cstr2[12]; // a long can have maximum 10 digits (2147483647), plus 1 digit sign if present, plus 1 digit for the string terminator = 12 digits total
char cstr3[12];
volatile byte intRTC = 0;
volatile byte intBTN = 0;
byte temperatureSensorAddress[8];
byte tmp[51]; // used to upload/download/convert settings

// GLOBAL CLASSES ////////////////////////////////////////////////////////
#if SCREEN_INTERFACE
  SSD1306AsciiSpi Screen;   // create an instance of the SPI screen class
#else
  SSD1306AsciiAvrI2c Screen; // create an instance of the I2C screen class
#endif
OneWire oneWire(PIN_ONE_WIRE); // initialize the OneWire protocol used to communicate with the sensor
// End of declarations -----------------------------------------



void setup() {
  // Set up the watchdog to reset the system after 8 seconds without a response
  wdt_reset();                      // restart reset counter to avoid a premature reset while entering the registers (if previous wtd was set to 16ms)
  cli();                            // disable interrupts for changing the registers
  MCUSR = 0;                        // reset status register flags
  WDTCSR |= 0b00011000;             //  Set WDCE (5th from left) and WDE (4th from left) to enter config mode, using bitwise OR assignment leaving other bits unchanged.
  WDTCSR =  0b00001000 | 0b100001;  //  combination interrupt + watchdog:0b01001000 only watchdog:0b00001000 | duration  (16 ms:0b000000   500 ms:0b000101  1 second:0b000110   2 seconds:0b000111  4 seconds:0b100000  8 seconds:0b100001)
  sei();                            // re-enable interrupts

  TWBR = 72; // set I2C speed to 100Khz
  delay(10);
  pinMode(PIN_LED, OUTPUT); // prepare the red led pin
  Blink(150, 333); //blink the led for the first time to communicate that the microcontroller is in good working order and it has started the setup process
  #if SCREEN_INTERFACE
    Screen.begin(SCREEN_TYPE, PIN_SPI_CS, PIN_SPI_DC, PIN_SPI_RST); // intialize the SPI screen class
  #else
    Screen.begin(SCREEN_TYPE, SCREEN_ADDRESS); // intialize the I2C screen class
  #endif
  Screen.setContrast(SCREEN_CONTRAST); 
  Screen.clear();
  showMsg(0, F("BOOT")); // print at screen using a formatting function
  delay(333);
  pinMode(PIN_INT0, INPUT_PULLUP); // receives the interrupt from the clock square wave 1Hz. The system goes to sleep mode after processing a loop(), and it is woke up by this signal every second
  pinMode(PIN_BTN1, INPUT_PULLUP); // The button 1 is the setup button. In sleep mode it has an interrupt attached to wake up
  pinMode(PIN_BTN2, INPUT_PULLUP); // The button 2 is the first navigation button. In sleep mode it has an interrupt attached to wake up
  pinMode(PIN_BTN3, INPUT_PULLUP); // The button 3 is the second navigation button. In sleep mode it has an interrupt attached to wake up
  pinMode(PIN_OUT, OUTPUT); // This pin is used to send the signal to the MOSFET that switches on the normally closed (NC) solenoid valve
  digitalWrite(PIN_OUT, LOW); // start with the valve off (closed)
  pinMode(PIN_BAT_CHECK, INPUT); //this pin is used to sense the battery voltage
  delay(10);
  // Before continuing check settings validity using the CRC algorithm
  // If the CRC is wrong, the user must decide if writing the default settings, or rebooting the system. DEFAULT_ALARM_STATUS=1 writes the default settings without asking
  // CRC can be wrong for one of the following reasons:
  // 1) settings has never been written yet to the DS1307 memory (first run)
  // 2) the DS1307 backup battery is flat (or missing) and the system is rebooting after a black-out
  // 3) a catastrophic failure of the DS1307 memory
  int pressedBtn = -1;
  DownloadSettings(tmp);
  while (oneWire.crc8(tmp, 50) != tmp[50] && pressedBtn == -1) {
    Screen.clear();
    showMsg(2, F("MEM ERR"));
    showMsg(4, F("Reset?"));
    showMsg(6, F("BT1=Y BT3=N"));
    while (pressedBtn == -1) {
      if (digitalRead(PIN_BTN1) == DEFAULT_ALARM_STATUS) {
        pressedBtn = 1;
        Screen.clear();
        showMsg(0, F("Reset"));
      }
      if (digitalRead(PIN_BTN3) == 0) {
        pressedBtn = 3;
        Screen.clear();
        showMsg(0, F("Restart"));
      }
      delay(SYSTEM_DELAY);
      wdt_reset(); // extend the watchdog reset
    }
  }
  if (pressedBtn == 3) { // reboot requested by the user
    while (1) {}; // block the execution and wait for watchdog reset
  }
  if (pressedBtn == 1) { //reset requested by the user. write the default settings
    memcpy_P(tmp, defSettings, 50);
    UploadSettings(tmp);
    SetDateTime(dtm);
    while (1) {}; // block the execution and wait for watchdog reset. settings will be checked again after reboot
  }
  wdt_reset();
  Settings2Variables(tmp); // convert the settings array to global variables
  showMsg(2, F("MEM OK"));
  delay(333);

  CreateCalibrationMatrix(); // Initialize the array for the Clock calibration functionality
  InternalVoltage = readVcc();
  ActivateClockOscillator(); //activate the clock oscillator (if it is not yet active)
  //activate the clock square wave 1Hz used as an interrupt to wake up from sleep mode and as a timer for refreshing the screen 
  Wire.beginTransmission(CLOCK_ADDRESS);
  Wire.write((uint8_t)0x07);
  Wire.write((uint8_t)dec2bcd(10));
  if (Wire.endTransmission() != 0) {
    digitalWrite(PIN_LED, HIGH);
    showMsg(4, F("CLK ERR"));
    while (1) {};
  } else {
    showMsg(4, F("CLK OK"));
    delay(333);
  }
  wdt_reset();
  batteryLevel = getBatteryLevel();
 
  //internal atmega328 temperature
  if (sensorType==1)
  {
      showMsg(6, F("SNS OK"));
      delay(333);
  }
  //I2C sensor
  if (sensorType==2)
  {
    Wire.begin();
    Wire.beginTransmission(0x20);
    Wire.write(6);    
    if (Wire.endTransmission() != 0) {
      showMsg(6, F("SNS ERR"));
      delay(2000);
    }
    else
    {
      showMsg(6, F("SNS OK"));
      delay(333);
    }
  }
  //1-wire sensor
  if (sensorType==3 || sensorType==4)
  {
    oneWire.target_search(0x28);
    if ( !oneWire.search(temperatureSensorAddress)) {
      showMsg(6, F("SNS ERR"));
      delay(2000);
    }
    else {
      showMsg(6, F("SNS OK"));
      delay(333);
    }
  }
  
  wdt_reset();
  Blink(150, 333);
  Blink(150, 333);
  Blink(150, 333);
  Screen.clear();
  wdt_reset();
  attachInterrupt(INT0, ClockInterrupt, FALLING); // sets the 1Hz square wave of the clock DS1307 as an external interrupt that trigs the ClockInterrupt function.
}
// End of setup
// =========================================================================================================================================


void ClockInterrupt() { // 1Hz square wave signal from the clock
  intRTC++;
}


ISR (PCINT2_vect) { // buttons pin status change
  intBTN++;
}

ISR(WDT_vect) {
}

ISR (PCINT0_vect) {
}

void loop()
{
  long i = 0;
    
  wdt_reset(); // restart the 8 seconds counter of the watchdog
  byte _intBTN = intBTN; // copy volatile flags to local variables
  byte _intRTC = intRTC;
  if (_intBTN > 0) { // clear volatile flag
    intBTN = 0;
  }
  if (_intRTC > 0) { // clear volatile flag
    intRTC = 0;
  }
  if (_intBTN > 0) { // if a button is pressed, wake up from screen sleep mode,
    sleepCt = 0;
    if (screenSleep == true) { // only if the system is actually sleeping
      SwitchOnScreen();
      skipButtonProcess = true;
    }
  }
  if ((_intRTC > 0) || (_intBTN > 0) || (editMode == true)) { //one of the two interrupts has been triggered, or edit mode is ON
    dtm = GetDateTime(); // DS1307 keeps datetime without daylight saving...
    if (IsSummerTime(dtm))
    {
      dtm_ds = Add1Hour(dtm); //... and it is added when printed at screen
    } else {
      dtm_ds = dtm;
    }
  }

  if ((_intRTC > 0) || (_intBTN > 0)) { // one of the two interrupts has been triggered
    if (_intRTC > 0) { // the interrupt comes from the 1Hz square wave
      Blink(1, 0); // blink the led for 1 millisecond
      //ActivateClockOscillator(); // this is done every second because there is a (very rare) chance that the clock oscillator stops oscillating while the square wave remains in good working order.

      if ((dtm.second%2)==0) //every two seconds check the sensor
      {
        if (sensorType==1) //Internal ATMega328 temp sensor
        {
          temperatureSensorValueRaw = readInternalTemp(temperatureSensorValue);
        }    
        
        if (sensorType==2) //I2C sensor by Catnip Ele
        {
          soilSensorValueRaw=readI2CSensorRegister(CATNIP_SENSOR_ADDRESS, 0);
          temperatureSensorValueRaw=readI2CSensorRegister(CATNIP_SENSOR_ADDRESS, 5)/10;
          soilSensorMin=100;
          soilSensorMax=600;
        }
        
        if (sensorType==3 || sensorType==4) // 1-wire sensors based on DS18B20
        {
          getTemperatureAndHumidity_DS18B20();
        }        
      }

      //average of the last 10 temperature and humidity samples
      if (sensorErr==0)
      {
        for (byte x = 0; x < 9; x++ ) {
          temperatureSample[x] = temperatureSample[x + 1];
        }
        temperatureSample[9] = temperatureSensorValueRaw;
        
        i = soilSensorValueRaw;
        if (i < soilSensorMin)
        {
          i = soilSensorMin;
        }
        i = i - soilSensorMin;
        if (i > (soilSensorMax - soilSensorMin))
        {
          i = soilSensorMax - soilSensorMin;
        }
        i = int(i * 100 / (soilSensorMax - soilSensorMin)); //humidity from 0=min to 100=max
        if (soilSensorInverted) //humidity from 0=max to 100=min
        {
          i = 100 - i;
        }
        for (byte x = 0; x < 9; x++ ) {
          soilMoistureSample[x] = soilMoistureSample[x + 1];
        }
        soilMoistureSample[9] = i;

        i = 0;
        for (byte x = 0; x < 10; x++ ) {
          i = i + temperatureSample[x];
        }
        temperatureSensorValue = i / 10; 
        i = 0;
        for (byte x = 0; x < 10; x++ ) {
          i = i + soilMoistureSample[x];
        }
        soilSensorValue = i / 10;        
      }

  
      //record the minimum and maximum temperature of the day
      if (dateDiff(minTempDateTime, dtm_ds) == 0) {
        if (temperatureSensorValue < minTempValue) {
          minTempDateTime.hour = dtm_ds.hour;
          minTempDateTime.minute = dtm_ds.minute;
          minTempValue = temperatureSensorValue;
        }
      } else {
        minTempDateTime = dtm_ds;
        minTempValue = temperatureSensorValue;
      }
      if (dateDiff(maxTempDateTime, dtm_ds) == 0) {
        if (temperatureSensorValue > maxTempValue) {
          maxTempDateTime.hour = dtm_ds.hour;
          maxTempDateTime.minute = dtm_ds.minute;
          maxTempValue = temperatureSensorValue;
        }
      } else {
        maxTempDateTime = dtm_ds;
        maxTempValue = temperatureSensorValue;
      }

      //  every six minutes apply clock calibration using the clock calibration array (more info below).
      if (((dtm.hour * 60 + dtm.minute) % 6) == 0 && dtm.second == 20) { // apply the calibration after a few seconds to avoid concurrency with alarms
        if ((ReadCalibrationBit((dtm.hour * 60 + dtm.minute) / 6) == true) && (clockCalibration != 0)) {
          if (screenSleep == true) {
            SwitchOnScreen();
            delay(SYSTEM_DELAY);
          }
          sleepCt = SCREEN_SLEEP_SEC - 8; //switch ON the screen for 8 seconds during calibration
          Screen.clear();
          if (clockCalibration < 0) {
            dtm.second = 19;
          } else {
            dtm.second = 21;
          }
          SetDateTime(dtm);
          if (clockCalibration < 0) {
            showMsg(0, F("calib-1"));
          } else {
            showMsg(0, F("calib+1"));
          }
          wdt_reset();
          delay(3000);
        }
        wdt_reset();
        // every six minutes save the day min/max temperature to the clock memory
        SaveSettings();
        // refresh battery level
        InternalVoltage = readVcc();
        batteryLevel = getBatteryLevel();
      } // end of clock calibration
    } // end of 1Hz square wave processing

    if (AlmTrig == 0 && screenSleep == false) { // if no alarm is triggered, increase the counter for the sleep mode
      sleepCt++;
    }
    CheckAlarms(); // check if there is an alarm to switch ON
    SwitchOffAlarms(); // check if there is an alarm to switch OFF
    if (editMode == false) { // screen is refreshed every interrupt if not in edit mode
      refreshNow = true;
    }
  } // end of interrupts processing
  
  if ((sleepCt > SCREEN_SLEEP_SEC) && (SCREEN_SLEEP_SEC != 0)) { // Switch off the screen after SCREEN_SLEEP_SEC seconds of inactivity
    screenSleep = true;
    sleepCt = 0;
    editMode = false;
    blinkPos = 0;
    pressCt = 0;
    activeScreen = 1;
    Screen.clear();
    Screen.ssd1306WriteCmd(SSD1306_DISPLAYOFF);
    Screen.ssd1306WriteCmd(SH1106_SET_PUMP_MODE);
    Screen.ssd1306WriteCmd(SH1106_PUMP_OFF);
  }
  if (screenSleep == false) { // buttons are monitored every loop (don't process buttons during sleep mode and don't process buttons that woke up from sleep mode)
    MonitorButtons();
    if ((millis() - ms) > EDIT_FREQ) { // buttons are processed only every 70 ms (to have a proper input increase/decrease speed, when they are kept pressed)
      ms = millis();
      ProcessButtons();
      if ((editMode == true) || (AlmTrig != 0))  { // screen is refreshed only when necessary
        refreshNow = true;
      }
    }
    if ((millis() - lastBlink) > BLINK_FREQ) { // This to have the blink effect in edit mode (200 ms)
      blinkStatus = !blinkStatus;
      lastBlink = millis();
      if ((editMode == true) || (AlmTrig != 0)) { // screen is refreshed only when necessary
        refreshNow = true;
      }
    }
  }
  if (screenSleep == false)
  {
    if (refreshNow == true) { // screen is not refreshed every loop but only when necessary
      PrintScreen(activeScreen);
      refreshNow = false;
    }
  }

  if (screenSleep == true) { // unless the screen is active, go to sleep mode, till the next 1Hz (1 second) square wave signal
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    noInterrupts ();
    power_all_disable();
    sleep_bod_disable();
    interrupts ();
    //enable pin change interrupts for the three buttons
    PCICR  |= bit (PCIE2);
    PCMSK2 |= bit (PCINT17);
    PCMSK2 |= bit (PCINT19);
    PCMSK2 |= bit (PCINT20);
    PCIFR  |= bit (PCIF2);
    sleep_cpu();

    sleep_disable();              //wake up here
    power_all_enable();
    //disable pin change interrupts for the three buttons
    PCICR  &= bit (PCIE2);
    PCMSK2 &= bit (PCINT17);
    PCMSK2 &= bit (PCINT19);
    PCMSK2 &= bit (PCINT20); 
    
  }

  delay(2);
} // End of loop
// ========================================================================================



//   <!------------------------------------------------->
//   <!--    Clock Calibration by Fabrizio Ranieri    -->
//   <!--    GNU General Public License applies       -->
//   <!------------------------------------------------->
// Clock Calibration is based on a 240 bit array. Each bit represents a period of 6 minutes (6x240 = 1440 = 24 hours) and every 6 minutes the corresponding bit is checked.
// When the bit is high represents 1 second of error to be adjusted accordingly with the sign of the setting [clockCalibration](-240 +240) (-=clock moves behind +=clock moves ahead).
// The array is created from the absolute value contained in the two bytes of the setting [clockCalibration], equally distributing the time corrections throughout the day. 
// The clock can be calibrated for an error of maximum 240 seconds in 24 hours.
// Only [clockCalibration] value (2 bytes) is stored to the battery backed memory, and the array (30 bytes) must be created at every reboot or [clockCalibration] setting change.
void CreateCalibrationMatrix() {
  int clockCal, ct = 0;
  clockCal = abs(clockCalibration);
  bool standardProcess = (clockCal <= 120); // if high bit are the majority, invert the distribution logic for a better result
  int timeFrame = standardProcess ? (240 / (clockCal == 0 ? 1 : clockCal)) : (240 / ((240 - clockCal) == 0 ? 1 : (240 - clockCal)));
  for (byte i = 0; i < 30; i++)
  {
    calibration[i] = standardProcess ? (byte)0x00 : (byte)0xFF;
  }
  for (byte i = 0; i < 240; i++)
  {
    if (ct < (standardProcess ? clockCal : (240 - clockCal)))
    {
      if ((i % timeFrame) == 0)
      {
        ct = ct + 1;
        WriteCalibrationBit(i, standardProcess);
      }
    }
  }
}

bool ReadCalibrationBit(byte pos) {
  byte ps = pos / 8;
  return (calibration[ps] &  (1 << (7 - (pos % 8)))) == (1 << (7 - (pos % 8)));
}

void WriteCalibrationBit(byte pos, bool val) {
  byte ps = pos / 8;
  calibration[ps] =  val ? calibration[ps] | (1 << (7 - (pos % 8))) : calibration[ps] & (255 - (1 << (7 - (pos % 8)))) ;
}

String Byte2BinString(byte b) { // used during development to monitor the calibration array
  String s = "";
  for (int i = 7; i >= 0; i-- )
  {
    s = s + String((b >> i) & 0X01); //shift and select first bit
  }
  return s;
}
// end of clock calibration



int readVcc() {
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA, ADSC)); //first sample is discarded
  ADCSRA |= _BV(ADSC); 
  while (bit_is_set(ADCSRA,ADSC));  
  return 1126400L / ADC;  
}

int getBatteryMillivolts() {
  analogReference(DEFAULT);
  delay(2);
  unsigned long f = analogRead(PIN_BAT_CHECK); //first sample is discarded
  delay(2);
  f = analogRead(PIN_BAT_CHECK);
  f = f * InternalVoltage;
  return int(f / 1024);
}

int getBatteryLevel() {
  int i, res;
  i = getBatteryMillivolts();
  if (i < 2200) {
    res = 0;
  } else if (i < 2400) {
    res = 1;
  } else if (i < 2600) {
    res = 2;
  } else if (i < 2900) {
    res = 3;
  } else {
    res = 4;
  }
  return res;
}

// read settings array from the DS1307 memory
void DownloadSettings(byte* data) {
  byte x = 0;
  Wire.beginTransmission(CLOCK_ADDRESS);
  Wire.write((uint8_t)0x08);
  if (Wire.endTransmission() != 0) {
    digitalWrite(PIN_LED, HIGH);
    showMsg(0, F("CLKER-R"));
    while (1) {};
  }
  else
  {
    Wire.requestFrom(CLOCK_ADDRESS, 32);
    while (Wire.available()) {
      data[x] = Wire.read();
      x++;
    }
  }
  Wire.beginTransmission(CLOCK_ADDRESS);
  Wire.write((uint8_t)0x28);
  if (Wire.endTransmission() != 0) {
    digitalWrite(PIN_LED, HIGH);
    showMsg(0, F("CLKER-R"));
    while (1) {};
  }
  else
  {
    Wire.requestFrom(CLOCK_ADDRESS, 19);
    while (Wire.available()) {
      data[x] = Wire.read();
      x++;
    }
  }
}

// convert local variables to settings array, save it to the DS1307 memory and verify it
void SaveSettings() {
  byte z = 0;
  while (z < 3) {
    Variables2Settings(tmp);
    UploadSettings(tmp);
    DownloadSettings(tmp);
    if (oneWire.crc8(tmp, 50) == tmp[50]) {
      z = 4;
    }
    else {
      z++;
      delay(20);
    }
  }
  if (z == 3) { // if the settings cannot be written to the DS1307 memory, after three attempts reset the system
    showMsg(0, F("CLKER-W"));
    while (1) {};
  }
}

// write settings array to the DS1307 memory
void UploadSettings(byte* data) {
  Wire.beginTransmission(CLOCK_ADDRESS);
  Wire.write((uint8_t)0x08);
  for (byte x = 0; x < 31; x++ ) {
    Wire.write((uint8_t)data[x]);
  }
  if (Wire.endTransmission() != 0) {
    digitalWrite(PIN_LED, HIGH);
    showMsg(0, F("CLKER-W"));
    while (1) {};
  }
  else
  {
    Wire.beginTransmission(CLOCK_ADDRESS);
    Wire.write((uint8_t)0x27);
    for (byte x = 31; x < 50; x++ ) {
      Wire.write((uint8_t)data[x]);
    }
    Wire.write((uint8_t)oneWire.crc8(data, 50));
  }
  if (Wire.endTransmission() != 0) {
    digitalWrite(PIN_LED, HIGH);
    showMsg(0, F("CLKER-W"));
    while (1) {};
  }
}

void Settings2Variables(byte* data) { // convert the settings from an array of bytes to global variables
  tempUnit = data[0];
  timezone = data[1] - 48;
  latitude = data[2] - 90;
  longitude = (data[3] + data[4] * 256) - 180;
  skpIce = data[5];
  sensorType = data[6];
  daySav = data[7];
  adjWater1 = data[8];
  clockCalibration = (data[9] + data[10] * 256) - 240;
  alm1Option = data[11];
  alm1DateTime.hour = data[12];
  alm1DateTime.minute = data[13];
  alm1Duration = data[14];
  alm1DateTime.year = data[15];
  alm1DateTime.month = data[16];
  alm1DateTime.day = data[17];
  alm2Option = data[18];
  alm2DateTime.hour = data[19];
  alm2DateTime.minute = data[20];
  alm2Duration = data[21];
  alm2DateTime.year = data[22];
  alm2DateTime.month = data[23];
  alm2DateTime.day = data[24];
  skpDays1 = data[25];
  skpHumidity1 = data[26];
  minDaylightMinutes = data[27] + (data[28] * 256);
  maxDaylightMinutes = data[29] + (data[30] * 256);
  adjWaterResult = data[31];
  alm3Duration = data[32];
  heatWave = data[33];
  minTempDateTime.year = data[34];
  minTempDateTime.month = data[35];
  minTempDateTime.day = data[36];
  minTempValue = data[37] - 125;
  minTempDateTime.hour = data[38];
  minTempDateTime.minute = data[39];
  maxTempDateTime.year = data[40];
  maxTempDateTime.month = data[41];
  maxTempDateTime.day = data[42];
  maxTempValue = data[43] - 125;
  maxTempDateTime.hour = data[44];
  maxTempDateTime.minute = data[45];
  skpDays2 = data[46];
  skpHumidity2 = data[47];
  adjWater2 = data[48];
}

void Variables2Settings(byte* data) { // convert the settings from global variables to an array of bytes
  data[0] = tempUnit;
  data[1] = timezone + 48;
  data[2] = latitude + 90;
  data[3] = (longitude + 180) % 256;
  data[4] = (longitude + 180) / 256;
  data[5] = skpIce;
  data[6] = sensorType; 
  data[7] = daySav;
  data[8] = adjWater1;
  data[9] = (clockCalibration + 240) % 256;
  data[10] = (clockCalibration + 240) / 256;
  data[11] = alm1Option;
  data[12] = alm1DateTime.hour;
  data[13] = alm1DateTime.minute;
  data[14] = alm1Duration;
  data[15] = alm1DateTime.year;
  data[16] = alm1DateTime.month;
  data[17] = alm1DateTime.day;
  data[18] = alm2Option;
  data[19] = alm2DateTime.hour;
  data[20] = alm2DateTime.minute;
  data[21] = alm2Duration;
  data[22] = alm2DateTime.year;
  data[23] = alm2DateTime.month;
  data[24] = alm2DateTime.day;
  data[25] = skpDays1;
  data[26] = skpHumidity1;
  data[27] = minDaylightMinutes % 256;
  data[28] = minDaylightMinutes / 256;
  data[29] = maxDaylightMinutes % 256;
  data[30] = maxDaylightMinutes / 256;
  data[31] = adjWaterResult;
  data[32] = alm3Duration;
  data[33] = heatWave;
  data[34] = minTempDateTime.year;
  data[35] = minTempDateTime.month;
  data[36] = minTempDateTime.day;
  data[37] = minTempValue + 125;
  data[38] = minTempDateTime.hour;
  data[39] = minTempDateTime.minute;
  data[40] = maxTempDateTime.year;
  data[41] = maxTempDateTime.month;
  data[42] = maxTempDateTime.day;
  data[43] = maxTempValue + 125;
  data[44] = maxTempDateTime.hour;
  data[45] = maxTempDateTime.minute;
  data[46] = skpDays2;
  data[47] = skpHumidity2;
  data[48] = adjWater2;
}



long SecondDiff(dateTime tim1, dateTime tim2) {
  long r = tim1.hour;
  r = r * 3600;
  r = r + (tim1.minute * 60) + tim1.second;
  long r2 = tim2.hour;
  r2 = r2 * 3600;
  r2 = r2 + (tim2.minute * 60) + tim2.second;
  return r2 - r;
}


//check and trig alarms
void CheckAlarms() { 
  byte almNow = 0;
  bool skipAlarm = false;
  int daysToSkip = 0;
  int daysSinceLastAlarm;
  if (AlmTrig == 1)
  {
    daysSinceLastAlarm = dateDiff(alm1DateTime, dtm_ds);
  }
  else
  {
    daysSinceLastAlarm = dateDiff(alm2DateTime, dtm_ds);
  }
  
  if (daysSinceLastAlarm>2) //if the system restars after more than 2 days of inactivity, and alarm is in dusk/dawn mode, adjust the alarm time
  {
    AdjNextAlarmAndWater();
  }
  
  if (alm1Option != 0) { // check alarm 1
    if (SecondDiff(alm1DateTime, dtm_ds) == 0 || SecondDiff(alm1DateTime, dtm_ds) == 1) { // use a timeframe of two seconds to check if the alarm matches with the time
      almNow = 1;
    }
    if (SecondDiff(dtm_ds, alm1DateTime) == 10 && screenSleep == true) { // switch on the screen 10 seconds before the alarm
      SwitchOnScreen();
    }
  }

  if (alm2Option != 0) { // check alarm 2
    if (SecondDiff(alm2DateTime, dtm_ds) == 0 || SecondDiff(alm2DateTime, dtm_ds) == 1) { // use a timeframe of two seconds to check if the alarm matches with the time
      almNow = 2;
    }
    if (SecondDiff(dtm_ds, alm2DateTime) == 10 && screenSleep == true) { // switch on the screen 10 seconds before the alarm is triggered
      SwitchOnScreen();
    }
  }

  if ((!editMode) && AlmTrig == 0 && ((almNow == 1) || (almNow == 2))) { // one of the alarms is triggered
    AlmTrig = almNow;
    if (screenSleep == true) {
      SwitchOnScreen();
    }

    if ((AlmTrig == 1 ? skpDays1 : skpDays2) == 1) // skipdays is in auto mode
    {
      // adjWaterResult contains a value indicating from 1 to 100 in which season of the year we are (calculated using the length of the day)
      bool bothAlarmsOn=((AlmTrig == 1) && (alm1Option != 0) && (alm2Option != 0) && (skpDays1 == 1) && (skpDays2 == 1));
      if (adjWaterResult < 25) // in winter we water one day out of three
      {
        daysToSkip = 3; // skip 2 days
        if (bothAlarmsOn) { //if both alarms are ON, use only alarm-2
          skipAlarm = true;
        }
      }
      else if (adjWaterResult < 45) // in early spring and late autumn we water one day yes and one day no
      {
        daysToSkip = 2; // skip 1 day
        if (bothAlarmsOn) { //if both alarms are ON, use only alarm-2
          skipAlarm = true;
        }
      }
      else if (adjWaterResult < 70) // in late spring and early autumn we water every day, but only once a day
      {
        daysToSkip = 0;
        if (bothAlarmsOn) { //if both alarms are ON, use only alarm-2
          skipAlarm = true;
        }
      }
      else // in summer we water every day twice a day
      {
        daysToSkip = 0;
      }
    }
    else // skipdays value has been specified by the user
    {
      daysToSkip = (AlmTrig == 1 ? skpDays1 : skpDays2);
    }

    

    // If the alarm is triggered with a wrong date that was set to the future, and then the date is set back correctly, the next alarm will not be triggered until the first wrong date is reached (if skipdays is ON)
    // To fix this eventual issue, we set the last triggered alarm date to the current date.
    if (daysSinceLastAlarm < 0)
    {
      if (AlmTrig == 1)
      {
        alm1DateTime.day = dtm_ds.day;
        alm1DateTime.month = dtm_ds.month;
        alm1DateTime.year = dtm_ds.year;
      }
      else
      {
        alm2DateTime.day = dtm_ds.day;
        alm2DateTime.month = dtm_ds.month;
        alm2DateTime.year = dtm_ds.year;
      }

      SaveSettings();
      daysSinceLastAlarm = 0;
    } // end of future date correction

    //when alarm is in auto-mode, it cannot water more than once a day.
    //this could happen if the watering is in dusk/dawn mode with a very short period, and the system, restarting after 2-4 days of inactivity, gives water earlier
    if (daysSinceLastAlarm==0 && (AlmTrig == 1 ? alm1Option : alm2Option) > 1) 
    {
      skipAlarm = true; 
    }


    if (skipAlarm == false) { 
      if (((AlmTrig == 1 ? skpDays1 : skpDays2) == 0) || (daysToSkip <= daysSinceLastAlarm)) // skip days is off or skip days condition is true
      {
        if  (((AlmTrig == 1) && (skpHumidity1 == 0 || sensorErr > 1 || sensorType==0 || soilSensorValue < skpHumidity1)) 
              || ((AlmTrig == 2) && (skpHumidity2 == 0 || sensorErr > 1 || sensorType==0 || soilSensorValue < skpHumidity2))) // skip humidity
        {
          if ((skpIce == 0) || (temperatureSensorValue >= (skpIce - 5))) // skip ice
          {
            // alarm triggered!
            lastAlm = millis(); // this global variable is used to keep track of the elapsed seconds  
            blinkPos = 0; // reset blinking cell position
            editMode = false; // start in view mode
            activeScreen = 1; // show default screen
            //save last alarm trig date
            if (AlmTrig == 1)
            {
              alm1DateTime.day = dtm_ds.day;
              alm1DateTime.month = dtm_ds.month;
              alm1DateTime.year = dtm_ds.year;

            }
            else
            {
              alm2DateTime.day = dtm_ds.day;
              alm2DateTime.month = dtm_ds.month;
              alm2DateTime.year = dtm_ds.year;
            }
            SaveSettings();
            digitalWrite(PIN_OUT, HIGH); // WATERING!
          }
          else
          {
            showMsg(0, F("SKIP ICE"));
            delay(3000);
            AlmTrig = 0;
            sleepCt = 0;
            manualStop = false;
            AdjNextAlarmAndWater();
            SaveSettings();
          }
        }
        else
        {
          showMsg(0, F("SKIP HUMID"));
          delay(3000);
          AlmTrig = 0;
          sleepCt = 0;
          manualStop = false;
          AdjNextAlarmAndWater();
          SaveSettings();
        }
      }
      else {
        Print(0, 0, false, F("SKIPDAY&/&^"), lngToChar(cstr1, daysSinceLastAlarm, false), lngToChar(cstr2, daysToSkip - 1, false), "", "", "");
        delay(3000);
        AlmTrig = 0;
        sleepCt = 0;
        manualStop = false;
        AdjNextAlarmAndWater();
        SaveSettings();
      }
    }
    else {
      showMsg(0, F("SKIP ALARM"));
      delay(3000);
      AlmTrig = 0;
      sleepCt = 0;
      manualStop = false;
      AdjNextAlarmAndWater();
      SaveSettings();
    }
  }
}


unsigned long AlarmDuration(byte alm)
{
  unsigned long t;
  if (adjWater1 == 0 || alm == 3) { // 0 = automatic adjustment is OFF
    t = almDurValue[(alm == 1 ? alm1Duration : (alm == 2 ? alm2Duration : alm3Duration)) - 1];
    t = t * 60000;
  }
  else {
    t = almDurValue[(alm == 1 ? alm1Duration : alm2Duration) - 1] * 60;
    t = t * (100+((adjWaterResult * (alm == 1 ? adjWater1 : adjWater2) * 50)/100)) * 10; // automatic adjustment of the watering period
  }

  if (heatWave != 0 && alm != 3 && sensorErr<2) { // if option heat is ON and the limit temperature has been reached, apply the adjustment to the watering period
    if (maxTempValue >= (heatWave + 29)) {
      t = t / 100;
      t = t * (100 + HEAT_MULTIPLIER);
    }
  }
  return t;
}

void SwitchOffAlarms() {
  if (AlmTrig == 1 || AlmTrig == 2 || AlmTrig == 3) {
    unsigned long mss = AlarmDuration(AlmTrig);
    if ((manualStop) || ((millis() - lastAlm) > mss)) { // manual stop or alarm elapsed
      AlmTrig = 0;
      sleepCt = 0;
      manualStop = false;
      if (AlmTrig != 3) {
        AdjNextAlarmAndWater();
        SaveSettings();
      }

      Screen.clear();
      digitalWrite(PIN_OUT, LOW); // switch-off the output signal
    }
  }
}


void MonitorButtons() {
  bool btnValue;
  byte by;
  // Keep track of the buttons using global variables btn1, btn2, btn3.
  // A button can have one of these statuses: 0=pressed, 1=not pressed, 2=press process completed, turning to 1.
  btnValue = digitalRead(PIN_BTN1);
  if (btnValue != lastBtn1Status) { // every time the button status changes, restarts the debounce process
    lastPin1Debounce = millis();
  }
  if ((millis() - lastPin1Debounce) > DEBOUNCE_DELAY) { // Debounce button
    if (btnValue != btn1Status) {
      btn1Status = btnValue; // track the status of the button 1
    }
  }
  lastBtn1Status = btnValue;
  btnValue = digitalRead(PIN_BTN2);
  if (btnValue != lastBtn2Status) {
    lastPin2Debounce = millis();
  }
  if ((millis() - lastPin2Debounce) > DEBOUNCE_DELAY) {
    if (btnValue != btn2Status) {
      btn2Status = btnValue;
    }
  }
  lastBtn2Status = btnValue;
  btnValue = digitalRead(PIN_BTN3);
  if (btnValue != lastBtn3Status) {
    lastPin3Debounce = millis();
  }
  if ((millis() - lastPin3Debounce) > DEBOUNCE_DELAY) {
    if (btnValue != btn3Status) {
      btn3Status = btnValue;
    }
  }
  lastBtn3Status = btnValue;
  //to enter the edit mode, the blinkstatus must have visible status, and we are not already in edit mode, and the screen is not read-only, and button 1 is pressed for more than 1.4 seconds
  if (blinkStatus && !btn1Keep && blinkPosCt[activeScreen - 1] != 0 && pressCt > (1400 / EDIT_FREQ) && btn1Status == 0 ) {
    if (AlmTrig == 0) { // also enter the edit mode only if no alarm is triggered
      editMode = !editMode; // switch mode
      blinkPos = editMode == true ? blinkPosCt[activeScreen - 1] : 0; // when entering the edit mode, set the blinking pointer to the last input position of the screen, so edit can start from the first input position after button is processed.
      // SAVE DATA ////////////////////////////////////////////////////////
      if (!editMode) { // if exiting the edit mode
        if (activeScreen == 1 || activeScreen == 2 || activeScreen == 3) {
          AdjNextAlarmAndWater();
        }
        else if (activeScreen == 4) {// Screen Geo Location
          int mm1, mm2;
          float tmz = timezone;
          Dusk2Dawn D2D(latitude, longitude, tmz / 4);
          by = 1;
          maxDaylightMinutes = 0;
          minDaylightMinutes = 1440;
          while ( by < 13 )
          {
            mm1  = D2D.sunrise(dtm.year, by, 21, false);
            mm2  = D2D.sunset(dtm.year, by, 21, false);
            if ((mm1 > 1440) || (mm2 > 1440)) {
              mm1 = 0;
              mm2 = 1440;
            }
            if ((mm2 - mm1) < minDaylightMinutes) {
              minDaylightMinutes = (mm2 - mm1);
            }
            if ((mm2 - mm1) > maxDaylightMinutes) {
              maxDaylightMinutes = (mm2 - mm1);
            }
            by++;
          }
          AdjNextAlarmAndWater();
        }
        else if (activeScreen == 5) {// Clock calibration
          CreateCalibrationMatrix();
        }
        else if (activeScreen == 6) {// Watering now
          AlmTrig = 3;
          lastAlm = millis(); // this global variable is used to keep track of the seconds elapsed since the last alarm was triggered
          blinkPos = 0; // a screen can have zero, one or more edit cells. if the blinking position is zero no cell is blinking
          editMode = false; // start in view mode
          Screen.clear();
          activeScreen = 1; // current screen for view/edit mode
          digitalWrite(PIN_OUT, HIGH);
        }
        SaveSettings();
      } // end of save data
    }
    else {
      if (activeScreen == 1) { // If the alarm is triggered and the user (only in the time screen) press the button 1 for more than 2 seconds, switch-off the watering.
        manualStop = true;
      } else { // manual stop of the watering is allowed only in the main screen
        showMsg(0, F("Turn OFF"));
        showMsg(2, F("watering"));
        showMsg(4, F("from the"));
        showMsg(6, F("Time Screen"));
        delay(3000);
        Screen.clear();
      }
    }
    btn1Keep = true; // Track the status "button pressed for more than 1.4 seconds"
  }
  if (btn1Keep && btn1Status == 1) { // If the button 1 is not pressed restart the timer for entering the edit mode
    btn1Keep = false;
    pressCt = 0; // restart the timer for entering the edit mode.
  }
  if (editMode == true && btn1Status == 0 && btn1 == 1) { // If in edit mode and the button 1 is pressed (and it was not pressed)
    btn1 = 0; // save the status (pressed) of the button, it will be processed by the method: ProcessButtons
    sleepCt = 0; // restart the timer for entering the sleep mode.
  }
  if (btn1Status == 1 && btn1 == 2) { // If the button 1 is not pressed and the previous button 1 process has been completed
    btn1 = 1; // save the status (not pressed) of the button
    sleepCt = 0;
  }
  if (btn2Keep && pressCt > (800 / EDIT_FREQ) && btn2Status == 0 ) { // when in edit mode the button 2 is kept pressed for more than 0.8 seconds, the value is increased/decreased automatically.
    btn2Keep = false; // this is done mocking up the release of the button
  }
  if (!btn2Keep && btn2Status == 1) { // if the button 2 is not pressed we reset also the keep status to not pressed
    btn2Keep = true;
  }
  if (btn3Keep && pressCt > (800 / EDIT_FREQ) && btn3Status == 0 ) { // when in edit mode the button 3 is kept pressed for more than 0.8 seconds, the value is increased/decreased automatically.
    btn3Keep = false; // this is done mocking up the release of the button
  }
  if (!btn3Keep  && btn3Status == 1) { // if the button 3 is not pressed we reset also the keep status to not pressed
    btn3Keep = true;
  }
  if (!btn1Keep) {
    if (btn2Status == 0 && btn2 == 1) { // copy the button 2 status (pressed) to a global variable
      btn2 = 0;
      sleepCt = 0;
    }
    if (btn2Status == 1 && btn2 == 2) { // copy the button 2 status (released) to a global variable
      btn2 = 1;
      sleepCt = 0;
    }
    if (btn3Status == 0 && btn3 == 1) { // copy the button 3 status (pressed) to a global variable
      btn3 = 0;
      sleepCt = 0;
    }
    if (btn3Status == 1 && btn3 == 2) { // copy the button 3 status (released) to a global variable
      btn3 = 1;
      sleepCt = 0;
    }
  }
}

void Blink(int milliseconds, int del) {
  digitalWrite(PIN_LED, HIGH);   // turn the LED ON
  delay(milliseconds);           // period with the led ON
  digitalWrite(PIN_LED, LOW);    // turn OFF the led
  delay(del);                    // pause after blink
}

uint8_t dec2bcd(uint8_t num) { // Convert Decimal to Binary Coded Decimal (BCD)
  return ((num / 10 * 16) + (num % 10));
}

uint8_t bcd2dec(uint8_t num) { // Convert Binary Coded Decimal (BCD) to Decimal
  return ((num / 16 * 10) + (num % 16));
}


dateTime GetDateTime() { // Read date and time from the clock. The values are stored in the clock registry with a binary encoded decimal format, and they must be converted to decimal
  Wire.beginTransmission(CLOCK_ADDRESS);
  Wire.write((uint8_t)0x00);
  if (Wire.endTransmission() != 0) {
    digitalWrite(PIN_LED, HIGH);
    showMsg(0, F(" -G"));
    while (1) {}; // without clock the system cannot proceed and will reset until a working clock will be found
  }
  else
  {
    dateTime res;
    byte rdata = 0x00;
    Wire.requestFrom(CLOCK_ADDRESS, 7);
    if (Wire.available()) rdata = Wire.read();
    res.second = bcd2dec(rdata & 0x7f);
    if (Wire.available()) rdata = Wire.read();
    res.minute = bcd2dec(rdata);
    if (Wire.available()) rdata = Wire.read();
    res.hour = bcd2dec(rdata & 0x3f);
    if (Wire.available()) rdata = Wire.read();
    res.dow = bcd2dec(rdata);
    if (Wire.available()) rdata = Wire.read();
    res.day = bcd2dec(rdata);
    if (Wire.available()) rdata = Wire.read();
    res.month = bcd2dec(rdata);
    if (Wire.available()) rdata = Wire.read();
    res.year = bcd2dec(rdata);
    return res;
  }
  delay(SYSTEM_DELAY);
}

void SetDateTime(dateTime val) { // Write date and time to the clock. Values must be saved to the clock with a binary encoded decimal format.
  Wire.beginTransmission(CLOCK_ADDRESS);
  Wire.write((uint8_t)0x00);
  if (Wire.endTransmission() != 0) {
    digitalWrite(PIN_LED, HIGH);
    showMsg(0, F("CLKER-S"));
    while (1) {}; // without clock the system cannot work and it will reset until a clock is found
  }
  else {
    Wire.beginTransmission(CLOCK_ADDRESS);
    Wire.write((uint8_t)0x00);
    Wire.write((uint8_t)0x80);
    Wire.write((uint8_t)dec2bcd(val.minute));
    Wire.write((uint8_t)dec2bcd(val.hour & 0x3f));
    Wire.write(val.dow);
    Wire.write((uint8_t)dec2bcd(val.day));
    Wire.write((uint8_t)dec2bcd(val.month));
    Wire.write((uint8_t)dec2bcd(val.year));
    Wire.endTransmission();
    Wire.beginTransmission(CLOCK_ADDRESS);
    Wire.write((uint8_t)0x00);
    Wire.write((uint8_t)dec2bcd(val.second));
    Wire.endTransmission();
  }
  delay(SYSTEM_DELAY);
}



void ActivateClockOscillator() { //activate the clock oscillator if not yet active
  bool isActive = true;
  Wire.beginTransmission(CLOCK_ADDRESS);
  Wire.write((uint8_t)0x00);
  if (Wire.endTransmission() != 0) {
    digitalWrite(PIN_LED, HIGH);
    showMsg(0, F("CLKER-O"));
    while (1) {}; // without clock the system cannot work and it will reset until a clock is found
  }
  else {
    Wire.requestFrom(CLOCK_ADDRESS, 1);
    byte rdata = Wire.read();
    if ((rdata & 0b10000000) == 0b10000000) {
      isActive = false;
    }
    if (!isActive) {
      Wire.beginTransmission(CLOCK_ADDRESS);
      Wire.write((uint8_t)0x00);
      Wire.write((uint8_t)dec2bcd(0));
      Wire.endTransmission();
    }
  }
  delay(SYSTEM_DELAY);
}



void AdjNextAlarmAndWater() {
  byte hh, mm, by;
  int tmm;
  int mm1, mm2, cu;
  unsigned long f;
  float tmz = timezone;
  Dusk2Dawn D2D(latitude, longitude, tmz / 4);
  mm1 = D2D.sunrise(dtm.year + 2000, dtm.month, dtm.day, IsSummerTime(dtm));
  mm2 = D2D.sunset(dtm.year + 2000, dtm.month, dtm.day, IsSummerTime(dtm));
  //dtm = GetDateTime();
  for (byte alm = 1; alm < 3; alm++ ) {
    by = alm == 1 ? alm1Option : alm2Option;
    if (by > 1) { // adjust with dusk/dawn
      if (alm == 1) {
        tmm  = mm1;
      }
      else {
        tmm  = mm2;
      }
      tmm = tmm + (alm == 1 ? alarmDDAdjust[by] : -alarmDDAdjust[by]) * 10;
      if (tmm < 0 || tmm > 1440) {
        tmm = 0;
      }
      mm = tmm % 60;
      if (mm > 59) {
        mm = 0;
      }
      hh = tmm / 60;
      if (hh > 23) {
        hh = 0;
      }
      if (alm == 1) {
        alm1DateTime.hour = hh;
        alm1DateTime.minute = mm;
      }
      else {
        alm2DateTime.hour = hh;
        alm2DateTime.minute = mm;
      }
    }
  }
  //adjust watering period
  cu = (mm2 - mm1);
  f = (cu - minDaylightMinutes);
  f = (f * 100) / (maxDaylightMinutes - minDaylightMinutes);
  adjWaterResult = (byte)(f);
}



void ProcessButtons() {
  if (digitalRead(PIN_BTN1) == 0 || digitalRead(PIN_BTN2) == 0 || digitalRead(PIN_BTN3) == 0) { // if one of the buttons is pressed
    pressCt = pressCt + 1; // increase the timer for entering the edit mode
    sleepCt = 0; // restart the timer for entering the sleep mode.
  }
  else {
    pressCt = 0; // restart the timer for entering the edit mode
  }
  if (btn1 == 0) {
    btn1 = 2;
    if (blinkPos == 1 && activeScreen == 3 && alm1Option > 1) { // skip blink positions for time if the alarm is automatic
      blinkPos = 4;
    } else if (blinkPos == 1 && activeScreen == 4 && alm2Option > 1) {
      blinkPos = 4;
    } else {
      blinkPos = blinkPos + 1;
    }
    refreshNow = true;
    if (blinkPos == (blinkPosCt[activeScreen - 1] + 1)) {
      blinkPos = 1;
    }
  }
  if (btn2 == 0 || btn3 == 0) {
    refreshNow = true;
    if (editMode == false) { // In view mode the navigation buttons are used to switch between screens.
      if (skipButtonProcess) //we don't process the button that woke up from sleep mode
      {
        skipButtonProcess = false;
        refreshNow = false;
      }
      else
      {
        if (activeScreen == 1 && btn2 == 0) {
          activeScreen = sizeof(blinkPosCt);
        }
        else if (activeScreen == sizeof(blinkPosCt) && btn3 == 0) {
          activeScreen = 1;
        }
        else {
          activeScreen = activeScreen + ((btn2 == 0) ? -1 : + 1);
        }
        Screen.clear();
      }

    }
    else {
      //***********************************************************
      if (activeScreen == 1) { // SCREEN DATETIME
        if (blinkPos == 1) {
          dtm.dow = ((dtm.dow  == 1 && btn2 == 0) ? 7 : (dtm.dow == 7 && btn3 == 0) ? 1 : (dtm.dow + ((btn2 == 0) ? -1 : + 1)));
        }
        else if (blinkPos == 2) {
          dtm.day = ((dtm.day  == 1 && btn2 == 0) ? 31 : (dtm.day == 31 && btn3 == 0) ? 1 : (dtm.day + ((btn2 == 0) ? -1 : + 1)));
        }
        else if (blinkPos == 3) {
          dtm.month = ((dtm.month  == 1 && btn2 == 0) ? 12 : (dtm.month == 12 && btn3 == 0) ? 1 : (dtm.month + ((btn2 == 0) ? -1 : + 1)));
        }
        else if (blinkPos == 4) {
          dtm.year = (((dtm.year)  == 0 && btn2 == 0) ? 99 : ((dtm.year) == 99 && btn3 == 0) ? 0 : ((dtm.year) + ((btn2 == 0) ? -1 : + 1)));
        }
        else if (blinkPos == 5) {
          dtm.hour = ((dtm.hour  == 0 && btn2 == 0) ? 23 : (dtm.hour == 23 && btn3 == 0) ? 0 : (dtm.hour + ((btn2 == 0) ? -1 : + 1)));
        }
        else if (blinkPos == 6) {
          dtm.minute = ((dtm.minute  == 0 && btn2 == 0) ? 59 : (dtm.minute == 59 && btn3 == 0) ? 0 : (dtm.minute + ((btn2 == 0) ? -1 : + 1)));
        }
        else if (blinkPos == 7) {
          dtm.second = ((dtm.second  == 0 && btn2 == 0) ? 59 : (dtm.second == 59 && btn3 == 0) ? 0 : (dtm.second + ((btn2 == 0) ? -1 : + 1)));
        }
        SetDateTime(dtm);
        AdjNextAlarmAndWater();
      }
      else if (activeScreen == 2) { // SCREEN GEOLOCATION
        if (blinkPos == 1) {
          timezone = ((timezone  == -48 && btn2 == 0) ? 56 : (timezone == 56 && btn3 == 0) ? -48 : (timezone + ((btn2 == 0) ? -1 : + 1)));
        }
        if (blinkPos == 2) {
          daySav = !daySav;
        }
        if (blinkPos == 3) {
          latitude = ((latitude  == -90 && btn2 == 0) ? 90 : (latitude == 90 && btn3 == 0) ? -90 : (latitude + ((btn2 == 0) ? -1 : + 1)));
        }
        if (blinkPos == 4) {
          longitude = ((longitude  == -180 && btn2 == 0) ? 180 : (longitude == 180 && btn3 == 0) ? -180 : (longitude + ((btn2 == 0) ? -1 : + 1)));
        }
      }      
      else if (activeScreen == 3) { // SCREEN ALARM 1
        if (blinkPos == 1) {
          alm1Option = ((alm1Option  == 0 && btn2 == 0) ? 8 : (alm1Option == 8 && btn3 == 0) ? 0 : (alm1Option + ((btn2 == 0) ? -1 : + 1)));
          AdjNextAlarmAndWater();
        }
        else if (blinkPos == 2) {
          alm1DateTime.hour = ((alm1DateTime.hour  == 0 && btn2 == 0) ? 23 : (alm1DateTime.hour == 23 && btn3 == 0) ? 0 : (alm1DateTime.hour + ((btn2 == 0) ? -1 : + 1)));
        }
        else if (blinkPos == 3) {
          alm1DateTime.minute = ((alm1DateTime.minute  == 0 && btn2 == 0) ? 59 : (alm1DateTime.minute == 59 && btn3 == 0) ? 0 : (alm1DateTime.minute + ((btn2 == 0) ? -1 : + 1)));
        }
        else if (blinkPos == 4) {
          alm1Duration = ((alm1Duration  == 1 && btn2 == 0) ? 21 : (alm1Duration == 21 && btn3 == 0) ? 1 : (alm1Duration + ((btn2 == 0) ? -1 : + 1)));
        }
        else if (blinkPos == 5) {
          adjWater1 = ((adjWater1  == 0 && btn2 == 0) ? 8 : (adjWater1 == 8 && btn3 == 0) ? 0 : (adjWater1 + ((btn2 == 0) ? -1 : + 1)));
          AdjNextAlarmAndWater();
        }
        else if (blinkPos == 6) {
          skpHumidity1 = ((skpHumidity1  == 0 && btn2 == 0) ? 99 : (skpHumidity1 == 99 && btn3 == 0) ? 0 : (skpHumidity1 + ((btn2 == 0) ? -1 : + 1)));
        }
        if (blinkPos == 7) {
          skpDays1 = ((skpDays1  == 0 && btn2 == 0) ? 7 : (skpDays1 == 7 && btn3 == 0) ? 0 : (skpDays1 + ((btn2 == 0) ? -1 : + 1)));
        }
      }
      else if (activeScreen == 4) { // SCREEN ALARM 2
        if (blinkPos == 1) {
          alm2Option = ((alm2Option  == 0 && btn2 == 0) ? 8 : (alm2Option == 8 && btn3 == 0) ? 0 : (alm2Option + ((btn2 == 0) ? -1 : + 1)));
          AdjNextAlarmAndWater();
        }
        else if (blinkPos == 2) {
          alm2DateTime.hour = ((alm2DateTime.hour  == 0 && btn2 == 0) ? 23 : (alm2DateTime.hour == 23 && btn3 == 0) ? 0 : (alm2DateTime.hour + ((btn2 == 0) ? -1 : + 1)));
        }
        else if (blinkPos == 3) {
          alm2DateTime.minute = ((alm2DateTime.minute  == 0 && btn2 == 0) ? 59 : (alm2DateTime.minute == 59 && btn3 == 0) ? 0 : (alm2DateTime.minute + ((btn2 == 0) ? -1 : + 1)));
        }
        else if (blinkPos == 4) {
          alm2Duration = ((alm2Duration  == 1 && btn2 == 0) ? 21 : (alm2Duration == 21 && btn3 == 0) ? 1 : (alm2Duration + ((btn2 == 0) ? -1 : + 1)));
        }
        else if (blinkPos == 5) {
          adjWater2 = ((adjWater2  == 0 && btn2 == 0) ? 8 : (adjWater2 == 8 && btn3 == 0) ? 0 : (adjWater2 + ((btn2 == 0) ? -1 : + 1)));
          AdjNextAlarmAndWater();
        }
        else if (blinkPos == 6) {
          skpHumidity2 = ((skpHumidity2  == 0 && btn2 == 0) ? 99 : (skpHumidity2 == 99 && btn3 == 0) ? 0 : (skpHumidity2 + ((btn2 == 0) ? -1 : + 1)));
        }
        if (blinkPos == 7) {
          skpDays2 = ((skpDays2  == 0 && btn2 == 0) ? 7 : (skpDays2 == 7 && btn3 == 0) ? 0 : (skpDays2 + ((btn2 == 0) ? -1 : + 1)));
        }
      }
      else if (activeScreen == 5) { // SCREEN OPTIONS
        if (blinkPos == 1) {
          tempUnit = !tempUnit;
        }
        if (blinkPos == 2) {
          skpIce = ((skpIce  == 0 && btn2 == 0) ? 16 : (skpIce == 16 && btn3 == 0) ? 0 : (skpIce + ((btn2 == 0) ? -1 : + 1)));
        }
        if (blinkPos == 3) {
          heatWave = ((heatWave  == 0 && btn2 == 0) ? 26 : (heatWave == 26 && btn3 == 0) ? 0 : (heatWave + ((btn2 == 0) ? -1 : + 1)));
        }
        if (blinkPos == 4) {
          clockCalibration = ((clockCalibration  == -240 && btn2 == 0) ? 240 : (clockCalibration == 240 && btn3 == 0) ? -240 : (clockCalibration + ((btn2 == 0) ? -1 : + 1)));
        }
        if (blinkPos == 5) {
          sensorType = ((sensorType  == 0 && btn2 == 0) ? 4 : (sensorType == 4 && btn3 == 0) ? 0 : (sensorType + ((btn2 == 0) ? -1 : + 1)));
        }
      }

      else if (activeScreen == 6) { // WATERING NOW
        if (blinkPos == 1) {
          alm3Duration = ((alm3Duration  == 1 && btn2 == 0) ? 21 : (alm3Duration == 21 && btn3 == 0) ? 1 : (alm3Duration + ((btn2 == 0) ? -1 : + 1)));
        }
      }
      blinkStatus = true;
    }
  }

  if (btn2 == 0) {
    btn2 = 2;
  }
  else if (btn2 == 2 && !btn2Keep) {
    btn2 = 1;
  }
  if (btn3 == 0) {
    btn3 = 2;
  }
  else if (btn3 == 2 && !btn3Keep) {
    btn3 = 1;
  }
}

//convert milliseconds in the corresponding timespan with format hh:mm:ss
//hh is shown only if greater than zero
char * msToChar(char * str, unsigned long mss)
{
  int i = 0;
  char cpy[3] = {0, 0, 0};
  mss = mss / 1000;
  unsigned long h = mss / 3600;
  if (h > 0)
  {
    lngToChar(cpy, h, true);
    str[i++] = cpy[0];
    str[i++] = cpy[1];
    str[i++] = ':';
  }
  h = (mss % 3600) / 60;
  lngToChar(cpy, h, true);
  str[i++] = cpy[0];
  str[i++] = cpy[1];
  str[i++] = ':';
  h = (mss % 3600) % 60;
  lngToChar(cpy, h, true);
  str[i++] = cpy[0];
  str[i++] = cpy[1];
  str[i] = '\0';
  return str;
}

// https://www.geeksforgeeks.org/implement-itoa/
// modified to accept long, and to add an optional leading zero for the time digits
char * lngToChar(char * str, long num, bool leadingZero)
{
  int i = 0;
  bool isNegative = false;
  if (num < -9 || num > 9) //apply leading zero only to 1 digit numbers
  {
    leadingZero = false;
  }
  // Handle 0 explicitely, otherwise empty string is printed for 0 
  if (num == 0)
  {
    str[i++] = '0';
    if (leadingZero == true)
    {
      str[i++] = '0';
    }
    str[i] = '\0';
    return str;
  }
  // In standard itoa(), negative numbers are handled only with
  // base 10. Otherwise numbers are considered unsigned.
  if (num < 0)
  {
    isNegative = true;
    num = -num;
  }
  // Process individual digits
  while (num != 0)
  {
    int rem = num % 10;
    str[i++] = (rem > 9) ? (rem - 10) + 'a' : rem + '0';
    num = num / 10;
  }
  if (leadingZero == true)
  {
    str[i++] = '0';
  }
  // If number is negative, append '-'
  if (isNegative)
    str[i++] = '-';
  str[i] = '\0'; // Append string terminator
  // Reverse the string
  reverse(str, i);
  return str;
}


// reverse a string except for the last char
void reverse(char str[], int length)
{
  int start = 0;
  int end = length - 1;
  while (start < end)
  {
    SWAP(*(str + start), *(str + end));
    start++;
    end--;
  }
}

void PrintScreen(byte scr) {
  char buffer1[11];
  char buffer2[3];
  unsigned long mss;
  int temp = (tempUnit ? ((temperatureSensorValue * 9) / 5 + 32) : temperatureSensorValue);
  bool blkStatus = (blinkStatus == true && btn2Keep && btn3Keep);
  Screen.setFont(lcd5x7m); // font LCD5x7 revisited
  Screen.setLetterSpacing(1); // thanks to the fix applied to the screen libray (see above), the letter spacing of 1 pixel is not doubled like the characters, allowing 11 chars to fit in the screen
  Screen.set2X();
  if (scr == 1) { // MAIN DATETIME SCREEN  ///////////////////////////////////////////////////////////////////////////////////////////////////
    if (AlmTrig == 1 || AlmTrig == 2 || AlmTrig == 3) // watering is ON
    {
      Print(0, 0, false, F("&^"), blinkStatus == true  ? ""  : "  WATERING", "", "", "", "");
      mss = AlarmDuration(AlmTrig);
      mss = mss - (millis() - lastAlm) + 1000;
      Print(0, 6, false, mss > 3600000 ? F("& &%^") : F(" &  &%^"), msToChar(cstr1, mss), (sensorErr<2 && (sensorType==2 or sensorType==4)) ? lngToChar(cstr2, soilSensorValue, false) : "--" , "", "", "");
    }
    else
    {
      strcpy_P(buffer1, (char *)pgm_read_word(&(months[dtm_ds.month - 1])));
      strcpy_P(buffer2, (char *)pgm_read_word(&(weekDays[dtm_ds.dow - 1])));
      Print(0, 0, false, dtm_ds.day < 10 ? F("[[[[[& & & &^") : F("[[[&]]&]]&]]&^"), (blkStatus == true && blinkPos == 1 ? "  "  : buffer2),
            (blkStatus == true && blinkPos == 2 ? (dtm_ds.day < 10 ? " " : "  ")  : lngToChar(cstr1, dtm_ds.day, false)),
            (blkStatus == true && blinkPos == 3 ? "   "  : buffer1), (blkStatus == true && blinkPos == 4 ? "  "  : lngToChar(cstr2, dtm_ds.year, false)), "");

      strcpy_P(buffer1, (char *)pgm_read_word(&(batteryChargePict[batteryLevel])));

      int sp = (sensorErr<2 && (sensorType==2 or sensorType==4)) ? ((temp < -9 || temp > 99 ? 1 : 5) + (temp > -1 && temp < 10 ? 5 : 0) + (soilSensorValue < 10 ? 5 : 0)) : 5;
      
      Screen.clear(0, sp - 1, 6 , 7);
      Print(sp, 6, false, F("& ]&& ]&%^"), (batteryLevel == 0 && blinkStatus == true) ? "  " : buffer1, (sensorErr<2 && (sensorType>0)) ? lngToChar(cstr1, temp, false) : "--" , tempUnit ? "," : "~", (sensorErr<2 && (sensorType==2 or sensorType==4)) ? lngToChar(cstr2, soilSensorValue, false) : "--", "");
    }
    //time
    Screen.setFont(arialNarrow15x27);
    Screen.setLetterSpacing(0);
    Screen.set1X();
    Print(4, 2, false, F("&:&:&^"), (blkStatus == true && blinkPos == 5 ? "//"  : lngToChar(cstr1, dtm_ds.hour, true)),
          (blkStatus == true && blinkPos == 6 ? "//"  : lngToChar(cstr2, dtm_ds.minute, true)),
          (blkStatus == true && blinkPos == 7 ? "//"  : lngToChar(cstr3, dtm_ds.second, true)), "", "");
  }
  else if (scr == 2) { // SCREEN GEOLOCATION ///////////////////////////////////////////////////////////////////////////////////////////////////
    Print(0, 0, true, F("[[[GEOLOCATION^"), "", "", "", "", "");
    Screen.set1X();
    int r = abs(timezone % 4);
    Print(0, 2, false, (blkStatus == true && blinkPos == 1) ? F("TIMEZONE= ^") : F("TIMEZONE= &&&&^"),
          (timezone > -1 ? "+" : (timezone / 4 == 0 ? "-" : "")),
          lngToChar(cstr1, (timezone / 4), false),
          (r == 0 ? "" : (r == 1 ? ":15" : (r == 2 ? ":30" : ":45"))),
          "", "");
    Print(0, 3, false, F("DAYLIGHT SAVING= &^"), (blkStatus == true && blinkPos == 2 ? ""  : (daySav ? "ON" : "OFF")), "", "", "", "");
    Print(0, 4, false, (blkStatus == true && blinkPos == 3) ? F("LATITUDE= ^") : F("LATITUDE= &&^"), (latitude > -1 ? "+" : ""),
          lngToChar(cstr1, latitude, false),
          (latitude > -10 && latitude < 10) ? " " : "" , "", "");
    Print(0, 5, false, (blkStatus == true && blinkPos == 4) ? F("LONGITUDE= ^") : F("LONGITUDE= &&^"), (longitude > -1 ? "+" : ""),
          lngToChar(cstr1, longitude, false), "", "", "");
  }  
  else if ((scr == 3) || (scr == 4)) { // SCREEN ALARM 1 & 2 ///////////////////////////////////////////////////////////////////////////////////////////////////
    strcpy_P(buffer1, (char *)pgm_read_word(&(scr == 3 ? alm1Opts[alm1Option] : alm2Opts[alm2Option])));
    Print(0, 0, true, F("  [[[ALARM-&^"), scr == 3 ? "1" : "2", "", "", "", "");
    Screen.set1X();
    Print(0, 2, false, F("MODE= &^"), (blkStatus == true && blinkPos == 1 ? ""  : buffer1), "", "", "", "");
    Print(0, 3, false, F("TIME= &:&^"), (blkStatus == true && blinkPos == 2 ? "  "  : lngToChar(cstr1, scr == 3 ? alm1DateTime.hour : alm2DateTime.hour, true)),
          (blkStatus == true && blinkPos == 3 ? ""  : lngToChar(cstr2, scr == 3 ? alm1DateTime.minute : alm2DateTime.minute, true)),
          "", "", "");
    strcpy_P(buffer1, (char *)pgm_read_word(&(almDurOpts[(scr == 3 ? alm1Duration : alm2Duration) - 1])));
    Print(0, 4, false, F("DURATION= &^"), (blinkStatus == true && blinkPos == 4 && btn2Keep && btn3Keep ? ""  : buffer1), "", "", "", "");
    Print(0, 5, false, (blkStatus == true && blinkPos == 5) ? F("SEASON ADJUST= ^") : ((scr == 3 ? adjWater1 : adjWater2) == 0 ? F("SEASON ADJUST= OFF^") : F("SEASON ADJUST= +&%^")), lngToChar(cstr1, (scr == 3 ? adjWater1 : adjWater2)*50, false), "", "", "", "");
    Print(0, 6, false, (blkStatus == true && blinkPos == 6) ? F("SKIP HUMIDITY= ^") : ((scr == 3 ? skpHumidity1 : skpHumidity2) == 0 ? F("SKIP HUMIDITY= OFF^") : F("SKIP HUMIDITY= &%^")), lngToChar(cstr1, scr == 3 ? skpHumidity1 : skpHumidity2, false),
          "", "", "", "");
    Print(0, 7, false, (blkStatus == true && blinkPos == 7) ? F("SKIP DAYS= ^") : ((scr == 3 ? skpDays1 : skpDays2) == 0 ? F("SKIP DAYS= OFF^") : ((scr == 3 ? skpDays1 : skpDays2) == 1 ? F("SKIP DAYS= AUTO^") : F("SKIP DAYS= &^"))), lngToChar(cstr1, (scr == 3 ? skpDays1 : skpDays2) - 1, false), "", "", "", "");
  }

  else if (scr == 5) { // SCREEN OPTIONS ///////////////////////////////////////////////////////////////////////////////////////////////////
    Print(0, 0, true, F("  [[[OPTIONS^"), "", "", "", "", "");
    Screen.set1X();
    int tmpi = (tempUnit ? (((skpIce - 5) * 9) / 5 + 32) : (skpIce - 5));
    Print(0, 2, false, F("TEMPER. UNIT= &^"), (blkStatus == true && blinkPos == 1 ? ""  : (tempUnit ? "FAHRENH." : "CELSIUS")), "", "", "", "");
    Print(0, 3, false, (blkStatus == true && blinkPos == 2) ? F("SKIP ICE= ^") : (skpIce == 0 ? F("SKIP ICE= OFF^") : (tempUnit ? F("SKIP ICE= &&,^") : F("SKIP ICE= &&~^"))), (tmpi >= 0 ? "+" : ""),
          lngToChar(cstr1, tmpi, false), "",  "", "");
    Print(0, 4, false, F("HEATWAVE ADJUST= &&^"), (blkStatus == true && blinkPos == 3 ? "    "  : (heatWave == 0 ? "OFF" : lngToChar(cstr1, (tempUnit ? (((heatWave + 29) * 9) / 5 + 32) : (heatWave + 29)) , false))),
          (blkStatus == true && blinkPos == 3 ? " "  : (heatWave == 0 ? " " : (tempUnit ? "," : "~"))), "", "", "");
    Print(0, 5, false, F("CLOCK CALIBRAT.= &&^"), (blkStatus == true && blinkPos == 4 ? " "  : (clockCalibration >= 0 ? "+" : "")),
          (blkStatus == true && blinkPos == 4 ? "   "  : lngToChar(cstr1, clockCalibration, false)), "", "", "");
    strcpy_P(buffer1, (char *)pgm_read_word(&(scr == 3 ? sensorTypes[sensorType] : sensorTypes[sensorType])));
    Print(0, 6, false, F("SENSOR= &^"), (blkStatus == true && blinkPos == 5 ? ""  : buffer1), "", "", "", "");
              
  }

  else if (scr == 6) { // IMMEDIATE WATER ///////////////////////////////////////////////////////////////////////////////////////////////////
    Print(0, 0, true, F("[WATERING[[]NOW^"), "", "", "", "", "");
    strcpy_P(buffer1, (char *)pgm_read_word(&(almDurOpts[alm3Duration - 1])));
    Print(0, 4, false, F("Dur.=&^"), (blkStatus == true && blinkPos == 1 ? ""  : buffer1), "", "", "", "");
  }

  else if (scr == 7) { // SCREEN TEMPERATURE ///////////////////////////////////////////////////////////////////////////////////////////////////
    Print(0, 0, true, F("[[[TEMPERATURE^"), "", "", "", "", "");
    int tmpi = (tempUnit ? ((maxTempValue * 9) / 5 + 32) : maxTempValue);
    Print(0, 2, false, F("{&& &:&^"), lngToChar(cstr1, tmpi, false), (tempUnit ? "," : "~"), lngToChar(cstr2, maxTempDateTime.hour, true), lngToChar(cstr3, maxTempDateTime.minute, true), "");
    Print(0, 4, false, F(" &&^"), lngToChar(cstr1, temp, false), (tempUnit ? "," : "~"), "", "", "");
    tmpi = (tempUnit ? ((minTempValue * 9) / 5 + 32) : minTempValue);
    Print(0, 6, false, F("|&& &:&^"), lngToChar(cstr1, tmpi, false), (tempUnit ? "," : "~"), lngToChar(cstr2, minTempDateTime.hour, true), lngToChar(cstr3, minTempDateTime.minute, true), "");
  }
  else if (scr == 8) { // SCREEN ABOUT ///////////////////////////////////////////////////////////////////////////////////////////////////
    
    Screen.set1X();
    Print(0, 0, false, F("SUNFY-328 V2.1^"), "", "", "", "", "");
    Print(0, 1, false, F("WWW.PICAPOT.COM^"), "", "", "", "", "");
    Print(0, 2, false, F("LAST AL1=20&-&-&^"), lngToChar(cstr1, alm1DateTime.year, true), lngToChar(cstr2, alm1DateTime.month, true),  lngToChar(cstr3, alm1DateTime.day, true), "", "");
    Print(0, 3, false, F("DURATION=&^"), msToChar(cstr1, AlarmDuration(1)), "",  "", "", "");
    Print(0, 4, false, F("LAST AL2=20&-&-&^"), lngToChar(cstr1, alm2DateTime.year, true), lngToChar(cstr2, alm2DateTime.month, true),  lngToChar(cstr3, alm2DateTime.day, true), "", "");
    Print(0, 5, false, F("DURATION=&^"), msToChar(cstr1, AlarmDuration(2)), "",  "", "", "");
    InternalVoltage = readVcc();
    Print(0, 6, false, F("VCC=& BAT=&^"), lngToChar(cstr1, InternalVoltage, false), lngToChar(cstr2, getBatteryMillivolts(), false),  "", "", "");
    Print(0, 7, false, F("HUM=& &% TMP=&&^"), (sensorErr<2 && (sensorType==2 or sensorType==4)) ? lngToChar(cstr1, soilSensorValueRaw, false) : "---" , (sensorErr<2 && (sensorType==2 or sensorType==4)) ? lngToChar(cstr2, soilSensorValue, false) : "--" ,  (sensorErr<2 && (sensorType>0)) ? lngToChar(cstr3, temp, false) : "--" , (tempUnit ? "," : "~"), "");
    
  }
  
}

// Show a text at screen and feed the line
void showMsg(int row, const __FlashStringHelper * s0) {
  char buf[24] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  const char *p = (const char *)s0;
  int i = 0;
  uint8_t c  = 0;
  do
  {
    c = pgm_read_byte(p++);
    buf[i++] = c;
  } while ((c != 0) && (i < 25));
  Screen.setFont(lcd5x7m); 
  Screen.setLetterSpacing(1); 
  Screen.set2X();  
  Print(0, row, false, F("&^"), buf, "", "", "", "");
}

// Print function with formatting functionality. Every character "&" present in s0 is replaced with the corresponding value s(x). Maximum 5 replacements.
// When a parameter is numeric (integer), it must be converted using the function lngToChar. If more numeric values (max 3) are passed in the same call, each value must be converted using a different array cstr[1->3]
// Reserved characters: &=PLACEHOLDER  [=1x PIXEL SPACE  ]=4x PIXEL SPACE ^=LINEFEED(it can be used only as trailing char)
void Print(int x, int y, bool inverted, const __FlashStringHelper * s00, const char * s1, const char * s2, const char * s3, const char * s4, const char * s5) {
  char buf[24] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  byte par = 1;
  int i = 0;
  int sz = -1;
  const char * ref;

  char s0[24] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  const char *p = (const char *)s00;
  uint8_t c  = 0;
  do
  {
    c = pgm_read_byte(p++);
    s0[i++] = c;
  } while ((c != 0) && (i < 25));

  i = 0;
  for (byte y = 0; y < 24; y++ ) {
    if (s0[y] == 0) {
      sz = y;
      break;
    }
  }

  if (sz == -1) { // no string terminator found
    buf[i] = '*';
  }
  else {
    for (byte c = 0; c < sz; c++ ) {
      if (s0[c] == 38) {
        if (par == 1) {
          ref = s1;
        } else if (par == 2) {
          ref = s2;
        } else if (par == 3) {
          ref = s3;
        } else if (par == 4) {
          ref = s4;
        } else {
          ref = s5;
        }
        int szp = -1;
        for (byte y = 0; y < 24; y++ ) {
          if (ref[y] == 0) {
            szp = y;
            break;
          }
        }
        if (szp == -1) { // no string terminator found
          buf[i++] = '*' ;
        }
        else
        {
          for (byte y = 0; y < szp; y++ ) {
            buf[i++] = ref[y];
          }
        }
        par++;
      }
      else {
        buf[i++] = s0[c];
      }
    }
  }
  buf[i] = '\0'; //add string terminator
  Screen.setCursor(x, y);
  Screen.setInvertMode(inverted);
  Screen.write((char *)buf);
}





int dateDiff(dateTime inDtm1, dateTime inDtm2) { // return the difference in days between two dates
  const int dater[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334}; // count of elapsed days from the start of the year for each month.
  int ref, dd1, dd2;
  ref = inDtm1.year;
  if (inDtm2.year < inDtm1.year)
    ref = inDtm2.year;
  dd1 = 0;
  dd1 = dater[inDtm1.month - 1];
  for (int i = ref; i < inDtm1.year; i++)
  {
    if (i % 4 == 0) // leap year
      dd1 += 1;
  }
  dd1 = dd1 + inDtm1.day + (inDtm1.year - ref) * 365;
  dd2 = 0;
  for (int i = ref; i < inDtm2.year; i++)
  {
    if (i % 4 == 0)
      dd2 += 1;
  }
  dd2 = dater[inDtm2.month - 1] + dd2 + inDtm2.day + ((inDtm2.year - ref) * 365);
  return dd2 - dd1;
}


dateTime Add1Hour(dateTime inDtm) { // add one hour
  const int monthDays[12] = {31, (inDtm.year % 4) == 0 ? 29 : 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  if (inDtm.hour == 23)
  {
    inDtm.hour = 0;
    if (inDtm.day == monthDays[inDtm.month - 1])
    {
      if (inDtm.month == 12)
      {
        inDtm.day = 1;
        inDtm.month = 1;
        inDtm.year++;
      }
      else
      {
        inDtm.day = 1;
        inDtm.month++;
      }
    }
    else
    {
      inDtm.day++;
    }
  }
  else
  {
    inDtm.hour++;
  }
  return inDtm;
}


dateTime Remove1Hour(dateTime inDtm) { // remove one hour
  const int monthDays[12] = {31, (inDtm.year % 4) == 0 ? 29 : 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  if (inDtm.hour == 0)
  {
    inDtm.hour = 23;
    if (inDtm.day == 1)
    {
      if (inDtm.month == 1)
      {
        inDtm.day = 31;
        inDtm.month = 12;
        inDtm.year--;
      }
      else
      {
        inDtm.day = monthDays[inDtm.month - 2];
        inDtm.month--;
      }
    }
    else
    {
      inDtm.day--;
    }
  }
  else
  {
    inDtm.hour--;
  }
  return inDtm;
}


bool IsSummerTime(dateTime inDtm) { // return true if datetime is in the summer time period
  inDtm.dow--;
  if (daySav == 0) // if the option "daylight saving" is OFF always return false
  {
    return false;
  }
  bool res = false;
  if (inDtm.month == 1 || inDtm.month == 2 || inDtm.month == 11 || inDtm.month == 12)
  {
    res = false;
  }
  else if (inDtm.month == 3)
  {
    if (inDtm.dow == 0)
    {
      if (((inDtm.day + 7) > 31) && (inDtm.hour >= 2)) {
        res = true;
      }
    }
    else
    {
      if ((inDtm.day + (7 - inDtm.dow)) > 31) {
        res = true;
      }
    }
  }
  else if (inDtm.month >= 4 && inDtm.month <= 9)
  {
    res = true;
  }
  else if (inDtm.month == 10)
  {
    if (inDtm.dow == 0)
    {
      if (((inDtm.day + 7) > 31) && (inDtm.hour >= 3)) {
        res = false;
      }
      else {
        res = true;
      }
    }
    else
    {
      if ((inDtm.day + (7 - inDtm.dow)) > 31) {
        res = false;
      }
      else {
        res = true;
      }
    }
  }
  return res;
}


void getTemperatureAndHumidity_DS18B20() {
  int16_t r = 0;
  long i = 0;
  byte data[12];
  oneWire.target_search(0x28);
  if (oneWire.search(temperatureSensorAddress))
  {
    delay(1);
    if (oneWire.reset()) {
      oneWire.select(temperatureSensorAddress);
      oneWire.write(0xBE, 0);
      oneWire.read_bytes(data, 9);
      int c = oneWire.crc8(data, 8);
      if (c == data[8]) {
        r = (data[1] << 8) | data[0];
        //r = r & ~7;
        r = (float)r / 16;
        byte tmpSign = (data[1] & 0b10000000) + (data[1] & 0b01000000) + (data[1] & 0b00100000) + (data[1] & 0b00010000) + (data[1] & 0b00001000) + (data[1] & 0b00000100);
        i = (long)data[2] * 256 + data[3];
        soilSensorValueRaw = sensorType==3 ? 0 : i;
        if ((r > -41) && (r < 86) && (tmpSign == 0 || tmpSign == 6)) { // && (i < 1024)
          r = r * (tmpSign == 0 ? +1 : -1);
          temperatureSensorValueRaw = r;
          sensorErr=0;
        }
        else
        {
          sensorErr++;
        }
      }
      else
      {
        sensorErr++;
      }
    }
    else
    {
      sensorErr++;
    }
  }
  else
  {
    sensorErr++;
  }
}

unsigned int readI2CSensorRegister(int addr, int reg) {
  unsigned int t = 0;
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) {
    sensorErr++;
  }
  else
  {
    delay(20);
    Wire.requestFrom(addr, 2);
    t = Wire.read() << 8;
    t = t | Wire.read();
    sensorErr=0;
  }
  return t;
}

// Read atmega328 internal temperature
double readInternalTemp(double prevTemp)
{
  // Read temperature sensor against 1.1V reference
  ADMUX = _BV(REFS1) | _BV(REFS0) | _BV(MUX3);
  // Start AD conversion
  ADCSRA |= _BV(ADEN);
  delay(12);
  ADCSRA |= _BV(ADSC);
  // Detect end-of-conversion
  while (bit_is_set(ADCSRA, ADSC));
  // return raw data
  // long rawTemp = ADCL | (ADCH << 8);
  long rawTemp = ADCW;
  double res = 0;
  res = ((rawTemp - 324.31) / 1.22) + INT_TEMP_COMP;
  if (res < -10 || res > 60)
  {
    res = prevTemp;
  }
  for (byte x = 0; x < 9; x++ ) {
    temperatureSample[x] = temperatureSample[x + 1];
  }
  temperatureSample[9] = res;
  double i = 0;
  for (byte x = 0; x < 10; x++ ) {
    i = i + temperatureSample[x];
  }
  res = i / 10;
  sensorErr=0;
  return res;
}

void SwitchOnScreen()
{
  Screen.ssd1306WriteCmd(SH1106_SET_PUMP_MODE);
  delay(1);
  Screen.ssd1306WriteCmd(SH1106_PUMP_ON);
  delay(1);
  Screen.ssd1306WriteCmd(SSD1306_DISPLAYON);
  screenSleep = false;
  sleepCt = 0;
  refreshNow = true;
  delay(1);
}

// more from author: www.ipposnif.com
