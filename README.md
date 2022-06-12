# Sunfy328
Sunfy328 is an open source irrigation control unit based on the ATMega328 microcontroller.  
From this repository you can download the latest version of the software.
		
The hardware, the user's manual, and all the needed instructions are freely available from:  
http://www.picapot.com

The code has been developed using the Arduino programming environment.  
https://www.arduino.cc/  


**Files Description** 
* Sunfy328.ino (Sunfy328 source code)	
* Sunfy328.ino.hex (Sunfy328 compiled program working with a 128x64 SPI display SSD1306/SSD1309)	
* avrdude.conf (AVRdude configuration file)
* avrdude.exe (command program to manage AVR memory)
* upload.bat (script to upload the program with an USBTinyISP programmer)

**Dependencies** 	
* I2C library - https://github.com/esp8266/Arduino/blob/master/libraries/Wire/Wire.h
* 1-WIRE library - https://github.com/PaulStoffregen/OneWire	
* Dusk2Dawn library - https://github.com/Ipposnif/Dusk2Dawn (fork of https://github.com/dmkishi/Dusk2Dawn)
* Screen library - https://github.com/Ipposnif/SSD1306Ascii (fork of https://github.com/greiman/SSD1306Ascii)
