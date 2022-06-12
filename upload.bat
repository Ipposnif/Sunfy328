cd %CD%
avrdude  -c usbtiny -p atmega328p  -b 300 -B 100 -F -e -u -v -s -U lock:w:0x3f:m  -U lfuse:w:0xEF:m -U hfuse:w:0xCF:m -U efuse:w:0xFD:m -U lock:w:0xFF:m
avrdude  -c usbtiny -p atmega328p  -b 9600 -B 1 -V -F -e -u -v -s -U lock:w:0x3f:m  -U flash:w:Sunfy328.ino.hex:i  -U lock:w:0x00:m
pause

