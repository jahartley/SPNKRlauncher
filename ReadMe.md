Pinout diagram inclueded for blue pill... watch out for higher voltages on 3.3v pins
For programing and debuging, plug micro USB cable into usb port for power, connect STLink ground, SWDIO, SWCLK pins to GND,DIO,DCKL on programing header. Debug messages will go over USB serial via micro USB cable


Issues:
ERROR #include <arduino.h> {THE FIX: change arduino.h to Arduino.h in .pio/libdeps/bluepill_f103c8/FlexyStepper/src/FlexyStepper.h line 37}