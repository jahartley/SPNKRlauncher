Pinout diagram inclueded for blue pill... watch out for higher voltages on 3.3v pins
For programing and debuging, plug micro USB cable into usb port for power, connect STLink ground, SWDIO, SWCLK pins to GND,DIO,DCKL on programing header. Debug messages will go over USB serial via micro USB cable


Issues:
ERROR #include <arduino.h> {THE FIX: change arduino.h to Arduino.h in .pio/libdeps/bluepill_f103c8/FlexyStepper/src/FlexyStepper.h line 37}


TEST1 check list
    verify serial output messages in vs code on startup
    with lid unlocked press b4 and verify no motion and a message in serial
    close lid... verify message in serial
    press b4, verify 2 rotations
    press b4, when drum starts turning open lid and verify messages and stop
    close lid
    press b4, wait for stop
    press and hold trigger, verifiy that nothing happens
    release trigger, verify messages then press b4 (increase speed)
    press and hold self test, verifiy that nothing happens
    release self test, verifiy messages then press b4 (increases accel)

    write down speeds that look good and test for max accel that works.
    update code StartRpm and StartAccel values in test2, test3, and final

TEST 2 check list
    after startup complete, press b4 and wait for 10 revolutions.
    scroll through serial info, should have 20 sets of info from index pin
    get average value between pin releases to see if they are consistant
    use this info to set test3 StepsBetweenIndex release with lowest reasonable value

TEST 3 check list
    after startup, use self test button to move drum till the index has just released.
    press trigger till drum is in the correct firing position, note value from msg.
    if drum moves too far on each trigger press, adjust test3 StepsPerTrigger value and reupload
    repeat a few times to find correct value for final StepsFromLimit...
