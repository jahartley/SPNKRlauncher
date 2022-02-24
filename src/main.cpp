/*
*     SPNKR Launcher code, STM32F103C8T6 blue pill
*     
*/

//Define Settings
#define WaitForUsbSerial        //Wait for Debug Serial connection on startup, comment to turn off

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!Only one of the below modes should be uncommented at a time!!!!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

#define final   //final is used when done with testing

//#define test1   //test1 is to get rpm and accel info by pressing trigger, self test, B4 buttons
                //press b4 to move 2 rotations... set by teeth count in stepper constants
                //press trigger to increase rpm for next run
                //press self test to increase accel for next run
                //keep prefered rpm and accel for later

//#define test2   //test 2 measures steps from index pin triggers
                //press b4 pin to spin drum 10 times, speed and accel set in test2 settings
                //note the info displayed, and use value for test 3

//#define test3 //test 3 uses trigger to find steps from index release to centered position,
              //use self test button to move to index using value from test2
              //use trigger to move trigger amount in settings till drum is in appropriate position
              //note steps from index to position for use in final...

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

#ifdef test1 //test1 settings
  #define StartRpm 10.0      //drum start rpm for test1 adjust as needed
  #define StartAccel 2000.0   //drum start accel for test1 adjust as needed
  float currentRpm = StartRpm;
  float currentAccel = StartAccel;
#endif

#ifdef test2 //test2 settings... set with data from test1
  #define StartRpm 60.0      //drum start rpm for test2 adjust as needed
  #define StartAccel 2000.0   //drum start accel for test2 adjust as needed
#endif

#ifdef test3 //test3 Settings... set with data from test1
  #define StartRpm 60.0         //drum start rpm for test3 adjust as needed
  #define StartAccel 10000.0     //drum start accel for test3 adjust as needed
  #define SlowRpm 10.0           //slow speed for precision
  #define StepsPerTrigger 20.0 //steps to move per trigger press
  #define StepsBetweenIndexRelease 1220.0 //adjust from results of test2...
  long stepsFromIndexRelease = 0;
#endif

#ifdef final //final settings
  #define StartRpm 60.0          //drum rpm for final
  #define StartAccel 10000.0      //accel rate for final
  #define StepsFromLimit 127.0      //steps from limit release till line up
  #define IndexRpm 40
  #define IndexStartAccel 8000
  #define IndexStopAccel 6000
  #define FireRpm 63
  #define FireStartAccel 20000
  #define FireStopAccel 25000
#endif

//Stepper Constants
#define DrumTeeth 366.0           //number of teeth on drum
#define StepperTeeth 60.0         //number of teeth on stepper
#define MicroStepping 2.0         //microstep setting on stepper driver
#define StepsPerRotation (DrumTeeth/StepperTeeth)*(MicroStepping*200)   //steps per drum rotation calc
#define MaxStepsPerSecond (StartRpm/60)*StepsPerRotation  //max steps per sec used not in testing

//Index pin Debounce minimum steps from first press
#define indexDebounceSteps 30

//Button Debounce routine Settings
#define debounceCount 4 //+1 checks > this number

//Pin Definitions for hookups
#define LEDPin PC13
#define TriggerPin PB6
#define ReloadPin PB7
#define IndexPin PB8
#define AnimationPin PB9
#define DebugActionPin PB4
#define StepPin PB12
#define DirectionPin PB13

//Included Librarys
#include <Arduino.h>            //Add arduino functions
#include <FlexyStepper.h>       //Add Stepper Library

//Select which serial port to use for debug messages
#define DebugSerial SerialUSB

FlexyStepper stepper;           //Create stepper object

//global variables
bool stopMotion = false;  //stops motion when carrage open
bool inMotion = false;    //prevents another function from running while another is in motion
long lastIndexRelease = 0;
long currentPosition = 0;

//function declarations required by cpp
void buttonCheck();
void IndexRoutine();
void FireRoutine();
void AnimationRoutine();
void StopMotion();
constexpr float RpmToSteps(float);
long TargetPositionRotations(float);
int CheckHolds();
int DelayPlus(int);
int IndexPlus(long, float, float, float, long);
int MovePlus(long, float, float, float);
void MeasureStepsBetweenIndexPin();


void setup() {  // startup code
  DebugSerial.begin(); //activate USB CDC driver
  #ifdef WaitForUsbSerial
    while(!DebugSerial); //blocks till usb serial is connected... turn off if not connected to PC.
  #endif
  DebugSerial.println("SPNKR Launcher starting");
  DebugSerial.print("Setup input pins...");
  pinMode(IndexPin, INPUT_PULLUP);
  pinMode(TriggerPin, INPUT_PULLUP);
  pinMode(ReloadPin, INPUT_PULLUP);
  pinMode(AnimationPin, INPUT_PULLUP);
  pinMode(DebugActionPin, INPUT_PULLUP);

  DebugSerial.println(" complete.");

  DebugSerial.print("Setup stepper library... ");
  stepper.connectToPins(StepPin, DirectionPin);
  stepper.setSpeedInStepsPerSecond(RpmToSteps(StartRpm));
  stepper.setAccelerationInStepsPerSecondPerSecond(StartAccel);
  stepper.setCurrentPositionInSteps(0);
  stepper.setTargetPositionInSteps(0);
  
  DebugSerial.println(" complete.");
  DebugSerial.print("Variables check: DrumTeeth: ");
  DebugSerial.print(DrumTeeth);
  DebugSerial.print(" StepperTeeth: ");
  DebugSerial.println(StepperTeeth);
  DebugSerial.print("Microstepping: ");
  DebugSerial.println(MicroStepping);
  DebugSerial.print("StepsPerRotation (DrumTeeth/StepperTeeth)*(MicroStepping*200): ");
  DebugSerial.println(StepsPerRotation);
  DebugSerial.print("MaxStepsPerSecond (StartRpm/60)*StepsPerRotation : ");
  DebugSerial.println(MaxStepsPerSecond);
  DebugSerial.print("StartRpm: ");
  DebugSerial.print(StartRpm);
  DebugSerial.print(" StartAccel: ");
  DebugSerial.println(StartAccel);
}

void loop() {  //Idle...
  buttonCheck();
}

void IndexRoutine(){ //routine to index drum, called when reloadPin is pressed
  stopMotion = false; //allow motion
  DebugSerial.println("LID Closed allow motion");
  #ifdef final
    //Check Holds...
    if (CheckHolds()) return;
    //Start motion
    inMotion = true; //block others till complete
    if (IndexPlus(StepsFromLimit, RpmToSteps(IndexRpm), IndexStartAccel, IndexStopAccel, 0)) {
      DebugSerial.println("Index Routine IndexPlus failed");
      return;
    }
    inMotion = false; //release hold
  #endif
}

void FireRoutine(){ //routine to switch barrels after firing
  #ifdef test1
    //Increase rpm setting
    DebugSerial.println("Fire button pressed... increase rpm by 10");
    currentRpm = currentRpm + 10;
    DebugSerial.print("CurrentRpm now at: ");
    DebugSerial.println(currentRpm);
  #endif

  #ifdef test3
    //move drum by StepsPerTrigger amount each click
    //Check Holds...
    if (CheckHolds()) return;
    //Start motion
    inMotion = true; //block others till complete
    currentPosition = stepper.getCurrentPositionInSteps();
    DebugSerial.print("Moving from: ");
    DebugSerial.print(currentPosition);
    DebugSerial.print(" to: ");
    DebugSerial.println(currentPosition + StepsPerTrigger);
    if (MovePlus(StepsPerTrigger, RpmToSteps(StartRpm), StartAccel, StartAccel)) {
      DebugSerial.println("Fire Routine MovePlus failed");
      return;
    }
    DebugSerial.println("Done...");
    currentPosition = stepper.getCurrentPositionInSteps();
    DebugSerial.print("Steps from last index is: ");
    DebugSerial.println(currentPosition - stepsFromIndexRelease);
    DebugSerial.println("average this value after a few tries for use in final StepsFromLimit");
    inMotion = false; //release hold
  #endif

  #ifdef final
  //normal operation of fire button
  //Check Holds...
  if (CheckHolds()) return;
  //Start motion
  inMotion = true; //block others till complete
  if (IndexPlus(StepsFromLimit, RpmToSteps(FireRpm), FireStartAccel, FireStopAccel, 0)) {
    DebugSerial.println("Fire Routine IndexPlus failed");
    return;
  }
  inMotion = false; //release hold
  #endif
}

void AnimationRoutine(){ //routine to perform animation
  #ifdef test1
  DebugSerial.println("animation button pressed... increase accel by 100");
  currentAccel = currentAccel + 100;
  DebugSerial.print("CurrentAccel now at: ");
  DebugSerial.println(currentAccel);
  #endif

  #ifdef test3
    //move drum till index relase, then stop at next release...
    //Check Holds...
    if (CheckHolds()) return;
    //Start motion
    inMotion = true; //block others till complete
    stepper.setTargetPositionInSteps(TargetPositionRotations(1.0));
    int indexSet = 0;
    int keepGoing = 1;
    long lastIndexPress = 0;
    long lastIndexRelease = 0;
    while(keepGoing){
    while(!stepper.motionComplete()){
      stepper.processMovement();
      currentPosition = stepper.getCurrentPositionInSteps();
      if (digitalRead(IndexPin)) { //pin released
        if (indexSet == 1) {
          if (currentPosition - lastIndexPress > indexDebounceSteps) {
          //was pressed but now released
          stepper.setTargetPositionInSteps(currentPosition + StepsBetweenIndexRelease - 200);
          indexSet = 2; //look for next press
          lastIndexRelease = currentPosition;
          }
        }
        if (indexSet == 3) {
          if (currentPosition - lastIndexPress > indexDebounceSteps) {
            indexSet = 4;
            lastIndexRelease = currentPosition;
          }
        }
      } else { //pin pressed
        if (indexSet == 0) {
          //pin now pressed...
          indexSet = 1;
          lastIndexPress = currentPosition;
        }
        if (indexSet == 2) {
          //pin now pressed...
          indexSet = 3;
          lastIndexPress = currentPosition;
        }
        
      }
      if (stopMotion) {
        stepper.setTargetPositionToStop();
        DebugSerial.println("test3 animate Error stopMotion stopping");
      }
      buttonCheck();
    }

    if (indexSet == 4) { 
      keepGoing = 0;
      stepper.setSpeedInStepsPerSecond(RpmToSteps(StartRpm));
    } else {
      stepper.setSpeedInStepsPerSecond(RpmToSteps(SlowRpm));
      stepper.setTargetPositionInSteps(stepper.getCurrentPositionInSteps() + 10);
    }
    
    }
    stepsFromIndexRelease = stepper.getCurrentPositionInSteps();
    DebugSerial.print("Slow home to index pin release finished now at: ");
    DebugSerial.println(stepsFromIndexRelease);

    inMotion = false; //release hold
  #endif

  #ifdef final
  //Check Holds...
  if (CheckHolds()) return;
  //Start motion
  inMotion = true; //block others till complete
  //code here for animation

  DebugSerial.println("Animation #1");
  //#1
  if (MovePlus(TargetPositionRotations(-0.0625), RpmToSteps(30), 10000, 10000)) {
    DebugSerial.println("Animation Routine #1 failed");
    return;
  }
  //#2
  DebugSerial.println("Animation #2");
  DelayPlus(250);
  //#3
  DebugSerial.println("Animation #3");
  if (IndexPlus(TargetPositionRotations(0.0625), RpmToSteps(30), 10000, 10000, 700)) {
    DebugSerial.println("Animation Routine #3 IndexPlus failed");
    return;
  }
  //#4
  DebugSerial.println("Animation #4");
  DelayPlus(250);
  //#5
  DebugSerial.println("Animation #5");
  if (MovePlus(lastIndexRelease + StepsFromLimit, RpmToSteps(30), 10000, 10000)) {
    DebugSerial.println("Animation Routine #5 failed");
    return;
  }
  //#6
  DebugSerial.println("Animation #6");
  DelayPlus(250);
  //#7
  DebugSerial.println("Animation #7");
  if (IndexPlus(TargetPositionRotations(0.0625), RpmToSteps(30), 10000, 10000, 700)) {
    DebugSerial.println("Animation Routine #7 IndexPlus failed");
    return;
  }
  //#8
  DebugSerial.println("Animation #8");
  DelayPlus(250);
  //#9
  DebugSerial.println("Animation #9");
  if (MovePlus(lastIndexRelease + StepsFromLimit, RpmToSteps(30), 10000, 10000)) {
    DebugSerial.println("Animation Routine #9 failed");
    return;
  }
  DebugSerial.println("Animation Complete");
  inMotion = false; //release hold
  #endif
}

void StopMotion(){ //called when reload pin is released
  stopMotion = true; //Stop motion...
  stepper.setTargetPositionToStop(); //stop motion...
  DebugSerial.println("LID OPENED STOP IMMEADATLEY>>>");
  inMotion = false; //release hold
}


void MeasureStepsBetweenIndexPin(){ //debug routine to show steps between index hits via serial output
  //Check Holds...
  if (CheckHolds()) return;

  #ifdef test2 //test2 code
  //Variables...
  long currentPosition = 0;
  long lastIndexPress = 0;
  long lastIndexRelease = 0;
  long lastFireCircuitPress = 0;
  long lastFireCircuitRelease = 0;
  int indexSet = 0;
  int countPress = 0;
  DebugSerial.println("Measure steps for pins routine starting");
  // Start motion
  inMotion = true; //block others till complete
  //check index pin and DebugFireCircuit pin for measurements
  stepper.setCurrentPositionInSteps(currentPosition);
  stepper.setTargetPositionInSteps(TargetPositionRotations(10));
  while(!stepper.motionComplete()){
    stepper.processMovement();
    currentPosition = stepper.getCurrentPositionInSteps();
    // Index pin checking... 1 is released, 0 pressed.
    // if released and was released, do nothing...
    // if pressed and was released, msg position, indexSet = 0
    // if pressed and was pressed do nothing,
    // if released and was pressed, msg position, indexSet = 1
    if (digitalRead(IndexPin)) { //pin released
      if (indexSet == 1) {
        //was pressed but now released
        if (currentPosition - lastIndexPress > indexDebounceSteps) {
          long diff = currentPosition - lastIndexPress;
          long diff2 = currentPosition - lastIndexRelease;
          DebugSerial.print("Index Released. Steps since last press: ");
          DebugSerial.print(diff);
          DebugSerial.print(" and since last release: ");
          DebugSerial.println(diff2);
          lastIndexRelease = currentPosition;
          indexSet = 0;
          countPress++;
        }
      }
    } else { //pin pressed
      if (indexSet == 0) {
        //pin now pressed...
        if (currentPosition - lastIndexRelease > indexDebounceSteps) {
          long diff = currentPosition - lastIndexPress;
          long diff2 = currentPosition - lastIndexRelease;
          DebugSerial.print("New Press group count is: ");
          DebugSerial.print(countPress);
          DebugSerial.println(" ----------------------------------");
          DebugSerial.print("Index Pressed. Steps since last press: ");
          DebugSerial.print(diff);
          DebugSerial.print(" and since last release: ");
          DebugSerial.println(diff2);
          lastIndexPress = currentPosition;
          indexSet = 1;
        }
      }
    }
 
    buttonCheck();
  }
  #endif

  #ifdef test1 //test 1 code
  // Start motion
  inMotion = true; //block others till complete
  //rotate drum 2 rotations and stop...
  DebugSerial.println("Rotate 2 times then stop with current accel and rpm");
  DebugSerial.print("Current Accel value: ");
  DebugSerial.println(currentAccel);
  DebugSerial.print("Current RPM value: ");
  DebugSerial.println(currentRpm);
  if (MovePlus(TargetPositionRotations(2), RpmToSteps(currentRpm), currentAccel, currentAccel)) {
    DebugSerial.println("MeasureSteps MovePlus failed");
    return;
  }
  #endif
  DebugSerial.println("Movement complete");
  inMotion = false; //release hold
}


constexpr float RpmToSteps(float rpm) {
  return (rpm/60.0)*StepsPerRotation;
}

long TargetPositionRotations(float rotations) {
  currentPosition = stepper.getCurrentPositionInSteps();
  return currentPosition + (StepsPerRotation * rotations);
}


//4 buttons round robin 40ms debounce
//check each 5 times on a 2ms interval rr through the 4 buttons
//need state, counter, rotation counter

//Global Variables for Button Debounce
int b1counter = 0;
int b2counter = 0;
int b3counter = 0;
int b4counter = 0;
int b1state = 1;
int b2state = 1;
int b3state = 1;
int b4state = 1;
int b1flag = 0;
int b2flag = 0;
int b3flag = 0;
int b4flag = 0;

int rrCounter = 0;

unsigned long lastTime = 0;
unsigned long now = 0;

//Button Debounce Routine
void buttonCheck() {
  now = millis();
  if (now - lastTime < 2) return; // stop if less than 2ms gone by
  lastTime = now;
  switch (rrCounter) {
    case 0:
      /* code */
      if (digitalRead(TriggerPin)) {
        if (b1state == 0) b1counter++;
        if (b1state == 1) b1counter = 0;
        if (b1counter > debounceCount) {
          b1counter = 0;
          b1state = 1;
          b1flag = 1;
        }
      } else {
        if (b1state == 1) b1counter++;
        if (b1state == 0) b1counter = 0;
        if (b1counter > debounceCount) {
          b1counter = 0;
          b1state = 0;
          b1flag = 2;
        }
      }
      rrCounter++;
    break;
  
    case 1:
      /* code */
      if (digitalRead(ReloadPin)) {
        if (b2state == 0) b2counter++;
        if (b2state == 1) b2counter = 0;
        if (b2counter > debounceCount) {
          b2counter = 0;
          b2state = 1;
          b2flag = 1;
        }
      } else {
        if (b2state == 1) b2counter++;
        if (b1state == 0) b2counter = 0;
        if (b2counter > debounceCount) {
          b2counter = 0;
          b2state = 0;
          b2flag = 2;
        }
      }
      rrCounter++;
    break;
  
    case 2:
      /* code */
      if (digitalRead(AnimationPin)) {
        if (b3state == 0) b3counter++;
        if (b3state == 1) b3counter = 0;
        if (b3counter > debounceCount) {
          b3counter = 0;
          b3state = 1;
          b3flag = 1;
        }
      } else {
        if (b3state == 1) b3counter++;
        if (b3state == 0) b3counter = 0;
        if (b3counter > debounceCount) {
          b3counter = 0;
          b3state = 0;
          b3flag = 2;
        }
      }
      rrCounter++;
    break;

    case 3:
      /* code */
      if (digitalRead(DebugActionPin)) {
        if (b4state == 0) b4counter++;
        if (b4state == 1) b4counter = 0;
        if (b4counter > debounceCount) {
          b4counter = 0;
          b4state = 1;
          b4flag = 1;
        }
      } else {
        if (b4state == 1) b4counter++;
        if (b4state == 0) b4counter = 0;
        if (b4counter > debounceCount) {
          b4counter = 0;
          b4state = 0;
          b4flag = 2;
        }
      }
      rrCounter = 0;
    break;
  }
  //flag 1 = pin released
  //flag 2 = pin pressed
  if (b1flag == 1) {
    b1flag = 0;
    DebugSerial.println("Trigger Released");
    FireRoutine();
  }
  if (b1flag == 2) {
    b1flag = 0;
    DebugSerial.println("Trigger Pressed");
  }
  if (b2flag == 1) {
    b2flag = 0;
    DebugSerial.println("Reload Released");
    StopMotion();
  }
  if (b2flag == 2) {
    b2flag = 0;
    DebugSerial.println("Reload Pressed");
    IndexRoutine();
  }
  if (b3flag == 1) {
    b3flag = 0;
    DebugSerial.println("Animate Released");
    AnimationRoutine();
  }
  if (b3flag == 2) {
    b3flag = 0;
    DebugSerial.println("Animate Pressed");
  }
    if (b4flag == 1) {
    b4flag = 0;
    DebugSerial.println("Debug Released");
    MeasureStepsBetweenIndexPin();
  }
  if (b4flag == 2) {
    b4flag = 0;
    DebugSerial.println("Debug Pressed");
  }

}

//Move ignoring Index pin
//returns 0 if successful, 1 for fail
int MovePlus(long dist, float sps, float startA, float stopA) {
  currentPosition = stepper.getCurrentPositionInSteps();
  stepper.setSpeedInStepsPerSecond(sps);
  stepper.setAccelerationInStepsPerSecondPerSecond(startA);
  stepper.setTargetPositionInSteps(currentPosition + dist);
  int ret = 0;
  while(!stepper.motionComplete()){
    stepper.processMovement();
    if (stepper.getCurrentVelocityInStepsPerSecond() == sps) {
      stepper.setAccelerationInStepsPerSecondPerSecond(stopA);
    }
    if (stopMotion) {
        stepper.setTargetPositionToStop();
        DebugSerial.println("Error stopMotion stopping");
        ret = 1;
    }
    buttonCheck();
  }
  return ret;
}

//Move using Index Pin
//returns 0 if successful, 1 for fail
int IndexPlus(long extra, float sps, float startA, float stopA, long minimum) {
  currentPosition = stepper.getCurrentPositionInSteps();
  stepper.setTargetPositionInSteps(currentPosition + (StepsPerRotation * 2));
  stepper.setSpeedInStepsPerSecond(sps);
  stepper.setAccelerationInStepsPerSecondPerSecond(startA);
  long minDistance = currentPosition + minimum;
  long startPosition = currentPosition;
  int indexSet = 0;
  int keepGoing = 1;
  long lastIndexPress = 0;
  int ret = 0;
  DebugSerial.println("IndexPlus routine");
  while(keepGoing){
    while(!stepper.motionComplete()){
      stepper.processMovement();
      currentPosition = stepper.getCurrentPositionInSteps();
      if (digitalRead(IndexPin)) { //pin released
        if (indexSet == 1) {
          //DebugSerial.println("First Index release detected...");
          if (currentPosition - lastIndexPress > indexDebounceSteps) {
          //was pressed but now released
          DebugSerial.println("#1 First Index release debounce ok");
          //DebugSerial.println(currentPosition);
          lastIndexRelease = currentPosition;
          stepper.setTargetPositionInSteps(currentPosition + extra);
          indexSet = 3; //end routine
          }
        }
      } else { //pin pressed
        if (indexSet == 0) {
          //pin now pressed...
          if (currentPosition - minDistance > 0) {
            DebugSerial.println("#0 First index press detected...");
            DebugSerial.println(currentPosition);
            stepper.setAccelerationInStepsPerSecondPerSecond(stopA);
            indexSet = 1;
            lastIndexPress = currentPosition;
          } else {
            DebugSerial.println("Index press detected but minimum travel not met...");
            DebugSerial.print("Distance start to hit: ");
            DebugSerial.println(currentPosition-startPosition);
          }
        }
      }
      if (stopMotion) {
        stepper.setTargetPositionToStop();
        DebugSerial.println("Error stopMotion stopping");
        ret = 1;
      }
      buttonCheck();
    } //end proccess movement

    if (indexSet == 0) {
      DebugSerial.println("ERROR Failed to detect index pin...");
      keepGoing = 0;
      ret = 1;
    }

    if (indexSet == 3) {
      DebugSerial.println("Done, setting normal speeds.");
      DebugSerial.println(currentPosition);
      keepGoing = 0;
    }

  }
  stepper.setSpeedInStepsPerSecond(RpmToSteps(StartRpm));
  stepper.setAccelerationInStepsPerSecondPerSecond(StartAccel);
  stepper.setCurrentPositionInSteps(0);
  return ret; 
}

//Delay function while still checking buttons
//returns 0 if successful, 1 for fail
int DelayPlus(int delay) {
  unsigned long now = millis();
  unsigned long ending = now + delay;
  while(true) {
    if (millis() - ending > 0) {
      break;
    }
    buttonCheck();
  }
  return 0;
}

int CheckHolds(){ //returns 1 if holds, 0 if none
  //Check Holds...
  int holds = 0;
  if (stopMotion) { //if motion not allowed do nothing
    DebugSerial.println("Error stopMotion blocking");
    holds = 1;
  } 
  if (inMotion) { //if already inMotion cant move...
    DebugSerial.println("Error already in motion");
    holds = 1;
  }
  return holds;
}

//The END!