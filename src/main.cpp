/*
*     SPNKR Launcher code, STM32F103C8T6 blue pill
*     
*/

//Define Settings
#define DebugStepper            //turn on debug stepper functions, comment out to turn off
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
  long currentPosition = 0;
#endif

#ifdef final //final settings
  #define StartRpm 60.0          //drum rpm for final
  #define StartAccel 10000.0      //accel rate for final
  #define SlowRpm 10.0           //slow speed for precision
  #define StepsFromLimit 120.0      //steps from limit release till line up
  #define StepsBetweenIndexRelease 1220.0 //adjust from results of test2...
  #define HalfTurn 1240.0
  long currentPosition = 0;
  long animationStopPosition = 0;
#endif

//Stepper Constants
#define DrumTeeth 366.0           //number of teeth on drum
#define StepperTeeth 60.0         //number of teeth on stepper
#define MicroStepping 2.0         //microstep setting on stepper driver
#define Reverse false           //not used yet
#define StepsPerRotation (DrumTeeth/StepperTeeth)*(MicroStepping*200)   //steps per drum rotation calc
#define MaxStepsPerSecond (StartRpm/60)*StepsPerRotation  //max steps per sec used not in testing
#define indexDebounceSteps 30

//Pin Definitions for hookups
#define LEDPin PC13
#define TriggerPin PB6
#define ReloadPin PB7
#define IndexPin PB8
#define AnimationPin PB9
#define StepPin PB12
#define DirectionPin PB13

#ifdef DebugStepper
  //#define DebugStepPin PB3
  //#define DebugFireCircuit PB15
  #define DebugMeasureStepsBetweenIndexPin PB4
#endif

//Button King press timer wait time
#define WaitTime 10


#include <Arduino.h>            //Add arduino functions
#include <FlexyStepper.h>       //Add Stepper Library
#include <ButtonKing.h>         //Add button king debounce
#define DebugSerial SerialUSB   //Pick which serial port to use for debug messages

//buttons
ButtonKing trigger(TriggerPin, true, true);
ButtonKing reload(ReloadPin, true, true);
//ButtonKing index(IndexPin, true, true);
ButtonKing animation(AnimationPin, true, true);
#ifdef DebugStepper
  ButtonKing debugMeasure(DebugMeasureStepsBetweenIndexPin, true,true);
#endif


FlexyStepper stepper;           //Create stepper object

//globl variables
bool stopMotion = false;  //stops motion when carrage open
bool inMotion = false;    //prevents another function from running while another is in motion
int buttonRoundRobin = 0; //used by button round robin function to speed up stepper motion

#ifdef DebugStepper
  int buttonCount = 4;
#else
  int buttonCount = 3;
#endif

//function declarations required by cpp
void buttonCheck();
void buttonCheckRR();
void IndexRoutine();
void FireRoutine();
void AnimationRoutine();
void StopMotion();
float RpmToSteps(float);
long TargetPositionRotations(long, float);
#ifdef DebugStepper
  void MeasureStepsBetweenIndexPin();
#endif
#ifdef test2
// This function will be called when the button1 was clicked.
void click1();
void doubleclick1();
void tripleclick1();
void press1();
void release1();
void doublepress1();
void triplepress1();
#endif


void setup() {  // startup code
  DebugSerial.begin(); //activate USB CDC driver
  #ifdef WaitForUsbSerial
    while(!DebugSerial); //blocks till usb serial is connected... turn off if not connected to PC.
  #endif
  DebugSerial.println("SPNKR Launcher starting");
  DebugSerial.print("Setup input pins...");
  pinMode(IndexPin, INPUT_PULLUP);
  //setup press/release wait timers
  trigger.setTimeCount(WaitTime);
  trigger.setTimeDebounce(50);
  animation.setTimeCount(WaitTime);
  reload.setTimeCount(500);
  reload.setTimeDebounce(50);
  //setup button functions
  trigger.setRelease(FireRoutine);
  #ifdef test2
    trigger.setClick(click1);
    trigger.setDoubleClick(doubleclick1);
    trigger.setTripleClick(tripleclick1);
    trigger.setPress(press1);
    trigger.setRelease(release1);
    trigger.setDoublePress(doublepress1);
    trigger.setTriplePress(triplepress1);
  #endif
  animation.setRelease(AnimationRoutine);
  reload.setPress(IndexRoutine);
  reload.setRelease(StopMotion);
  #ifdef DebugStepper
    //pinMode(DebugStepPin, INPUT_PULLUP);
    //pinMode(DebugFireCircuit, INPUT_PULLUP);
    debugMeasure.setTimeCount(WaitTime);
    debugMeasure.setRelease(MeasureStepsBetweenIndexPin);
  #endif
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

void loop() {  // put your main code here, to run repeatedly:
  buttonCheck();
  //stepper.processMovement();
}


void buttonCheck(){ //checks all buttons
  //code for button inputs to call routines...
  trigger.isClick();
  reload.isClick();
  //index.isClick();
  animation.isClick();
  #ifdef DebugStepper
    debugMeasure.isClick();
  #endif
}

void buttonCheckRR(){ //checks one button only per call, in round robin
  switch(buttonRoundRobin){
    case 0: {
      trigger.isClick();
      buttonRoundRobin++;
      break;
    }
    case 1: {
      reload.isClick();
      buttonRoundRobin++;
      break;
    }
    case 2: {
      animation.isClick();
      buttonRoundRobin++;
      break;
    }
    case 3: {
      #ifdef DebugStepper
        debugMeasure.isClick();
        buttonRoundRobin = 0;
        break;
      #else
        buttonRoundRobin = 0;
        break;
      #endif
    }
  }
}

void IndexRoutine(){ //routine to index drum, called when reloadPin is pressed
  stopMotion = false; //allow motion
  DebugSerial.println("LID Closed allow motion");
  #ifdef final
    //code here for index routine
    //move drum till index relase, then stop at next release...
    //Check Holds...
    if (stopMotion) { //if motion not allowed do nothing
      DebugSerial.println("Error stopMotion blocking");
      return;
    } 
    if (inMotion) { //if already inMotion cant move...
      DebugSerial.println("Error already in motion");
      return;
    }
    //Start motion
    inMotion = true; //block others till complete
    currentPosition = stepper.getCurrentPositionInSteps();
    stepper.setTargetPositionInSteps(TargetPositionRotations(currentPosition, 1.0));
    int indexSet = 0;
    int keepGoing = 1;
    long lastIndexPress = 0;
    long lastIndexRelease = 0;
    while(keepGoing){
      DebugSerial.print("KeepGoing indexSet: ");
      DebugSerial.println(indexSet);
    while(!stepper.motionComplete()){
      stepper.processMovement();
      currentPosition = stepper.getCurrentPositionInSteps();
      if (digitalRead(IndexPin)) { //pin released
        if (indexSet == 1) {
          if (currentPosition - lastIndexPress > indexDebounceSteps) {
          //was pressed but now released
          stepper.setTargetPositionInSteps(currentPosition + StepsBetweenIndexRelease - 400);
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
      //if (stopMotion) {
      //  stepper.setTargetPositionToStop();
      //  DebugSerial.println("Error stopMotion stopping");
      //}
      //buttonCheckRR();
    }

    if (indexSet == 5) {
      keepGoing = 0;
      stepper.setSpeedInStepsPerSecond(RpmToSteps(StartRpm));
      stepper.setCurrentPositionInSteps(0);
    }

    if (indexSet < 4) {
      stepper.setSpeedInStepsPerSecond(RpmToSteps(SlowRpm));
      stepper.setTargetPositionInSteps(stepper.getCurrentPositionInSteps() + 10);
    }

    if (indexSet == 4) { 
      stepper.setTargetPositionInSteps(stepper.getCurrentPositionInSteps() + StepsFromLimit);
      indexSet = 5;
    } 

    
    }
    DebugSerial.println("Show TIME!!");
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
    if (stopMotion) { //if motion not allowed do nothing
      DebugSerial.println("test3 FireRoutine Error stopMotion blocking");
      return;
    } 
    if (inMotion) { //if already inMotion cant move...
      DebugSerial.println("test3 FireRoutine Error already in motion");
      return;
    }
    //Start motion
    inMotion = true; //block others till complete
    currentPosition = stepper.getCurrentPositionInSteps();
    stepper.setTargetPositionInSteps(currentPosition + StepsPerTrigger);
    DebugSerial.print("Moving from: ");
    DebugSerial.print(currentPosition);
    DebugSerial.print(" to: ");
    DebugSerial.println(currentPosition + StepsPerTrigger);
    while(!stepper.motionComplete()){
      stepper.processMovement();
      if (stopMotion) {
        stepper.setTargetPositionToStop();
        DebugSerial.println("stopped by stop motion");
      }
      buttonCheckRR();
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
  if (stopMotion) { //if motion not allowed do nothing
    DebugSerial.println("FireRoutine Error stopMotion blocking");
    return;
  } 
  if (inMotion) { //if already inMotion cant move...
    DebugSerial.println("FireRoutine Error already in motion");
    return;
  }
  //Start motion
  inMotion = true; //block others till complete
  currentPosition = stepper.getCurrentPositionInSteps();
  stepper.setTargetPositionInSteps(currentPosition + HalfTurn);
  while(!stepper.motionComplete()){
    stepper.processMovement();
    if (stopMotion) {
      stepper.setTargetPositionToStop();
    }
    buttonCheckRR();
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
    if (stopMotion) { //if motion not allowed do nothing
      DebugSerial.println("test3 animate Error stopMotion blocking");
      return;
    } 
    if (inMotion) { //if already inMotion cant move...
      DebugSerial.println("test3 animate Error already in motion");
      return;
    }
    //Start motion
    inMotion = true; //block others till complete
    currentPosition = stepper.getCurrentPositionInSteps();
    stepper.setTargetPositionInSteps(TargetPositionRotations(currentPosition, 1.0));
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
      buttonCheckRR();
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
  if (stopMotion) { //if motion not allowed do nothing
    DebugSerial.println("AnimationRoutine Error stopMotion blocking");
    return;
  } 
  if (inMotion) { //if already inMotion cant move...
    DebugSerial.println("AnimationRoutine Error already in motion");
    return;
  }
  
  //Start motion
  inMotion = true; //block others till complete
  //code here for animation
  animationStopPosition = stepper.getCurrentPositionInSteps();
  int animationStep = 0;
  int keepGoing = 1;
  stepper.setSpeedInStepsPerSecond(RpmToSteps(20));
  stepper.setAccelerationInStepsPerSecondPerSecond(2000);
  while(keepGoing){
    while(!stepper.motionComplete()){
      stepper.processMovement();
      if (stopMotion) {
        stepper.setTargetPositionToStop();
      }
      buttonCheckRR();
    }
    //place all animation positions from last to first...
    currentPosition = stepper.getCurrentPositionInSteps();
    if (animationStep == 6) {
      stepper.setSpeedInStepsPerSecond(RpmToSteps(StartRpm));
      stepper.setAccelerationInStepsPerSecondPerSecond(StartAccel);
      keepGoing = 0;
    }
    if (animationStep == 5) {
      stepper.setTargetPositionInSteps(animationStopPosition);
      animationStep = 6;
    }
    if (animationStep == 4) {
      stepper.setTargetPositionInSteps(animationStopPosition + HalfTurn + HalfTurn);
      animationStep = 6;
    }
    if (animationStep == 3) {
      stepper.setTargetPositionInSteps(TargetPositionRotations(currentPosition, 0.6));
      animationStep = 4;
    }
    if (animationStep == 2) {
      stepper.setTargetPositionInSteps(animationStopPosition + HalfTurn);
      animationStep = 3;
    }
    if (animationStep == 1) {
      stepper.setTargetPositionInSteps(TargetPositionRotations(currentPosition, 0.6));
      animationStep = 2;
    }
    if (animationStep == 0) {
      stepper.setTargetPositionInSteps(TargetPositionRotations(currentPosition, -0.028));
      animationStep = 1;
    }
  }
  inMotion = false; //release hold
  #endif
}

void StopMotion(){ //called when reload pin is released
  //stopMotion = true; //Stop motion...
  //stepper.setTargetPositionToStop(); //stop motion...
  DebugSerial.println("LID OPENED STOP IMMEADATLEY>>>");
}

#ifdef DebugStepper

void MeasureStepsBetweenIndexPin(){ //debug routine to show steps between index hits via serial output
  //Check Holds...
  if (stopMotion) { //if motion not allowed do nothing
    DebugSerial.println("MeasureStepsBetween Error stopMotion blocking");
    return;
  } 
  if (inMotion) { //if already inMotion cant move...
    DebugSerial.println("MeasureStepsBetween Error already in motion");
    return;
  }

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
  stepper.setTargetPositionInSteps(TargetPositionRotations(currentPosition, 10));
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
 
    buttonCheckRR();
  }
  #endif

  #ifdef test1 //test 1 code
  // Start motion
  inMotion = true; //block others till complete
  //rotate drum 2 rotations and stop...
  DebugSerial.println("Rotate 2 times then stop with current accel and rpm");
  long currentPos = stepper.getCurrentPositionInSteps();
  DebugSerial.print("Current Accel value: ");
  DebugSerial.println(currentAccel);
  DebugSerial.print("Current RPM value: ");
  DebugSerial.println(currentRpm);
  stepper.setAccelerationInStepsPerSecondPerSecond(currentAccel);
  stepper.setSpeedInStepsPerSecond(RpmToSteps(currentRpm));
  stepper.setTargetPositionInSteps(TargetPositionRotations(currentPos, 2));
  DebugSerial.println("Starting movement");
  while(!stepper.motionComplete()){
    stepper.processMovement();
    if (stopMotion) {
      stepper.setTargetPositionToStop();
      DebugSerial.println("Motion stopped by stop motion...");
    }
    buttonCheckRR();
  }
  #endif
  DebugSerial.println("Movement complete");
  inMotion = false; //release hold
}
#endif

float RpmToSteps(float rpm) {
  return (rpm/60.0)*StepsPerRotation;
}

long TargetPositionRotations(long currentPosition, float rotations) {
  return currentPosition + (StepsPerRotation * rotations);
}

#ifdef test2
// This function will be called when the button1 was clicked.
void click1() {
  Serial.println("Button 1 click.");
} // click1

// This function will be called when the button1 was double-clicked.
void doubleclick1() {
  Serial.println("Button 1 doubleclick.");
} // doubleclick1

// This function will be called when the button1 was triple-clicked.
void tripleclick1() {
  Serial.println("Button 1 tripleclick.");
} // tripleclick1

// This function will be called when the button1 was pressed.
void press1() {
  Serial.println("Button 1 press.");
} // press1

// This function will be called when the button1 was released..
void release1() {
  Serial.println("Button 1 release.");
  Serial.println( trigger.getPressedTimer() );
} // release1

// This function will be called when the button1 was double-pressed.
void doublepress1() {
  Serial.println("Button 1 doublePRESS.");
} // doublepress1

// This function will be called when the button1 was triple-pressed.
void triplepress1() {
  Serial.println("Button 1 triplePRESS.");
} // triplepress1

#endif