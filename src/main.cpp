/*
*     SPNKR Launcher code, STM32F103C8T6 blue pill
*     
*/

//Define Settings
#define DebugSerial SerialUSB   //Pick which serial port to use for debug messages
#define DebugStepper            //turn on debug stepper functions, comment out to turn off
#define WaitForUsbSerial        //Wait for Debug Serial connection on startup, comment to turn off

//Pin Definitions for hookups
#define LEDPin PC13
#define TriggerPin PB6
#define ReloadPin PB7
#define IndexPin PB8
#define AnimationPin PB9
#define StepPin PB12
#define DirectionPin PB13

#ifdef DebugStepper
  #define DebugStepPin PB3
  #define DebugFireCircuit PB15
  #define DebugMeasureStepsBetweenIndexPin PB4
  #define DebugStartRpm 10
  #define DebugStartAccel 100
#endif

//Stepper Constants
#define MaxAccel 10000
#define MaxDrumRpm 500
#define DrumTeeth 370
#define StepperTeeth 60
#define MicroStepping 2
#define Reverse false
#define StepsPerRotation (DrumTeeth/StepperTeeth)*(MicroStepping*200)
#define MaxStepsPerSecond (MaxDrumRpm/60)*StepsPerRotation


#include <Arduino.h>            //Add arduino functions
#include <FlexyStepper.h>       //Add Stepper Library
#include <ButtonKing.h>         //Add button king debounce

//buttons
ButtonKing trigger(TriggerPin, true, true);
ButtonKing reload(ReloadPin, true, true);
//ButtonKing index(IndexPin, true, true);
ButtonKing animation(AnimationPin, true, true);
#ifdef DebugStepper
  ButtonKing debugMeasure(DebugMeasureStepsBetweenIndexPin, true,true);
#endif


FlexyStepper stepper;           //Create stepper object

//variables
bool stopMotion = false;
bool inMotion = false;
int buttonRoundRobin = 0;

#ifdef DebugStepper
  int buttonCount = 4;
  int currentRpm = DebugStartRpm;
  int currentAccel = DebugStartAccel;
#else
  int buttonCount = 3;
#endif

void buttonCheck();
void buttonCheckRR();
void IndexRoutine();
void FireRoutine();
void AnimationRoutine();
void StopMotion();
float RpmToSteps(int);
long TargetPositionRotations(long, float);
#ifdef DebugStepper
  void MeasureStepsBetweenIndexPin();
#endif


void setup() {  // put your setup code here, to run once:
  DebugSerial.begin(); //activate USB CDC driver
  #ifdef WaitForUsbSerial
    while(!DebugSerial); //blocks till usb serial is connected... turn off if not connected to PC.
  #endif
  DebugSerial.println("SPNKR Launcher starting");
  DebugSerial.print("Setup input pins...");
  pinMode(IndexPin, INPUT_PULLUP);
  trigger.setRelease(FireRoutine);
  animation.setRelease(AnimationRoutine);
  reload.setPress(IndexRoutine);
  reload.setRelease(StopMotion);
  #ifdef DebugStepper
    pinMode(DebugStepPin, INPUT_PULLUP);
    pinMode(DebugFireCircuit, INPUT_PULLUP);
    debugMeasure.setRelease(MeasureStepsBetweenIndexPin);
  #endif
  DebugSerial.println(" complete.");

  DebugSerial.print("Setup stepper library... ");
  stepper.connectToPins(StepPin, DirectionPin);
  #ifdef DebugStepper
  stepper.setSpeedInStepsPerSecond(RpmToSteps(DebugStartRpm));
  stepper.setAccelerationInStepsPerSecondPerSecond(DebugStartAccel);
  #else
  stepper.setSpeedInStepsPerSecond(MaxStepsPerSecond);
  stepper.setAccelerationInStepsPerSecondPerSecond(MaxAccel);
  #endif
  DebugSerial.println(" complete.");
}

void loop() {  // put your main code here, to run repeatedly:
  buttonCheck();
  stepper.processMovement();
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
}

void FireRoutine(){ //routine to switch barrels after firing
  #ifdef DebugStepper
  DebugSerial.println("Fire button pressed... increase rpm by 10");
  currentRpm = currentRpm + 10;
  DebugSerial.print("CurrentRpm now at: ");
  DebugSerial.println(currentRpm);
  #else
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
  long currentPosition = stepper.getCurrentPositionInSteps();
  stepper.setTargetPositionInSteps(TargetPositionRotations(currentPosition, 0.5));
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
  #ifdef DebugStepper
  DebugSerial.println("animation button pressed... increase accel by 100");
  currentAccel = currentAccel + 100;
  DebugSerial.print("CurrentAccel now at: ");
  DebugSerial.println(currentAccel);
  #else
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





  inMotion = false; //release hold
  #endif
}

void StopMotion(){ //called when reload pin is released
  stopMotion = true; //Stop motion...
  stepper.setTargetPositionToStop(); //stop motion...
}

#ifdef DebugStepper
#define test1 //test1 is to get rpm and accel info
//#define test2 //test 2 measures steps from index pin triggers

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
  int testRpm = 10;
  int testAccel = 10;
  long currentPosition = 0;
  long lastIndexPress = 0;
  long lastIndexRelease = 0;
  long lastFireCircuitPress = 0;
  long lastFireCircuitRelease = 0;
  int indexSet = 0;
  int fireSet = 0;
  DebugSerial.println("Measure steps for pins routine starting");
  // Start motion
  inMotion = true; //block others till complete
  //check index pin and DebugFireCircuit pin for measurements
  //move slowly
  stepper.setSpeedInStepsPerSecond(RpmToSteps(testRpm));
  stepper.setAccelerationInStepsPerSecondPerSecond(testAccel);
  stepper.setCurrentPositionInSteps(currentPosition);
  stepper.setTargetPositionInSteps(TargetPositionRotations(currentPosition, 10));
  while(!stepper.motionComplete()){
    stepper.processMovement();
    // Index pin checking... 1 is released, 0 pressed.
    // if released and was released, do nothing...
    // if pressed and was released, msg position, indexSet = 0
    // if pressed and was pressed do nothing,
    // if released and was pressed, msg position, indexSet = 1
    if (digitalRead(IndexPin)) { //pin released
      if (indexSet == 0) { // default position first time through
        // was released last time... do nothing
      } else if (indexSet == 1) {
        //was pressed but now released
        currentPosition = stepper.getCurrentPositionInSteps();
        long diff = lastIndexPress - currentPosition;
        long diff2 = lastIndexRelease - currentPosition;
        DebugSerial.print("Index Released. Steps since last press: ");
        DebugSerial.print(diff);
        DebugSerial.print(" and since last release: ");
        DebugSerial.println(diff2);
        lastIndexRelease = currentPosition;
        indexSet = 0;
      }
    } else { //pin pressed
      if (indexSet == 0) {
        //pin now pressed...
        currentPosition = stepper.getCurrentPositionInSteps();
        long diff = lastIndexPress - currentPosition;
        long diff2 = lastIndexRelease - currentPosition;
        DebugSerial.print("Index Pressed. Steps since last press: ");
        DebugSerial.print(diff);
        DebugSerial.print(" and since last release: ");
        DebugSerial.println(diff2);
        lastIndexPress = currentPosition;
        indexSet = 1;
      } else if (indexSet == 1) {
        // pin was pressed last time, do nothing
      }
    }
    /*
    if (digitalRead(DebugFireCircuit)) { //pin released

    } else { //pin pressed

    } */
    buttonCheckRR();
  }

  currentPosition = stepper.getCurrentPositionInSteps();


  stepper.setSpeedInStepsPerSecond(MaxStepsPerSecond); // return movement speed to normal
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

float RpmToSteps(int rpm) {
  return (rpm/60)*StepsPerRotation;
}

long TargetPositionRotations(long currentPosition, float rotations) {
  return currentPosition + (StepsPerRotation * rotations);
}