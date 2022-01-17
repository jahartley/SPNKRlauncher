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
#endif

//Stepper Constants
#define MaxAccel 100
#define MaxDrumRpm 50
#define DrumTeeth 400
#define StepperTeeth 12
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
#else
  int buttonCount = 3;
#endif

void buttonCheck();
void buttonCheckRR();
void IndexRoutine();
void FireRoutine();
void AnimationRoutine();
void StopMotion();
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
  stepper.setSpeedInStepsPerSecond(MaxStepsPerSecond);
  stepper.setAccelerationInStepsPerSecondPerSecond(MaxAccel);
  DebugSerial.println(" complete.");
}

void loop() {  // put your main code here, to run repeatedly:
  buttonCheck();
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
  stepper.setTargetPositionInSteps(currentPosition + (StepsPerRotation/2));
  while(!stepper.motionComplete()){
    stepper.processMovement();
    if (stopMotion) {
      stepper.setTargetPositionToStop();
    }
    buttonCheckRR();
  }
  inMotion = false; //release hold
}

void AnimationRoutine(){ //routine to perform animation
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
}

void StopMotion(){ //called when reload pin is released
  stopMotion = true; //Stop motion...
  stepper.setTargetPositionToStop(); //stop motion...
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
  //Variables...
  long currentPosition = 0;
  long lastIndexPress = 0;
  long lastIndexRelease = 0;
  long lastFireCircuitPress = 0;
  long lastFireCircuitRelease = 0;
  int indexSet = 0;
  int indexReset = 0;
  int fireSet = 0;
  DebugSerial.println("Measure steps for pins routine starting");
  // Start motion
  inMotion = true; //block others till complete
  //check index pin and DebugFireCircuit pin for measurements
  //move slowly
  stepper.setSpeedInStepsPerSecond(MaxStepsPerSecond/20);
  stepper.setCurrentPositionInSteps(currentPosition);
  stepper.setTargetPositionInSteps(2000000000);
  while(!stepper.motionComplete()){
    stepper.processMovement();
    if (digitalRead(IndexPin)) { //pin released
      if (indexSet != 0 && indexReset == 0) {

      }
    } else { //pin pressed

    }
    if (digitalRead(DebugFireCircuit)) { //pin released

    } else { //pin pressed

    }
  }

currentPosition = stepper.getCurrentPositionInSteps();


  stepper.setSpeedInStepsPerSecond(MaxStepsPerSecond); // return movement speed to normal
  inMotion = false; //release hold
}
#endif