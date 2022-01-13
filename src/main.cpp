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
  #define DebugDirectionPin PB15
  #define DebugMeasureStepsBetweenIndexPin PB4
#endif

//Stepper Constants
#define MaxAccel 10
#define MaxDecel 10
#define MaxDrumRpm 50
#define DrumTeeth 400
#define StepperTeeth 12
#define MicroStepping 2
#define Reverse false

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

void buttonCheck();
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
/*  pinMode(TriggerPin,INPUT_PULLUP);
  pinMode(ReloadPin, INPUT_PULLUP);
  pinMode(AnimationPin, INPUT_PULLUP);
  #ifdef DebugStepper
    pinMode(DebugStepPin, INPUT_PULLUP);
    pinMode(DebugDirectionPin, INPUT_PULLUP);
    pinMode(DebugMeasureStepsBetweenIndexPin, INPUT_PULLUP);
  #endif */
  pinMode(IndexPin, INPUT_PULLUP);
  trigger.setRelease(FireRoutine);
  animation.setRelease(AnimationRoutine);
  reload.setPress(IndexRoutine);
  reload.setRelease(StopMotion);
  #ifdef DebugStepper
    debugMeasure.setRelease(MeasureStepsBetweenIndexPin);
  #endif
  DebugSerial.println(" complete.");

  DebugSerial.print("Setup stepper library... ");
  stepper.connectToPins(StepPin, DirectionPin);
  DebugSerial.println(" complete.");
}

void loop() {  // put your main code here, to run repeatedly:
  buttonCheck();
}

void buttonCheck(){
  //code for button inputs to call routines...
  trigger.isClick();
  reload.isClick();
  //index.isClick();
  animation.isClick();
  #ifdef DebugStepper
    debugMeasure.isClick();
  #endif
}

void IndexRoutine(){ //routine to index drum
  stopMotion = false; //allow motion
}

void FireRoutine(){ //routine to switch barrels after firing
  if (stopMotion) {return;} //if motion not allowed do nothing
}

void AnimationRoutine(){ //routine to perform animation
  if (stopMotion) {return;} //if motion not allowed do nothing
}

void StopMotion(){
  stopMotion = true; //Stop motion...
  stepper.setTargetPositionToStop(); //stop motion...
}

#ifdef DebugStepper
void MeasureStepsBetweenIndexPin(){ //debug routine to show steps between index hits via serial output

}
#endif