/*
*     SPNKR Launcher code, STM32F103C8T6 blue pill
*     
*/

//Define Settings
#define DebugSerial SerialUSB   //Pick which serial port to use for debug messages
#define DebugStepper            //turn on debug stepper functions, comment out to turn off

#define TriggerPin
#define ReloadPin
#define IndexPin
#define AnimationPin
#define StepPin
#define DirectionPin

#ifdef DebugStepper
  #define DebugStepPin
  #define DebugDirectionPin
  #define DebugMesureStepsBetweenIndexPin
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

void IndexRoutine();
void FireRoutine();
void AnimationRoutine();


void setup() {  // put your setup code here, to run once:

}

void loop() {  // put your main code here, to run repeatedly:

}

void IndexRoutine(){

}

void FireRoutine(){

}

void AnimationRoutine(){
  
}