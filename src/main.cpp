/*
*     SPNKR Launcher code, STM32F103C8T6 blue pill
*     
*/

//Debug Settings comment out to disable
#define WaitForUsbSerial        //Wait for Debug Serial connection on startup, comment to turn off
#define DebugV                  //verbose debuging info... lots of info
#define DebugI                  //important debug info...
//NEVER COMMENT FOLLOWING LINE
#define DebugSerial SerialUSB   //Select which serial port to use for debug messages


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
  #define IndexStopAccel 10000
  #define FireRpm 63
  #define FireStartAccel 20000
  #define FireStopAccel 25000
  #define AnimatePause 200
  #define AnimateRpm 30
  #define AnimateAccel 20000
  const float notes[] = {82.41, 164.81, 185.00, 196.00, 185.00, 220.00, 196.00, 185.00, 164.81, 
                         82.41, 246.94, 277.18, 293.66, 277.18, 220.00, 277.18, 246.94,        82.41,
                         123.47, 146.83, 164.81, 196.00, 220.00, 185.00,       164.81, 196.00, 185.00,
                         164.81, 185.00, 146.83, 164.81,       82.41,       82.41,       82.41,
                         164.81, 82.41,       82.41,       82.41, 185.00, 82.41,       82.41,
                               82.41, 196.00, 82.41,       82.41,       82.41, 220.00, 82.41,
                         220.00,       220.00, 196.00, 185.00, 82.41,       82.41,       82.41,
                         164.81, 82.41,       82.41,       82.41, 185.00, 82.41,       82.41,
                               82.41, 196.00, 82.41,       82.41,       82.41, 220.00, 82.41,
                         220.00,       220.00, 196.00, 185.00};
  const long notesLength[] = {2133, 533, 533, 533, 533, 533, 533, 1000, 2133, 
                              1100, 533, 533, 1566, 533, 566, 533, 3200,      533,
                              533, 533, 533, 533, 533, 2166,      533, 533, 533,
                              533, 566, 1600, 3166,     200,   200,     166,
                              1400, 200,     200,     166, 1400, 200,     200,
                                   166, 1433, 200,     200,     166, 466, 166,
                              333,      133, 166, 133, 200,     200,     166,
                              1400, 200,     200,     166, 1400, 200,     200,
                                   166, 1433, 200,     200,     166, 466, 166,
                              333,     133, 166, 133};
  const int notesTotal = 29;
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


#ifdef DebugV
#define sPrintV(a) (DebugSerial.print(a))
#define sPrintlnV(a) (DebugSerial.println(a))
#else
#define sPrintV(a)
#define sPrintlnV(a)
#endif

#ifdef DebugI
#define sPrintI(a) (DebugSerial.print(a))
#define sPrintlnI(a) (DebugSerial.println(a))
#else
#define sPrintI(a)
#define sPrintlnI(a)
#endif


//Included Librarys
#include <Arduino.h>            //Add arduino functions
#include <FlexyStepper.h>       //Add Stepper Library



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
constexpr long TargetPositionRotations(float);
int CheckHolds();
int DelayPlus(long);
int IndexPlus(long, float, float, float, long);
int MovePlus(long, float, float, float);
int MoveExact(long, float, float, float);
void MeasureStepsBetweenIndexPin();
void printV();
void printE();


void setup() {  // startup code
  DebugSerial.begin(); //activate USB CDC driver
  #ifdef WaitForUsbSerial
    while(!DebugSerial); //blocks till usb serial is connected... turn off if not connected to PC.
  #endif
  sPrintlnI("SPNKR Launcher starting");
  sPrintI("Setup input pins...");
  pinMode(IndexPin, INPUT_PULLUP);
  pinMode(TriggerPin, INPUT_PULLUP);
  pinMode(ReloadPin, INPUT_PULLUP);
  pinMode(AnimationPin, INPUT_PULLUP);
  pinMode(DebugActionPin, INPUT_PULLUP);

  sPrintlnI(" complete.");

  sPrintI("Setup stepper library... ");
  stepper.connectToPins(StepPin, DirectionPin);
  stepper.setSpeedInStepsPerSecond(RpmToSteps(StartRpm));
  stepper.setAccelerationInStepsPerSecondPerSecond(StartAccel);
  stepper.setCurrentPositionInSteps(0);
  stepper.setTargetPositionInSteps(0);
  
  sPrintlnI(" complete.");
  sPrintV("Variables check: DrumTeeth: ");
  sPrintV(DrumTeeth);
  sPrintV(" StepperTeeth: ");
  sPrintlnV(StepperTeeth);
  sPrintV("Microstepping: ");
  sPrintlnV(MicroStepping);
  sPrintV("StepsPerRotation (DrumTeeth/StepperTeeth)*(MicroStepping*200): ");
  sPrintlnV(StepsPerRotation);
  sPrintV("MaxStepsPerSecond (StartRpm/60)*StepsPerRotation : ");
  sPrintlnV(MaxStepsPerSecond);
  sPrintV("StartRpm: ");
  sPrintV(StartRpm);
  sPrintV(" StartAccel: ");
  sPrintlnV(StartAccel);
}

void loop() {  //Idle...
  buttonCheck();
  if(stopMotion) {
    stepper.processMovement();
  }
}

void IndexRoutine(){ //routine to index drum, called when reloadPin is pressed
  stopMotion = false; //allow motion
  sPrintlnI("LID Closed allow motion");
  #ifdef final
    //Check Holds...
    if (CheckHolds()) return;
    //Start motion
    inMotion = true; //block others till complete
    if (IndexPlus(StepsFromLimit, RpmToSteps(IndexRpm), IndexStartAccel, IndexStopAccel, 0)) {
      sPrintlnV("Index Routine IndexPlus failed");
      return;
    }
    inMotion = false; //release hold
  #endif
}

void FireRoutine(){ //routine to switch barrels after firing
  #ifdef test1
    //Increase rpm setting
    sPrintlnV("Fire button pressed... increase rpm by 10");
    currentRpm = currentRpm + 10;
    sPrintV("CurrentRpm now at: ");
    sPrintlnV(currentRpm);
  #endif

  #ifdef test3
    //move drum by StepsPerTrigger amount each click
    //Check Holds...
    if (CheckHolds()) return;
    //Start motion
    inMotion = true; //block others till complete
    currentPosition = stepper.getCurrentPositionInSteps();
    sPrintV("Moving from: ");
    sPrintV(currentPosition);
    sPrintV(" to: ");
    sPrintlnV(currentPosition + StepsPerTrigger);
    if (MovePlus(StepsPerTrigger, RpmToSteps(StartRpm), StartAccel, StartAccel)) {
      sPrintlnV("Fire Routine MovePlus failed");
      return;
    }
    sPrintlnV("Done...");
    currentPosition = stepper.getCurrentPositionInSteps();
    sPrintV("Steps from last index is: ");
    sPrintlnV(currentPosition - stepsFromIndexRelease);
    sPrintlnV("average this value after a few tries for use in final StepsFromLimit");
    inMotion = false; //release hold
  #endif

  #ifdef final
  //normal operation of fire button
  //Check Holds...
  if (CheckHolds()) return;
  //Start motion
  inMotion = true; //block others till complete
  if (IndexPlus(StepsFromLimit, RpmToSteps(FireRpm), FireStartAccel, FireStopAccel, 0)) {
    sPrintlnV("Fire Routine IndexPlus failed");
    return;
  }
  inMotion = false; //release hold
  #endif
}

void AnimationRoutine(){ //routine to perform animation
  #ifdef test1
  sPrintlnV("animation button pressed... increase accel by 100");
  currentAccel = currentAccel + 100;
  sPrintV("CurrentAccel now at: ");
  sPrintlnV(currentAccel);
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
        sPrintlnI("test3 animate Error stopMotion stopping");
        stepper.setCurrentPositionInSteps(0);
        stepper.setTargetPositionInSteps(0);
        sPrintlnI("Error stopMotion stopping");
        ret = 1;
        keepGoing = 0;
        break;
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
    sPrintV("Slow home to index pin release finished now at: ");
    sPrintlnV(stepsFromIndexRelease);

    inMotion = false; //release hold
  #endif

  #ifdef final
  //Check Holds...
  if (CheckHolds()) return;
  //Start motion
  inMotion = true; //block others till complete
  //code here for animation

  sPrintlnV("Animation #1");
  //#1
  if (MovePlus(TargetPositionRotations(-0.0625), RpmToSteps(80), 26000, 30000)) {
    sPrintlnV("Animation Routine #1 failed");
    return;
  }
  //#2
  sPrintlnV("Animation #2");
  DelayPlus(173);
  //#3
  sPrintlnV("Animation #3");
  if (IndexPlus(282, RpmToSteps(85), 15000, 30000, 300)) {
    sPrintlnV("Animation Routine #3 IndexPlus failed");
    return;
  }
  //#4
  sPrintlnV("Animation #4");
  DelayPlus(164);
  //#5
  sPrintlnV("Animation #5");
  if (MoveExact((lastIndexRelease + StepsFromLimit), RpmToSteps(80), 26000, 30000)) {
    sPrintlnV("Animation Routine #5 failed");
    return;
  }

  //#6
  sPrintlnV("Animation #6");
  DelayPlus(300);
  //#7
  sPrintlnV("Animation #7");
  if (IndexPlus(282, RpmToSteps(80), 11000, 30000, 0)) {
    sPrintlnV("Animation Routine #7 IndexPlus failed");
    return;
  }
  //#8
  sPrintlnV("Animation #8");
  DelayPlus(200);
  //#9
  sPrintlnV("Animation #9");
  if (MoveExact((lastIndexRelease + StepsFromLimit), RpmToSteps(80), 26000, 30000)) {
    sPrintlnV("Animation Routine #9 failed");
    return;
  }
  sPrintlnV("Animation Complete");
  inMotion = false; //release hold
  #endif
}

void StopMotion(){ //called when reload pin is released
  stopMotion = true; //Stop motion...
  stepper.setCurrentPositionInSteps(0);
  stepper.setTargetPositionInSteps(0);
  //stepper.setTargetPositionToStop(); //stop motion...
  sPrintlnI("LID OPENED STOP IMMEADATLEY>>>");
  inMotion = false; //release hold
}


void MeasureStepsBetweenIndexPin(){ //debug routine to show steps between index hits via serial output
  //Check Holds...
  if (CheckHolds()) return;

  #ifdef final //final code, play song...
  //Start motion
  inMotion = true; //block others till complete
  unsigned long previousMillis = millis();  //timer storage
  unsigned long currentMillis = millis();  //timer storage
  int noteCounter = 0; //current note
  int stateCounter = 0; // current state (seperate from noteCounter to avoid array overflow)
  stepper.setCurrentPositionInSteps(0); //start up
  stepper.setTargetPositionInSteps(2000000); //a long way from here...
  stepper.setSpeedInStepsPerSecond(notes[noteCounter]); //first note...
  stepper.setAccelerationInStepsPerSecondPerSecond(5000); //music accel...
  while(!stepper.motionComplete()){
    stepper.processMovement();
    currentMillis = millis(); //set current time
    if ((currentMillis - previousMillis) >= notesLength[noteCounter]) { //if note length is past, update...
      previousMillis = currentMillis; //update previous time
      stateCounter++; // increment stateCounter
      if (stateCounter > notesTotal) { //past last note, stop machine...
        stepper.setCurrentPositionInSteps(0);
        stepper.setTargetPositionInSteps(0);
      } else { //setup next note
        noteCounter = stateCounter; //update current note
        sPrintV("Note # ");
        sPrintV(noteCounter);
        sPrintV(" freq: ");
        sPrintV(notes[noteCounter]);
        sPrintV(" length: ");
        sPrintlnV(notesLength[noteCounter]);
        if (notes[noteCounter] < 1.0) { //if the freq is less than 1hz, stop motor
          stepper.setCurrentPositionInSteps(0);
          stepper.setTargetPositionInSteps(0);
        } else { //normal note, or restarting from stopped note...
          stepper.setCurrentPositionInSteps(0);
          stepper.setTargetPositionInSteps(2000000);
          stepper.setSpeedInStepsPerSecond((notes[noteCounter])*2);
        }
      }
    }

    if (stopMotion) {
        stepper.setCurrentPositionInSteps(0);
        stepper.setTargetPositionInSteps(0);
        sPrintlnI("Error stopMotion stopping");
        break;
    }
    buttonCheck();
  }
  DelayPlus(1000);
  if (IndexPlus(StepsFromLimit, RpmToSteps(FireRpm), FireStartAccel, FireStopAccel, 0)) {
    sPrintlnV("Fire Routine IndexPlus failed");
    return;
  }
  inMotion = false; //release hold
  #endif

  #ifdef test2 //test2 code
  //Variables...
  long currentPosition = 0;
  long lastIndexPress = 0;
  long lastIndexRelease = 0;
  long lastFireCircuitPress = 0;
  long lastFireCircuitRelease = 0;
  int indexSet = 0;
  int countPress = 0;
  sPrintlnV("Measure steps for pins routine starting");
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
          sPrintV("Index Released. Steps since last press: ");
          sPrintV(diff);
          sPrintV(" and since last release: ");
          sPrintlnV(diff2);
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
          sPrintV("New Press group count is: ");
          sPrintV(countPress);
          sPrintlnV(" ----------------------------------");
          sPrintV("Index Pressed. Steps since last press: ");
          sPrintV(diff);
          sPrintV(" and since last release: ");
          sPrintlnV(diff2);
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
  sPrintlnV("Rotate 2 times then stop with current accel and rpm");
  sPrintV("Current Accel value: ");
  sPrintlnV(currentAccel);
  sPrintV("Current RPM value: ");
  sPrintlnV(currentRpm);
  if (MovePlus(TargetPositionRotations(2), RpmToSteps(currentRpm), currentAccel, currentAccel)) {
    sPrintlnV("MeasureSteps MovePlus failed");
    return;
  }
  #endif
  sPrintlnV("Movement complete");
  inMotion = false; //release hold
}


constexpr float RpmToSteps(float rpm) {
  return (rpm/60.0)*StepsPerRotation;
}

constexpr long TargetPositionRotations(float rotations) {
  //currentPosition = stepper.getCurrentPositionInSteps();
  return (StepsPerRotation * rotations);
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
    sPrintlnV("Trigger Released");
    FireRoutine();
  }
  if (b1flag == 2) {
    b1flag = 0;
    sPrintlnV("Trigger Pressed");
  }
  if (b2flag == 1) {
    b2flag = 0;
    sPrintlnV("Reload Released");
    StopMotion();
  }
  if (b2flag == 2) {
    b2flag = 0;
    sPrintlnV("Reload Pressed");
    IndexRoutine();
  }
  if (b3flag == 1) {
    b3flag = 0;
    sPrintlnV("Animate Released");
    AnimationRoutine();
  }
  if (b3flag == 2) {
    b3flag = 0;
    sPrintlnV("Animate Pressed");
  }
    if (b4flag == 1) {
    b4flag = 0;
    sPrintlnV("Debug Released");
    MeasureStepsBetweenIndexPin();
  }
  if (b4flag == 2) {
    b4flag = 0;
    sPrintlnV("Debug Pressed");
  }

}

//Move ignoring Index pin
//returns 0 if successful, 1 for fail
int MovePlus(long dist, float sps, float startA, float stopA) {
  sPrintlnV("MovePlus Routine");
  currentPosition = stepper.getCurrentPositionInSteps();
  stepper.setSpeedInStepsPerSecond(sps);
  stepper.setAccelerationInStepsPerSecondPerSecond(startA);
  stepper.setTargetPositionInSteps(currentPosition + dist);
  int ret = 0;
  unsigned long startedNow = millis();
  while(!stepper.motionComplete()){
    stepper.processMovement();
    if (stepper.getCurrentVelocityInStepsPerSecond() == sps) {
      stepper.setAccelerationInStepsPerSecondPerSecond(stopA);
    }
    if (stopMotion) {
        stepper.setCurrentPositionInSteps(0);
        stepper.setTargetPositionInSteps(0);
        sPrintlnI("Error stopMotion stopping");
        ret = 1;
        break;
    }
    buttonCheck();
  }
  sPrintV(millis() - startedNow);
  sPrintlnV(" ms long");
  return ret;
}

int MoveExact(long dist, float sps, float startA, float stopA) {
  sPrintlnV("MoveExact Routine");
  currentPosition = stepper.getCurrentPositionInSteps();
  stepper.setSpeedInStepsPerSecond(sps);
  stepper.setAccelerationInStepsPerSecondPerSecond(startA);
  stepper.setTargetPositionInSteps(dist);
  int ret = 0;
  unsigned long startedNow = millis();
  while(!stepper.motionComplete()){
    stepper.processMovement();
    if (stepper.getCurrentVelocityInStepsPerSecond() == sps) {
      stepper.setAccelerationInStepsPerSecondPerSecond(stopA);
    }
    if (stopMotion) {
        stepper.setCurrentPositionInSteps(0);
        stepper.setTargetPositionInSteps(0);
        sPrintlnI("Error stopMotion stopping");
        ret = 1;
        break;
    }
    buttonCheck();
  }
  sPrintV(millis() - startedNow);
  sPrintlnV(" ms long");
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
  sPrintlnV("IndexPlus routine");
  unsigned long startedNow = millis();
  while(keepGoing){
    while(!stepper.motionComplete()){
      stepper.processMovement();
      currentPosition = stepper.getCurrentPositionInSteps();
      if (digitalRead(IndexPin)) { //pin released
        if (indexSet == 1) {
          //sPrintlnV("First Index release detected...");
          if (currentPosition - lastIndexPress > indexDebounceSteps) {
          //was pressed but now released
          sPrintlnV("#1 First Index release debounce ok");
          //sPrintlnV(currentPosition);
          lastIndexRelease = currentPosition;
          stepper.setTargetPositionInSteps(currentPosition + extra);
          indexSet = 3; //end routine
          }
        }
      } else { //pin pressed
        if (indexSet == 0) {
          //pin now pressed...
          if (currentPosition - minDistance > 0) {
            sPrintlnV("#0 First index press detected...");
            sPrintlnV(currentPosition);
            stepper.setAccelerationInStepsPerSecondPerSecond(stopA);
            indexSet = 1;
            lastIndexPress = currentPosition;
          } else {
            //These comments cause stuttering in the routine...
            //sPrintlnV("Index press detected but minimum travel not met...");
            //sPrintV("Distance start to hit: ");
            //sPrintlnV(currentPosition-startPosition);
          }
        }
      }
      if (stopMotion) {
        sPrintlnI("Error stopMotion stopping");
        stepper.setCurrentPositionInSteps(0);
        stepper.setTargetPositionInSteps(0);
        keepGoing = 0;
        ret = 1;
        break;

      }
      buttonCheck();
    } //end proccess movement

    if (indexSet == 0) {
      sPrintlnI("ERROR Failed to detect index pin...");
      keepGoing = 0;
      ret = 1;
    }

    if (indexSet == 3) {
      sPrintlnV("Done, setting normal speeds.");
      sPrintlnV(currentPosition);
      keepGoing = 0;
    }

  }
  stepper.setSpeedInStepsPerSecond(RpmToSteps(StartRpm));
  stepper.setAccelerationInStepsPerSecondPerSecond(StartAccel);
  //stepper.setCurrentPositionInSteps(0);
  sPrintV(millis() - startedNow);
  sPrintlnV(" ms long");
  return ret; 
}


//Delay function while still checking buttons
//returns 0 if successful, 1 for fail
int DelayPlus(long valu) {
  sPrintlnV("Delay Plus routine");
  unsigned long now = millis();
  unsigned long ending = now + valu;
  int dumb = 1;
  while(dumb) {
    now = millis();
    //sPrintlnV(now);
    long test = now - ending;
    if (test > 0) {
      dumb = 0;
    }
    buttonCheck();
  }
  sPrintV(valu);
  sPrintlnV(" ms long");
  return 0;
}

int CheckHolds(){ //returns 1 if holds, 0 if none
  //Check Holds...
  int holds = 0;
  if (stopMotion) { //if motion not allowed do nothing
    sPrintlnI("Error stopMotion blocking");
    holds = 1;
  } 
  if (inMotion) { //if already inMotion cant move...
    sPrintlnI("Error already in motion");
    holds = 1;
  }
  return holds;
}

//The END!