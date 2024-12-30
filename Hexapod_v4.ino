// 4th generation Hexapod
// Hardware is largely the same as v3. Currently uses a Nano 33 BLE Sense Rev 2, but will also work on other capable
// Arduinos like the Nano 33 IoT

//#define DEBUG

#include <Arduino_BMI270_BMM150.h>
#include <Adafruit_PWMServoDriver.h>
#include "Config.h"
#include "Hexapod.h"
#include "Leg.h"

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(pwm1Addr);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(pwm2Addr);

// initialize all six legs of the robot using config.h
Leg legFrontRight = Leg(pwm2, coxaPinFR, femurPinFR, tibiaPinFR, coxaOffsetFR, femurOffsetFR, tibiaOffsetFR, true, buttonFR);
Leg legFrontLeft = Leg(pwm1, coxaPinFL, femurPinFL, tibiaPinFL, coxaOffsetFL, femurOffsetFL, tibiaOffsetFL, false, buttonFL);
Leg legMidRight = Leg(pwm2, coxaPinMR, femurPinMR, tibiaPinMR, coxaOffsetMR, femurOffsetMR, tibiaOffsetMR, true, buttonMR);
Leg legMidLeft = Leg(pwm1, coxaPinML, femurPinML, tibiaPinML, coxaOffsetML, femurOffsetML, tibiaOffsetML, false, buttonML);
Leg legRearRight = Leg(pwm2, coxaPinRR, femurPinRR, tibiaPinRR, coxaOffsetRR, femurOffsetRR, tibiaOffsetRR, true, buttonRR);
Leg legRearLeft = Leg(pwm1, coxaPinRL, femurPinRL, tibiaPinRL, coxaOffsetRL, femurOffsetRL, tibiaOffsetRL, false, buttonRL);

// Initialization of hexapod object
Hexapod myHexapod = Hexapod(legFrontRight, legFrontLeft, legMidRight, legMidLeft, legRearRight, legRearLeft);
unsigned long loopCounter = 0;

// array which stores the positions of each leg in x-y-z local coordinates
// used to pass the calculated new positions to the .movelegs() methode
float legPositions[6][3] = { { homePos[0], homePos[1], homePos[2] },    // front right
                             { homePos[0], homePos[1], homePos[2] },    // front left
                             { homePos[0], homePos[1], homePos[2] },    // mid right
                             { homePos[0], homePos[1], homePos[2] },    // mid left
                             { homePos[0], homePos[1], homePos[2] },    // rear right
                             { homePos[0], homePos[1], homePos[2] } };  // rear left

float currPositions[6][3];
float newPositions[6][3];
uint16_t counter = 0;

void setup() {

#ifdef ARDUINO_ARDUINO_NANO33BLE
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  // Built in RGB is inverted
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_BLUE, HIGH);
  digitalWrite(LED_GREEN, HIGH);
#endif
  // use the builtin LED on pin 13 as an OUTPUT
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  // use all the push buttons on the legs as INPUT
  pinMode(buttonFR, INPUT);
  pinMode(buttonFL, INPUT);
  pinMode(buttonMR, INPUT);
  pinMode(buttonML, INPUT);
  pinMode(buttonRR, INPUT);
  pinMode(buttonRL, INPUT);

#ifdef DEBUG
  // used for debug
  Serial.begin(9600);
#endif

  // start pwm on the servo drivers with 50Hz frequency
  pwm1.begin();
  pwm1.setPWMFreq(50);
  pwm2.begin();
  pwm2.setPWMFreq(50);

  // move all legs to their home position
  myHexapod.moveHome();

  // start the IMU (currently unused)
  if (!IMU.begin()) {
    Serial.println("IMU-Sensor couldn't be initialized!");
    while (1) {}
  }
  delay(1000);

  // Configure WDT. Only uncomment this if it is your final version, since the watchdog prevents
  // normal upload to the board. Otherwise it needs to be set to bootloader mode manually.
  /*NRF_WDT->CONFIG = 0x01;    // Configure WDT to run when CPU is asleep
  NRF_WDT->CRV = 65535;      // Timeout set to 2 seconds, timeout[s] = (CRV+1)/32768
  NRF_WDT->RREN = 0x01;      // Enable the RR[0] reload register
  NRF_WDT->TASKS_START = 1;  // Start WDT*/
}

void loop() {
  // get the current time
  unsigned long timeMillis = millis();

  // exemplary movements of the robot
  // uncomment one of the following
  //exampleBodyMovement();
  //exampleSteps();
  exampleSteps2();

  // update the leg position
  myHexapod.moveLegs(newPositions);

  // increment the loopCounter to keep track of the number of loop cycles
  loopCounter++;
  if (loopCounter == 2500) {
    loopCounter = 0;
  }

#ifdef ARDUINO_ARDUINO_NANO33BLE
  if (loopCounter % 2) {  // turn the led on and of to show each loop iteration
    digitalWrite(LED_GREEN, HIGH);
  } else {
    digitalWrite(LED_GREEN, LOW);
  }
#else
  if (loopCounter % 2) {  // turn the led on and of to show each loop iteration
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
#endif

#ifdef DEBUG
  // Make sure that the arduino is fast enough to calculate everything in periodMS time span
  unsigned long millisCalc = millis();
  Serial.print("Time calculated:");
  Serial.println(millisCalc - timeMillis);
  while (millis() < timeMillis + periodMs) {
    // wait a bit so that the loop is executet every periodMS ms
  }
  Serial.print("Time waited:");
  Serial.println(millis() - millisCalc);
#else
  while (millis() < timeMillis + periodMs) {
    // wait a bit so that the loop is executet every periodMS ms
  }
#endif

  // Reload the WDTs RR[0] reload register
  // if this line isn't called at least every 2 seconds, the TIMEOUT event is called and the CPU is reset
  // uncomment this if the watchdog is used
  //NRF_WDT->RR[0] = WDT_RR_RR_Reload;
}


void exampleSteps() {
  // example usage of the calcStep() method
  // keep track of the number of steps
  if (myHexapod.getAction() == 0) {
    // increase the counter each time the robot is resting
    counter++;
    if (counter == 25) {
      counter = 0;
    }
  }

  // do a couple steps in different directions
  if (counter < 5) {
    myHexapod.calcStep(legPositions, currPositions, 0, 30, 120, 0.0, 20);
  } else if (counter < 15) {
    myHexapod.calcStep(legPositions, currPositions, PI / 2, 20, 120, 0.15, 20);
  } else if (counter < 20) {
    myHexapod.calcStep(legPositions, currPositions, 0, 20, 120, 0.15, 20);
  } else {
    myHexapod.calcStep(legPositions, currPositions, PI, 3, 120, -0.2, 20);
  }

  // Adjust the body position each loop() iteration. This allows for superimposition of body movement independently of the step
  myHexapod.calcBodyMovement(currPositions, newPositions, 0, 0, 0, 0.0, 0.0, 0.0);
}

void exampleSteps2() {
  // example usage of the calcStep() method
  // keep track of the number of steps
  if (myHexapod.getAction() == 0) {
    // increase the counter each time the robot is resting
    counter++;
    if (counter == 21) {
      counter = 0;
    }
  }
  float newDir = PI + (counter - 1) * 0.15;
  myHexapod.calcStep(legPositions, currPositions, newDir, 20, 120, 0.15, 20);

  // Adjust the body position each loop() iteration. This allows for superimposition of body movement independently of the step
  myHexapod.calcBodyMovement(currPositions, newPositions, 0, 0, 0, 0.0, 0.0, 0.0);
}

void exampleBodyMovement() {
  // example usage of calcBodyMovement to move the robots body

  if (loopCounter < 150) {
    int b = map(loopCounter, 0, 150, 0, 180);
    myHexapod.calcBodyMovement(legPositions, newPositions, 0, 0, 0, 0.0, b / 1000.0, 0.0);

  } else if (loopCounter < 250) {
    myHexapod.calcBodyMovement(legPositions, newPositions, 0, 0, 0, 0.0, 0.15, 0.0);

  } else if (loopCounter < 400) {
    int b = map(loopCounter, 250, 400, 180, -180);
    myHexapod.calcBodyMovement(legPositions, newPositions, 0, 0, 0, 0.0, b / 1000.0, 0.0);

  } else if (loopCounter < 500) {
    myHexapod.calcBodyMovement(legPositions, newPositions, 0, 0, 0, 0.0, -0.18, 0.0);

  } else if (loopCounter < 650) {
    int b = map(loopCounter, 500, 650, -180, 0);
    myHexapod.calcBodyMovement(legPositions, newPositions, 0, 0, 0, 0.0, b / 1000.0, 0.0);

  } else if (loopCounter < 800) {
    int b = map(loopCounter, 650, 800, 0, 150);
    myHexapod.calcBodyMovement(legPositions, newPositions, 0, 0, 0, b / 1000.0, 0.0, 0.0);

  } else if (loopCounter < 950) {
    int b = map(loopCounter, 800, 950, 150, -150);
    myHexapod.calcBodyMovement(legPositions, newPositions, 0, 0, 0, b / 1000.0, 0.0, 0.0);

  } else if (loopCounter < 1100) {
    int b = map(loopCounter, 950, 1100, -150, 0);
    myHexapod.calcBodyMovement(legPositions, newPositions, 0, 0, 0, b / 1000.0, 0.0, 0.0);

  } else if (loopCounter < 1250) {
    int b = map(loopCounter, 1100, 1250, 0, 20);
    myHexapod.calcBodyMovement(legPositions, newPositions, 0, 0, b, 0.0, 0.0, 0.0);

  } else if (loopCounter < 1350) {
    myHexapod.calcBodyMovement(legPositions, newPositions, 0, 0, 20, 0.0, 0.0, 0.0);

  } else if (loopCounter < 1500) {
    int b = map(loopCounter, 1350, 1500, 20, -20);
    myHexapod.calcBodyMovement(legPositions, newPositions, 0, 0, b, 0.0, 0.0, 0.0);

  } else if (loopCounter < 1600) {
    myHexapod.calcBodyMovement(legPositions, newPositions, 0, 0, -20, 0.0, 0.0, 0.0);

  } else if (loopCounter < 1750) {
    int b = map(loopCounter, 1600, 1750, -20, 0);
    int c = map(loopCounter, 1600, 1750, 0, 20);
    myHexapod.calcBodyMovement(legPositions, newPositions, c, 0, b, 0.0, 0.0, 0.0);

  } else if (loopCounter < 2000) {
    int b = round(20.0 * sin(loopCounter * 2.0 * PI / 250.0));
    int c = round(20.0 * cos(loopCounter * 2.0 * PI / 250.0));
    myHexapod.calcBodyMovement(legPositions, newPositions, c, b, 0, 0.0, 0.0, 0.0);

  } else if (loopCounter < 2100) {
    int c = map(loopCounter, 2000, 2100, 20, 0);
    myHexapod.calcBodyMovement(legPositions, newPositions, c, 0, 0, 0.0, 0.0, 0.0);

  } else if (loopCounter < 2250) {
    int b = map(loopCounter, 2100, 2250, 0, 150);
    myHexapod.calcBodyMovement(legPositions, newPositions, 0, 0, 0, b / 1000.0, b / 1000.0, 0.0);

  } else if (loopCounter < 2400) {
    int b = map(loopCounter, 2250, 2400, 150, 0);
    myHexapod.calcBodyMovement(legPositions, newPositions, 0, 0, 0, b / 1000.0, b / 1000.0, 0.0);

  } else {
    myHexapod.calcBodyMovement(legPositions, newPositions, 0, 0, 0, 0.0, 0.0, 0.0);
  }
}
