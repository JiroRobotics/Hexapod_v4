// 4th generation Hexapod
// Hardware is largely the same as v3 (white). Currently uses a Nano 33 IoT, but will likely be upgraded to use a Nano 33 BLE Sense v2
// Completely new software, each leg is represented as a class with references to each servo
#include "Leg.h"
#include <Adafruit_PWMServoDriver.h>
#include <SpeedTrig.h>
#include "Config.h"
#include "Hexapod.h"

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);

// initialize all six legs of the robot using config.h
Leg legFrontRight = Leg(pwm2, coxaPinFR, femurPinFR, tibiaPinFR, coxaOffsetFR, femurOffsetFR, tibiaOffsetFR, true, buttonFR);
Leg legFrontLeft = Leg(pwm1, coxaPinFL, femurPinFL, tibiaPinFL, coxaOffsetFL, femurOffsetFL, tibiaOffsetFL, false, buttonFL);
Leg legMidRight = Leg(pwm2, coxaPinMR, femurPinMR, tibiaPinMR, coxaOffsetMR, femurOffsetMR, tibiaOffsetMR, true, buttonMR);
Leg legMidLeft = Leg(pwm1, coxaPinML, femurPinML, tibiaPinML, coxaOffsetML, femurOffsetML, tibiaOffsetML, false, buttonML);
Leg legRearRight = Leg(pwm2, coxaPinRR, femurPinRR, tibiaPinRR, coxaOffsetRR, femurOffsetRR, tibiaOffsetRR, true, buttonRR);
Leg legRearLeft = Leg(pwm1, coxaPinRL, femurPinRL, tibiaPinRL, coxaOffsetRL, femurOffsetRL, tibiaOffsetRL, false, buttonRL);

// an array of pointers pointing to the six legs
Leg* legs[6] = { &legFrontRight, &legFrontLeft, &legMidRight, &legMidLeft, &legRearRight, &legRearLeft };

// Initialization of hexapod object
Hexapod myHexapod = Hexapod(legFrontRight, legFrontLeft, legMidRight, legMidLeft, legRearRight, legRearLeft);
unsigned long loopCounter = 0;

// array which stores the positions of each leg in x-y-z local coordinates
// used to pass the calculated new positions to the .movelegs() methode
int legPositions[6][3] = { { homePos[0], homePos[1], homePos[2] },    // front right
                           { homePos[0], homePos[1], homePos[2] },    // front left
                           { homePos[0], homePos[1], homePos[2] },    // mid right
                           { homePos[0], homePos[1], homePos[2] },    // mid left
                           { homePos[0], homePos[1], homePos[2] },    // rear right
                           { homePos[0], homePos[1], homePos[2] } };  // rear left

void setup() {
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

  // used for debug
  Serial.begin(9600);

  // start pwm on the servo drivers with 50Hz frequency
  pwm1.begin();
  pwm1.setPWMFreq(50);
  pwm2.begin();
  pwm2.setPWMFreq(50);

  // move all legs to their home position
  myHexapod.moveHome();
  delay(1000);

  //Configure WDT.
  NRF_WDT->CONFIG = 0x01;    // Configure WDT to run when CPU is asleep
  NRF_WDT->CRV = 65535;      // Timeout set to 2 seconds, timeout[s] = (CRV+1)/32768
  NRF_WDT->RREN = 0x01;      // Enable the RR[0] reload register
  NRF_WDT->TASKS_START = 1;  // Start WDT
}
void loop() {
  // get the current time
  unsigned long timeMillis = millis();

  // calculate the new leg position
  // ...


  // update the leg position
  myHexapod.moveLegs(legPositions);
  while (millis() < timeMillis + periodMs) {
    // wait a bit so that the loop is executet every 20ms
  }
  // increment the loopCounter to keep track of the number of loop cycles
  loopCounter++;
  if(loopCounter == 1000){
    loopCounter = 0;
  }
  // Reload the WDTs RR[0] reload register
  // if this line isn't called at least every 2 seconds, the TIMEOUT event is called and the CPU is reset
  NRF_WDT->RR[0] = WDT_RR_RR_Reload;
}
/*
void loop() {

  for (int i = 0; i <= 25; ++i) {
    myHexapod.moveBodyCalc(legPositions, i, 0, 0, 0, 0, 0);
    myHexapod.moveLegs(legPositions);
    delay(5);
  }

  for (float i = 0.0; i <= 4 * PI; i += 0.05) {
    myHexapod.moveBodyCalc(legPositions, 25 * cos(i), 25 * sin(i), 10 * sin(i), 0, 0, 0);
    myHexapod.moveLegs(legPositions);
    delay(5);
  }
  for (int i = 25; i >= 0; --i) {
    myHexapod.moveBodyCalc(legPositions, i, 0, 0, 0, 0, 0);
    myHexapod.moveLegs(legPositions);
    delay(5);
  }
  for (int i = 0; i <= 30; ++i) {
    myHexapod.moveBodyCalc(legPositions, 0, 0, 0, i / 100.0, 0, 0);
    myHexapod.moveLegs(legPositions);
    delay(5);
  }
  delay(100);
  for (int i = 30; i >= -30; --i) {
    myHexapod.moveBodyCalc(legPositions, 0, 0, 0, i / 100.0, 0, 0);
    myHexapod.moveLegs(legPositions);
    delay(5);
  }
  delay(100);
  for (int i = -30; i <= 0; ++i) {
    myHexapod.moveBodyCalc(legPositions, 0, 0, 0, i / 100.0, 0, 0);
    myHexapod.moveLegs(legPositions);
    delay(5);
  }

  delay(100);
  for (int i = 0; i <= 30; ++i) {
    myHexapod.moveBodyCalc(legPositions, 0, 0, 0, 0, i / 100.0, 0);
    myHexapod.moveLegs(legPositions);
    delay(5);
  }
  delay(100);
  for (int i = 30; i >= -30; --i) {
    myHexapod.moveBodyCalc(legPositions, 0, 0, 0, 0, i / 100.0, 0);
    myHexapod.moveLegs(legPositions);
    delay(5);
  }
  delay(100);
  for (int i = -30; i <= 0; ++i) {
    myHexapod.moveBodyCalc(legPositions, 0, 0, 0, 0, i / 100.0, 0);
    myHexapod.moveLegs(legPositions);
    delay(5);
  }

  delay(100);
  for (int i = 0; i <= 30; ++i) {
    myHexapod.moveBodyCalc(legPositions, 0, 0, 0, 0, 0, i / 100.0);
    myHexapod.moveLegs(legPositions);
    delay(5);
  }
  delay(100);
  for (int i = 30; i >= -30; --i) {
    myHexapod.moveBodyCalc(legPositions, 0, 0, 0, 0, 0, i / 100.0);
    myHexapod.moveLegs(legPositions);
    delay(5);
  }
  delay(100);
  for (int i = -30; i <= 0; ++i) {
    myHexapod.moveBodyCalc(legPositions, 0, 0, 0, 0, 0, i / 100.0);
    myHexapod.moveLegs(legPositions);
    delay(5);
  }
  delay(100);

  for (int i = 0; i <= 100; ++i) {
    float c = map(i, 0, 100, 0, 220) / 1000.0;
    float d = map(i, 0, 100, 0, 300) / 1000.0;
    myHexapod.moveBodyCalc(legPositions, 0, 0, 0, c, c, d);
    myHexapod.moveLegs(legPositions);
    delay(5);
  }
  delay(100);
  for (int i = 100; i >= 0; --i) {
    float c = map(i, 0, 100, 0, 220) / 1000.0;
    float d = map(i, 0, 100, 0, 300) / 1000.0;
    myHexapod.moveBodyCalc(legPositions, 0, 0, 0, c, c, d);
    myHexapod.moveLegs(legPositions);
    delay(5);
  }
  delay(100);
}*/
