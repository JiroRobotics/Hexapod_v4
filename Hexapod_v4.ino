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
int legPositions[6][3] = { { (float)homePos[0], (float)homePos[1], (float)homePos[2] },    // front right
                           { (float)homePos[0], (float)homePos[1], (float)homePos[2] },    // front left
                           { (float)homePos[0], (float)homePos[1], (float)homePos[2] },    // mid right
                           { (float)homePos[0], (float)homePos[1], (float)homePos[2] },    // mid left
                           { (float)homePos[0], (float)homePos[1], (float)homePos[2] },    // rear right
                           { (float)homePos[0], (float)homePos[1], (float)homePos[2] } };  // rear left

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(buttonFR, INPUT);
  pinMode(buttonFL, INPUT);
  pinMode(buttonMR, INPUT);
  pinMode(buttonML, INPUT);
  pinMode(buttonRR, INPUT);
  pinMode(buttonRL, INPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.begin(9600);
  pwm1.begin();
  pwm1.setPWMFreq(50);
  pwm2.begin();
  pwm2.setPWMFreq(50);

  myHexapod.moveHome();
  delay(1000);
}
void loop() {
  // get the current time
  unsigned long timeMillis = millis();

  // calculate the new leg position
  // ...

  // update the leg position
  myHexapod.moveLegs(legPositions);
  while(millis() < timeMillis + periodMs){
    // wait a bit so that the loop is executet every 20ms
  }
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
