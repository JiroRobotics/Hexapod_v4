// 4th generation Hexapod
// Hardware is largely the same as v3 (white). Currently uses a Nano 33 IoT, but will likely be upgraded to use a Nano 33 BLE Sense v2
// Completely new software, each leg is represented as a class with references to each servo
#include <Arduino_BMI270_BMM150.h>
#include <Adafruit_PWMServoDriver.h>
#include <SpeedTrig.h>
#include "Config.h"
#include "Hexapod.h"
#include "Leg.h"

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

int currPositions[6][3];
int newPositions[6][3];
uint8_t counter = 0;

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

  // start the IMU
  if (!IMU.begin()) {
    Serial.println("IMU-Sensor couldn't be initialized!");
    while (1)
      ;
  }
  delay(1000);

  //Configure WDT.
  /*NRF_WDT->CONFIG = 0x01;    // Configure WDT to run when CPU is asleep
  NRF_WDT->CRV = 65535;      // Timeout set to 2 seconds, timeout[s] = (CRV+1)/32768
  NRF_WDT->RREN = 0x01;      // Enable the RR[0] reload register
  NRF_WDT->TASKS_START = 1;  // Start WDT*/
}
void loop() {
  // get the current time
  unsigned long timeMillis = millis();

  // calculate the new leg position
  // ...
  if (myHexapod.getAction() == 0) {
    counter++;
    if (counter == 20) {
      counter = 0;
    }

    float accX, accY, accZ;

    // read the acceleration
    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(accX, accY, accZ);

      // calculate the angle in radias
      float pitch = atan2(accY, accZ);
      float roll = atan2(accY, accX);
      pitch = pitch - PI / 2.0 + 0.04;
      roll -= PI / 2.0;

      myHexapod.calcBodyMovement(legPositions, legPositions, 0, 0, 0, roll, -pitch, 0.0);
    }
  }
  myHexapod.calcCrabwalk(legPositions, currPositions, 30, 0, 150, 35, true);


  /*if (counter % 5 == 2 || counter % 5 == 3 || counter % 5 == 4 || myHexapod.getAction() == 1) {
    myHexapod.calcCrabwalk(legPositions, currPositions, 30, 0, 100);
  } else if (counter % 5 == 0 || counter % 5 == 1 || myHexapod.getAction() == 2) {
    myHexapod.calcRotatingStep(legPositions, currPositions, 0.19, 100);
  }
  //myHexapod.calcCrabwalk(legPositions, currPositions, 20, dir, 100);

  if (loopCounter < 50) {
    int b = map(loopCounter, 0, 50, 0, 150);
    myHexapod.calcBodyMovement(currPositions, newPositions, 0, 0, 0, 0.0, b / 1000.0, 0.0);

  } else if (loopCounter < 150) {
    myHexapod.calcBodyMovement(currPositions, newPositions, 0, 0, 0, 0.0, 0.15, 0.0);

  } else if (loopCounter < 250) {
    int b = map(loopCounter, 150, 250, 150, -150);
    myHexapod.calcBodyMovement(currPositions, newPositions, 0, 0, 0, 0.0, b / 1000.0, 0.0);

  } else if (loopCounter < 350) {
    myHexapod.calcBodyMovement(currPositions, newPositions, 0, 0, 0, 0.0, -0.15, 0.0);

  } else if (loopCounter < 400) {
    int b = map(loopCounter, 350, 400, -150, 0);
    myHexapod.calcBodyMovement(currPositions, newPositions, 0, 0, 0, 0.0, b / 1000.0, 0.0);

  } else if (loopCounter < 450) {
    int b = map(loopCounter, 400, 450, 0, 150);
    myHexapod.calcBodyMovement(currPositions, newPositions, 0, 0, 0, b / 1000.0, 0.0, 0.0);

  } else if (loopCounter < 550) {
    int b = map(loopCounter, 450, 550, 150, -150);
    myHexapod.calcBodyMovement(currPositions, newPositions, 0, 0, 0, b / 1000.0, 0.0, 0.0);

  } else if (loopCounter < 600) {
    int b = map(loopCounter, 550, 600, -150, 0);
    myHexapod.calcBodyMovement(currPositions, newPositions, 0, 0, 0, b / 1000.0, 0.0, 0.0);

  } else if (loopCounter < 650) {
    int b = map(loopCounter, 600, 650, 0, 20);
    myHexapod.calcBodyMovement(currPositions, newPositions, 0, 0, b, 0.0, 0.0, 0.0);

  } else if (loopCounter < 750) {
    myHexapod.calcBodyMovement(currPositions, newPositions, 0, 0, 20, 0.0, 0.0, 0.0);

  } else if (loopCounter < 850) {
    int b = map(loopCounter, 750, 850, 20, -20);
    myHexapod.calcBodyMovement(currPositions, newPositions, 0, 0, b, 0.0, 0.0, 0.0);

  } else if (loopCounter < 950) {
    myHexapod.calcBodyMovement(currPositions, newPositions, 0, 0, -20, 0.0, 0.0, 0.0);

  } else if (loopCounter < 1000) {
    int b = map(loopCounter, 950, 1000, -20, 0);
    myHexapod.calcBodyMovement(currPositions, newPositions, 0, 0, b, 0.0, 0.0, 0.0);

  } else {
    myHexapod.calcBodyMovement(currPositions, newPositions, 0, 0, 0, 0.0, 0.0, 0.0);
  }*/

  myHexapod.calcBodyMovement(currPositions, newPositions, 0, 0, 10, 0.0, 0.0, 0.0);

  // update the leg position
  myHexapod.moveLegs(newPositions);
  //Serial.print("Zeit vor Pause: ");
  //Serial.println(millis());
  while (millis() < timeMillis + periodMs) {
    // wait a bit so that the loop is executet every 20ms
  }
  // increment the loopCounter to keep track of the number of loop cycles
  loopCounter++;
  if (loopCounter == 2000) {
    loopCounter = 0;
  }
  if (loopCounter % 2) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
  //Serial.print("Zeit nach Pause: ");
  //Serial.println(millis());
  // Reload the WDTs RR[0] reload register
  // if this line isn't called at least every 2 seconds, the TIMEOUT event is called and the CPU is reset
  //NRF_WDT->RR[0] = WDT_RR_RR_Reload;
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
