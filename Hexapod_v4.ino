// 4th generation Hexapod
// Hardware is largely the same as v3. Currently uses a Nano 33 BLE Sense Rev 2, but will also work on other capable
// Arduinos like the Nano 33 IoT

#include <Arduino_BMI270_BMM150.h>
#include <Adafruit_PWMServoDriver.h>
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
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  // Built in RGB is inverted
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_BLUE, HIGH);
  digitalWrite(LED_GREEN, HIGH);
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
    while (1);
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

  // calculate the new leg position

  // keep track of the number of steps
  if (myHexapod.getAction() == 0) {
    // increase the counter each time the robot is resting
    counter++;
    if (counter == 20) {
      counter = 0;
    }
  }

  // do five steps each in different directions
  if (counter < 5) {
    myHexapod.calcCrabwalkFlush(legPositions, currPositions, PI / 4, 30, 100);
  } else if (counter < 10) {
    myHexapod.calcCrabwalkFlush(legPositions, currPositions, 0, 30, 100);
  } else if (counter < 15) {
    myHexapod.calcCrabwalkFlush(legPositions, currPositions, PI, 30, 100);
  } else {
    myHexapod.calcCrabwalkFlush(legPositions, currPositions, 3*PI/4, 30, 100);
  }

  // Adjust the body position each loop() iteration
  myHexapod.calcBodyMovement(currPositions, newPositions, 0, 0, 0, 0.0, 0.0, 0.0);

  // update the leg position
  myHexapod.moveLegs(newPositions);
  
  // increment the loopCounter to keep track of the number of loop cycles
  loopCounter++;
  if (loopCounter == 2000) {
    loopCounter = 0;
  }
  if (loopCounter % 2) {      // turn the led on and of to show each loop iteration
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
  Serial.print("Millis before waiting: ");
  Serial.println(millis());
  while (millis() < timeMillis + periodMs) {
    // wait a bit so that the loop is executet every 20ms
  }
  Serial.print("Millis after waiting: ");
  Serial.println(millis());
  // Reload the WDTs RR[0] reload register
  // if this line isn't called at least every 2 seconds, the TIMEOUT event is called and the CPU is reset
  // uncomment this if the watchdog is used
  //NRF_WDT->RR[0] = WDT_RR_RR_Reload;
}
