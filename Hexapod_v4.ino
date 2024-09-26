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
Leg legFrontRight = Leg(pwm2, coxaPinFR, femurPinFR, tibiaPinFR, coxaOffsetFR, femurOffsetFR, tibiaOffsetFR, true , buttonFR);
Leg legFrontLeft  = Leg(pwm1, coxaPinFL, femurPinFL, tibiaPinFL, coxaOffsetFL, femurOffsetFL, tibiaOffsetFL, false, buttonFL);
Leg legMidRight   = Leg(pwm2, coxaPinMR, femurPinMR, tibiaPinMR, coxaOffsetMR, femurOffsetMR, tibiaOffsetMR, true , buttonMR);
Leg legMidLeft    = Leg(pwm1, coxaPinML, femurPinML, tibiaPinML, coxaOffsetML, femurOffsetML, tibiaOffsetML, false, buttonML);
Leg legRearRight  = Leg(pwm2, coxaPinRR, femurPinRR, tibiaPinRR, coxaOffsetRR, femurOffsetRR, tibiaOffsetRR, true , buttonRR);
Leg legRearLeft   = Leg(pwm1, coxaPinRL, femurPinRL, tibiaPinRL, coxaOffsetRL, femurOffsetRL, tibiaOffsetRL, false, buttonRL);

// an array of pointers pointing to the six legs
Leg* legs[6] = {&legFrontRight, &legFrontLeft, &legMidRight, &legMidLeft, &legRearRight, &legRearLeft};

// Initialization of hexapod object
Hexapod myHexapod = Hexapod(legFrontRight, legFrontLeft, legMidRight, legMidLeft, legRearRight, legRearLeft);

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

  /*for(int i = 0; i < 6; ++i){
    legs[i]->moveTo(homePos[0], homePos[1], homePos[2]);
  }*/
  myHexapod.moveHome();
  delay(1000);

  
}

void loop() {
  
  for(int i = 0; i <=25; ++i){
    myHexapod.moveBody(i, 0, 0, 0, 0, 0);
    delay(2);
  }

  for(float i = 0.0; i <=4*PI; i += 0.05){
    myHexapod.moveBody(25 * cos(i), 25 * sin(i), 10*sin(i), 0, 0, 0);
    //delay(5);
  }
  for(int i = 25; i >=0; --i){
    myHexapod.moveBody(i, 0, 0, 0, 0, 0);
    delay(2);
  }
  /*
  long time = millis();

  for(int i = 0; i < 1000; ++i){
    myHexapod.moveBody(i/100.0, i/100.0, 10, i/10000.0, i/10000.0, i/100000.0);
  }

  time = millis() - time;
  Serial.print("Zeit: ");
  Serial.print(time);
  Serial.println("ms");
  Serial.print("Zeit pro Durchgang: ");
  Serial.print(time/1000.0);
  Serial.println("ms");
  delay(2000);*/

  
  for(int i = 0; i <= 30; ++i){
    myHexapod.moveBody(0, 0, 0, i/100.0, 0, 0);
    delay(1);
  }
  delay(100);
  for(int i = 30; i >=-30; --i){
    myHexapod.moveBody(0, 0, 0, i/100.0, 0, 0);
    delay(1);
  }
  delay(100);
  for(int i = -30; i <=0; ++i){
    myHexapod.moveBody(0, 0, 0, i/100.0, 0, 0);
    delay(1);
  }

  delay(100);
  for(int i = 0; i <=30; ++i){
    myHexapod.moveBody(0, 0, 0, 0, i/100.0, 0);
    delay(1);
  }
  delay(100);
  for(int i = 30; i >=-30; --i){
    myHexapod.moveBody(0, 0, 0, 0, i/100.0, 0);
    delay(1);
  }
  delay(100);
  for(int i = -30; i <=0; ++i){
    myHexapod.moveBody(0, 0, 0, 0, i/100.0, 0);
    delay(1);
  }

  delay(100);
  for(int i = 0; i <=30; ++i){
    myHexapod.moveBody(0, 0, 0, 0, 0, i/100.0);
    delay(1);
  }
  delay(100);
  for(int i = 30; i >=-30; --i){
    myHexapod.moveBody(0, 0, 0, 0, 0, i/100.0);
    delay(1);
  }
  delay(100);
  for(int i = -30; i <=0; ++i){
    myHexapod.moveBody(0, 0, 0, 0, 0, i/100.0);
    delay(1);
  }
  delay(100);

  for(int i = 0; i <=100; ++i){
    float c = map(i,0,100, 0,220) / 1000.0;
    float d = map(i,0,100, 0,300) / 1000.0;
    myHexapod.moveBody(0, 0, 0, c, c, d);
    delay(1);
  }
  delay(100);
  for(int i = 100; i >=0; --i){
    float c = map(i,0,100, 0,220) / 1000.0;
    float d = map(i,0,100, 0,300) / 1000.0;
    myHexapod.moveBody(0, 0, 0, c, c, d);
    delay(1);
  }
  delay(100);
  
  
  /*
  for(int i = 0; i <=40; ++i){
    myHexapod.moveBody(i, 0, 0, 0, 0, 0);
    delay(5);
  }
  delay(2000);
  for(int i = 40; i >=0; --i){
    myHexapod.moveBody(i, 0, 0, 0, 0, 0);
    delay(5);
  }
  delay(2000);
  for(int i = 0; i <=30; ++i){
    myHexapod.moveBody(0, i, 0, 0, 0, 0);
    delay(5);
  }
  delay(2000);
  for(int i = 30; i >=0; --i){
    myHexapod.moveBody(0, i, 0, 0, 0, 0);
    delay(5);
  }
  delay(2000);
  for(int i = 0; i <=20; ++i){
    myHexapod.moveBody(0, 0, i, 0, 0, 0);
    delay(5);
  }
  delay(2000);
  for(int i = 20; i >=0; --i){
    myHexapod.moveBody(0, 0, i, 0, 0, 0);
    delay(5);
  }
  delay(2000);
  */
  
}
