// Leg.h
#ifndef Leg_H
#define Leg_H

#include <Adafruit_PWMServoDriver.h>
#include <SpeedTrig.h>
#include "Config.h"

class Leg {
public:
  // Constructor. Arguments are reference to PWM servo driver module, the 3 servos + according offsets and the push-button
  Leg(Adafruit_PWMServoDriver &servoDriver, uint8_t pinCoxa, uint8_t pinFemur, uint8_t pinTibia, int offsetCoxa, int offsetFemur, int offsetTibia, bool rightSide, uint8_t pinPushButton)
    : servoDriver(servoDriver), pinCoxa(pinCoxa), pinFemur(pinFemur), pinTibia(pinTibia), offsetCoxa(offsetCoxa), offsetFemur(offsetFemur), offsetTibia(offsetTibia), rightSide(rightSide), pinPushButton(pinPushButton) {}

  void setAngle(uint8_t servoPin, int offset, int angleVal);

  // moves the leg instantly to the desired position. Returns false if a servo couldn't reach target position
  bool moveTo(int xValue, int yValue, int zValue);

  // getter-functions for the current x-y-z-Values where the point of the leg is at
  [[nodiscard]] int getXVal();
  [[nodiscard]] int getYVal();
  [[nodiscard]] int getZVal();

private:
  // Reference to servo driver module
  Adafruit_PWMServoDriver &servoDriver;
  // pins on the servo driver board
  const uint8_t pinCoxa;
  const uint8_t pinFemur;
  const uint8_t pinTibia;
  // offset in degrees for each servo
  const int offsetCoxa;
  const int offsetFemur;
  const int offsetTibia;
  // true if leg is on the right side of the robot. False if on the left (mirrored)
  const bool rightSide;
  //pin for the push-button indicating whether the leg touches the ground
  const uint8_t pinPushButton;

  //stores the current position (x, y, z)
  int currPos[3];

  [[nodiscard]] int angle(uint8_t pwmAngle);

  [[nodiscard]] int angleFloat(float pwmAngle);

  float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
};

#endif /* Leg_H */