// Leg.cpp

#include "Leg.h"
//#define useFloat

void Leg::setAngle(uint8_t servoPin, int offset, int angleVal) {
  /*
   * sets the angle of a certain servo of the leg to the desired value. All angles in degrees.
   * uint8_t servoPin:  the pin (0-15) of the servo on the pwm module
   * int angle:         the angle (0 - 180) to move the servo to
   *
   */

  servoDriver.setPWM(servoPin, 0, angle(angleVal + offset));
}

bool Leg::moveTo(int xValue, int yValue, int zValue) {
  /*
   * moves the leg to the given x y z coordinates (in local coordinate system)
   * int xValue:    the x-value (left/right) in mm to move the legpoint to
   * int yValue:    the y-value (back/forth) in mm to move the legpoint to
   * int zValue:    the z-value (height) in mm to move the legpoint to
   */

  // flag rises when leg can't reach the desired position
  bool flag = true;
  //calculate the coxa servo angle using atan2. Mirror the result due to orientation of the servo
  //calculate 2 helper variables L1 and L. Used to calculate the two other angles
  float L1 = sqrt((float)xValue * xValue + yValue * yValue);
  float L = sqrt((float)zValue * zValue + (L1 - coxaLength) * (L1 - coxaLength));

#if defined(useFloat)
  float gamma = atan2(yValue, xValue) * 180 / PI;
  gamma = mapFloat(gamma, 0.0, 180.0, 180.0, 0.0);

  // angle for femur servo
  float alpha = (acos(zValue / L) + acos((tibiaLength * tibiaLength - femurLength * femurLength - L * L) / (-2 * femurLength * L))) * 180 / PI;

  // angle for tibia servo
  float beta = (acos((L * L - tibiaLength * tibiaLength - femurLength * femurLength) / (-2 * tibiaLength * femurLength))) * 180 / PI;

  // mirrors alpha and beta if the leg is on the right side of the robot
  if (rightSide) {
    alpha = mapFloat(alpha, 0.0, 180.0, 180.0, 0.0);
    beta = mapFloat(beta, 0.0, 180.0, 180.0, 0.0);
  }

  // checks whether the angle is in the allowed interval of [40°, 130°] and then sets the coxa servo to the desired angle
  if ((gamma + offsetCoxa) > 40 && (gamma + offsetCoxa) < 130) {
    servoDriver.setPWM(pinCoxa, 0, angleFloat(gamma + offsetCoxa));
  } else {
    flag = false;
  }

  // checks the same for femur and tibia servos
  if (L <= (femurLength + tibiaLength - 5)) {
    if (alpha + offsetTibia <= 134) {
      servoDriver.setPWM(pinFemur, 0, angleFloat(alpha + offsetFemur));
    } else {
      flag = false;
    }
    servoDriver.setPWM(pinTibia, 0, angleFloat(beta + offsetTibia));
  } else {
    flag = false;
  }
#else
  int gamma = SpeedTrig.atan2(yValue, xValue) * 180 / PI;
  gamma = map(gamma, 0, 180, 180, 0);

  // angle for femur servo
  int alpha = (SpeedTrig.acos(zValue / L) + SpeedTrig.acos((tibiaLength * tibiaLength - femurLength * femurLength - L * L) / (-2 * femurLength * L))) * 180 / PI;
  // angle for tibia servo
  int beta = (SpeedTrig.acos((L * L - tibiaLength * tibiaLength - femurLength * femurLength) / (-2 * tibiaLength * femurLength))) * 180 / PI;

  // mirrors alpha and beta if the leg is on the right side of the robot
  if (rightSide) {
    alpha = map(alpha, 0, 180, 180, 0);
    beta = map(beta, 0, 180, 180, 0);
  }

  // checks whether the angle is in the allowed interval of [40°, 130°] and then sets the coxa servo to the desired angle
  if ((gamma + offsetCoxa) > 40 && (gamma + offsetCoxa) < 130) {
    servoDriver.setPWM(pinCoxa, 0, angle(gamma + offsetCoxa));
  } else {
    flag = false;
  }

  // checks the same for femur and tibia servos
  if (L <= (femurLength + tibiaLength - 5)) {
    if (alpha + offsetTibia <= 134) {
      servoDriver.setPWM(pinFemur, 0, angle(alpha + offsetFemur));
    } else {
      flag = false;
    }
    servoDriver.setPWM(pinTibia, 0, angle(beta + offsetTibia));
  } else {
    flag = false;
  }
#endif

  // save the current position of the leg
  currPos[0] = xValue;
  currPos[1] = yValue;
  currPos[2] = zValue;

  // returns true if nothing went wrong, returns false if a servo couldn't reach the position
  return flag;
}

[[nodiscard]] int Leg::getXVal() {
  // returns the last x-Value in local coordinates
  return currPos[0];
}

[[nodiscard]] int Leg::getYVal() {
  // returns the last y-Value in local coordinates
  return currPos[1];
}

[[nodiscard]] int Leg::getZVal() {
  // returns the last y-Value in local coordinates
  return currPos[2];
}

[[nodiscard]] int Leg::angle(uint8_t pwmAngle) {
  /*
   * Converts an angle between 0° and 180° to the corresponding number of PWM ticks.
   * The min/max values (105/496) have to be tested 
   * pwmAngle:    the angle in degrees to be converted
   * returns:     the number of PWM ticks corresponding to the angle
   */

  // constrain the input to [0,180]. Avoids faulty outputs
  pwmAngle = constrain(pwmAngle, 0, 180);

  // map the angle in degrees to the PWM tick count
  int pwmValue = map(pwmAngle, 0, 180, 105, 496);

  return pwmValue;
}

[[nodiscard]] int Leg::angleFloat(float pwmAngle) {
  /*
   * Converts an angle between 0° and 180° to the corresponding number of PWM ticks.
   * The min/max values (105/496) have to be tested 
   * pwmAngle:    the angle in degrees to be converted
   * returns:     the number of PWM ticks corresponding to the angle
   */

  // constrain the input to [0,180]. Avoids faulty outputs
  pwmAngle = constrain(pwmAngle, 0.0, 180.0);

  // map the angle in degrees to the PWM tick count
  float result = mapFloat(pwmAngle, 0.0, 180.0, 105.0, 496.0);
  int pwmValue = round(result);

  return pwmValue;
}

float Leg::mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
