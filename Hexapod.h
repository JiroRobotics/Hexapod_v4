// Hexapod.h
#ifndef Hexapod_H
#define Hexapod_H

#include "Leg.h"
#include "Config.h"

class Hexapod{
public:
  // Constructor. Aggregation of the six legs of the hexapod
  Hexapod(Leg &legFR, Leg &legFL, Leg &legMR, Leg &legML, Leg &legRR, Leg &legRL) : legFR(legFR), legFL(legFL), legMR(legMR), legML(legML), legRR(legRR), legRL(legRL){}

  // method to shift and rotate the body of the robot. Returns true if successful.
  // Old!!! use moveBody
  bool bodyMovement(int16_t xTrans, int16_t yTrans, int16_t zTrans,  float roll, float pitch, float yaw);

  // calculates the next position for each leg and passes them to newPositions[][] array
  void moveBodyCalc(int newPositions[6][3], int16_t xTrans, int16_t yTrans, int16_t zTrans, float roll, float pitch, float yaw);

  // calculates the next position for crab movement for all legs and passes them to newPositions[][]
  void crabCalc(int newPositions[6][3], uint16_t stepDist, float stepDirection);

  // method to instantly move all legs to the default (home) position. Returns true if successful
  bool moveHome();

  // method to instantly move the legs to their positions as specified in the array
  bool moveLegs(int positions[6][3]);

private:
  Leg &legFR;
  Leg &legFL;
  Leg &legMR;
  Leg &legML;
  Leg &legRR;
  Leg &legRL;

  // z-coordinate of the robots center of mass (equivalent to the height of the coxa joints if rotation is zero)
  uint8_t zHeight = 80;
  // total orientation of the robots local coordinate system relative to the global coordinate system
  float totalRoll = 0.0;
  float totalPitch = 0.0;
  float totalYaw = 0.0;
  int16_t totalxTrans = 0;
  int16_t totalyTrans = 0;
  int16_t totalzTrans = 0;
};

#endif /* Hexapod_H */