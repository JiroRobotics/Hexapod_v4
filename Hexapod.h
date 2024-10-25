// Hexapod.h
#ifndef Hexapod_H
#define Hexapod_H

#include "Leg.h"
#include "Config.h"

class Hexapod {
public:
  // Constructor. Aggregation of the six legs of the hexapod
  Hexapod(Leg &legFR, Leg &legFL, Leg &legMR, Leg &legML, Leg &legRR, Leg &legRL)
    : legFR(legFR), legFL(legFL), legMR(legMR), legML(legML), legRR(legRR), legRL(legRL) {}

  // calculates the next position for each leg and passes them to newPositions[][] array
  void calcBodyMovement(int prevPositions[6][3], int newPositions[6][3], int16_t xTrans, int16_t yTrans, int16_t zTrans, float roll, float pitch, float yaw);

  // calculates the next position for crab movement for all legs and passes them to newPositions[][]
  bool calcCrabwalk(int prevPositions[6][3], int newPositions[6][3], uint16_t stepDist, float stepDirection, uint16_t stepNumber, uint8_t stepHeight = 20, bool useOffroad = false);

  // calculates the next position for rotating on the spot and passes them to newPositions[][]
  bool calcRotatingStep(int prevPositions[6][3], int newPositions[6][3], float stepAngle, uint16_t stepNumber, uint8_t stepHeight = 20);

  // calculates the next position for a step (forwards or backwards) with a specified turning radius
  bool calcRadiusStep(int prevPositions[6][3], int newPositions[6][3], uint8_t stepDist, float stepAngle, bool direction, uint16_t stepNumber, uint8_t stepHeight = 20);

  // method to instantly move all legs to the default (home) position. Returns true if successful
  bool moveHome();

  // method to instantly move the legs to their positions as specified in the array
  bool moveLegs(int positions[6][3]);

  // returns the action which the hexapod performs at the moment. 0 = sleeping / doing nothing. 1 = doing a crabwalk step. 2 = rotating on the spot
  [[nodiscard]] uint8_t getAction();

private:
  Leg &legFR;
  Leg &legFL;
  Leg &legMR;
  Leg &legML;
  Leg &legRR;
  Leg &legRL;

  // variable representing the current action of the hexapod. 
  // 0 = sleeping / doing nothing
  // 1 = doing a crab walk step
  // 2 = rotating on the spot
  // 3 = step with radius
  uint8_t action = 0;

  // z-coordinate of the robots center of mass (equivalent to the height of the coxa joints if rotation is zero)
  uint8_t zHeight = 80;
  // total orientation of the robots local coordinate system relative to the global coordinate system
  float averageHeight = 0.0f;

  // counter to keep track of the number of iterations in calcCrabwalk()
  uint16_t stepCounter = 1;
  // indicator whether to move legs FR, ML, RR or legs FL, MR, RL
  bool rightLegHome = true;
  bool leftLegHome = true;
  bool moveRightLeg = true;
  // array to save the end points of a crab walk step
  int finalPositions[6][3] = { { homePos[0], homePos[1], homePos[2] },    // front right
                               { homePos[0], homePos[1], homePos[2] },    // front left
                               { homePos[0], homePos[1], homePos[2] },    // mid right
                               { homePos[0], homePos[1], homePos[2] },    // mid left
                               { homePos[0], homePos[1], homePos[2] },    // rear right
                               { homePos[0], homePos[1], homePos[2] } };  // rear left

};

#endif /* Hexapod_H */