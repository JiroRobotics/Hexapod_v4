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
  void calcBodyMovement(int prevPositions[6][3], int newPositions[6][3], int16_t xTrans, int16_t yTrans, int16_t zTrans, float roll, float pitch, float yaw, uint8_t legMask = 0b111111);
  
  // calculates the leg end point coordinates and stores them in newPositions[][] for executing a step
  bool calcStep(int prevPositions[6][3], int newPositions[6][3], float stepDirection, uint8_t radius, uint16_t stepNumber, float overlayRotation = 0.0, uint8_t stepHeight = 20);

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
  // 5 = new CrabWalk
  uint8_t action = 0;

  // counter to keep track of the number of iterations in calcCrabwalk()
  uint16_t stepCounter = 1;
  // indicator whether to move legs FR, ML, RR or legs FL, MR, RL
  bool moveRightLeg = true;

  float prevDirection = 0.0;
  bool prevRightLeg = true;

  // array to save the end points of a crab walk step
  int finalPositions[6][3] = { { homePos[0], homePos[1], homePos[2] },    // front right
                               { homePos[0], homePos[1], homePos[2] },    // front left
                               { homePos[0], homePos[1], homePos[2] },    // mid right
                               { homePos[0], homePos[1], homePos[2] },    // mid left
                               { homePos[0], homePos[1], homePos[2] },    // rear right
                               { homePos[0], homePos[1], homePos[2] } };  // rear left

  void lineCircleIntersect(int mX, int mY, int radius, int pX, int pY, float direction, int intersections[2][2]);

  int getOppositeIntersection(int pX, int pY, float direction, int intersections[2][2]);
    
  void interpolateStep(int newPositions[6][3], int prevPositions[6][3], int finalPositions[6][3], uint8_t stepHeight, uint16_t stepCounter, uint16_t stepNumber, bool moveRightLeg, bool moveAllLegs = true);
};

#endif /* Hexapod_H */