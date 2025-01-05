// Hexapod.h
#ifndef Hexapod_H
#define Hexapod_H

#include "Leg.h"
#include "Config.h"
#include <Arduino_BMI270_BMM150.h>
#include "SpeedTrig.h"

class Hexapod {
public:
  // Constructor. Aggregation of the six legs of the hexapod
  Hexapod(Leg &legFR, Leg &legFL, Leg &legMR, Leg &legML, Leg &legRR, Leg &legRL)
    : legFR(legFR), legFL(legFL), legMR(legMR), legML(legML), legRR(legRR), legRL(legRL) {}

  // calculates the next position for each leg and passes them to newPositions[][] array
  void calcBodyMovement(float prevPositions[6][3], float newPositions[6][3], float xTrans, float yTrans, float zTrans, float roll, float pitch, float yaw, uint8_t legMask = 0b111111);
  
  // calculates the leg end point coordinates and stores them in newPositions[][] for executing a step
  bool calcStep(float prevPositions[6][3], float newPositions[6][3], float stepDirection, uint8_t radius, uint16_t stepNumber, float overlayRotation = 0.0, uint8_t stepHeight = 20);

  // method to instantly move all legs to the default (home) position. Returns true if successful
  bool moveHome();

  // method to instantly move the legs to their positions as specified in the array
  bool moveLegs(float positions[6][3]);

  // wrapper method to perform a simple step with the same parameters as calcStep()
  void step(float legPos[6][3], float stepDirection, uint8_t radius, uint16_t stepNumber, float overlayRotation = 0.0, uint8_t stepHeight = 20);

  // method to travel along a path specified by the points in array
  bool travelPath(float legPos[6][3], int16_t pathPoints[][2], uint8_t numPoints, uint16_t iterationNumber = 120, uint8_t pathType = 0, float maxRotationPerStep = 0.2, uint8_t stepHeight = 20);

  // returns the action which the hexapod performs at the moment. 0 = sleeping / doing nothing. 1 = doing a crabwalk step. 2 = rotating on the spot
  [[nodiscard]] uint8_t getAction();

  // getters for the pose of the robot in the global coordinate system
  [[nodiscard]] float getGlobalOrientation();
  [[nodiscard]] float getGlobalXPos();
  [[nodiscard]] float getGlobalYPos();

  // sets the start point of the Hexapod in the global coordinate system. Defaults to 0 if this method isn't called
  void setStartPoint(float xGlobal = 0.0, float yGlobal = 0.0, float heading = 0.0);

  // set the heading of the robot relative to the global coordinate system (the starting position)
  void setHeading(float heading = 0.0);
  
  // starts the IMU of the Arduino board
  bool startIMU();

  // calibrates the IMU (calculates and saves offsets)
  void calibrateIMU(uint16_t numSamples = 1000);

  // reads the current roll and pitch angles (rad) to the passed parameters
  bool readRollPitch(float &roll, float &pitch);

  bool balance(float legPos[6][3], float newPos[6][3]);
private:
  Leg &legFR;
  Leg &legFL;
  Leg &legMR;
  Leg &legML;
  Leg &legRR;
  Leg &legRL;

  // variable representing the current action of the hexapod.
  // 0 = sleeping / doing nothing
  // 1 = doing a step
  uint8_t action = 0;

  // counter to keep track of the number of iterations in calcCrabwalk()
  uint16_t stepCounter = 1;
  // indicator whether to move legs FR, ML, RR or legs FL, MR, RL
  bool moveRightLeg = true;

  // keep track of the previous step in order to determine which legs to lift
  float prevDirection = 0.0;
  bool prevRightLeg = true;

  // the following describes the (2D) pose of the base coordinate system and the global coordinate system
  float globalOrientation = 0.0;
  float globalXPosition = 0.0;
  float globalYPosition = 0.0;

  // variables for accuracy of steps (e.g. whether a theoretical step length of 50mm will actually result in movement of 50mm in global coordinate frame or only 50mm*0.9)
  float rotateAccuracy = 0.9;
  float lengthAccuracy = 0.95;

  // variables for calibrating IMU
  // IMU pitch offset angle (rad)
  float pitchOffset = 0.0;
  // IMU roll offset angle (rad)
  float rollOffset = 0.0;

  // variables for IIR filter
  // saves previous roll
  float prevRoll = 0.0;
  float prevPitch = 0.0;

  //save previous roll pitch variables for control loop
  // previous roll passed to calcBodyMovement in balance()
  float prevRollOutput = 0.0;
  // previous pitch passed to calcBodyMovement in balance()
  float prevPitchOutput = 0.0;

  // array to save the end points of a crab walk step
  float finalPositions[6][3] = { { homePos[0], homePos[1], homePos[2] },    // front right
                                 { homePos[0], homePos[1], homePos[2] },    // front left
                                 { homePos[0], homePos[1], homePos[2] },    // mid right
                                 { homePos[0], homePos[1], homePos[2] },    // mid left
                                 { homePos[0], homePos[1], homePos[2] },    // rear right
                                 { homePos[0], homePos[1], homePos[2] } };  // rear left

  // used in calcStep
  void lineCircleIntersect(float mX, float mY, int radius, float pX, float pY, float direction, int intersections[2][2]);

  // used in calcStep
  int getOppositeIntersection(float pX, float pY, float direction, int intersections[2][2]);
  
  // used in calcStep
  void interpolateStep(float newPositions[6][3], float prevPositions[6][3], float finalPositions[6][3], uint8_t stepHeight, uint16_t stepCounter, uint16_t stepNumber, bool moveRightLeg, bool moveAllLegs = true);

  // map() only works with integers, not floating point numbers
  float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
};

#endif /* Hexapod_H */