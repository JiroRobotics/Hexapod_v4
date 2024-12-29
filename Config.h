// Config.h

#ifndef Config_H
#define Config_H

// includes all constants necessary, i.e. pin numbers, measurements, etc

// measurements of the robot in mm or radians
// y distance from the center of the robot to the coxa joint of the corner legs in global coordinates
const float cornerLegYDistGlobal = 43.0;
// x distance from the center of the robot to the coxa joint of the corner legs in global coordinates
const float cornerLegXDistGlobal = 71.3;
// y distance from the center of the robot to the coxa joint of the middle legs in local (and global) coordinates
const float centerLegYDist = 53.0;
// angle of the corner legs relative to the middle legs
const float cornerLegAngle = PI / 6;

// array with all local coordinate system positions and orientations relative to the global coordinate system of the robot (x dist, y dist, angle)
const float legCoords[6][3] = { { cornerLegXDistGlobal, cornerLegYDistGlobal, cornerLegAngle },          // leg FR
                                { cornerLegXDistGlobal, -cornerLegYDistGlobal, 5 * cornerLegAngle },     // leg FL
                                { 0, centerLegYDist, 0 },                                                // leg MR
                                { 0, -centerLegYDist, 6 * cornerLegAngle },                              // leg ML
                                { -cornerLegXDistGlobal, cornerLegYDistGlobal, -cornerLegAngle },        // leg RR
                                { -cornerLegXDistGlobal, -cornerLegYDistGlobal, 7 * cornerLegAngle } };  // leg RL

// Length of the three links coxa, femur and tibia
const uint8_t coxaLength = 22;
const uint8_t femurLength = 46;
const uint8_t tibiaLength = 88;

// pin numbers for each leg (FL = front left, FR = front right, ML = mid left, RL = rear left)
const uint8_t coxaPinFR = 5;
const uint8_t femurPinFR = 6;
const uint8_t tibiaPinFR = 4;

const uint8_t coxaPinFL = 13;
const uint8_t femurPinFL = 14;
const uint8_t tibiaPinFL = 15;

const uint8_t coxaPinMR = 8;
const uint8_t femurPinMR = 9;
const uint8_t tibiaPinMR = 10;

const uint8_t coxaPinML = 11;
const uint8_t femurPinML = 8;
const uint8_t tibiaPinML = 9;

const uint8_t coxaPinRR = 12;
const uint8_t femurPinRR = 13;
const uint8_t tibiaPinRR = 14;

const uint8_t coxaPinRL = 7;
const uint8_t femurPinRL = 6;
const uint8_t tibiaPinRL = 5;

// offsets in degrees for each motor
const int8_t coxaOffsetFR = 1;
const int8_t femurOffsetFR = 0;
const int8_t tibiaOffsetFR = 4;

const int8_t coxaOffsetFL = 0;
const int8_t femurOffsetFL = -4;
const int8_t tibiaOffsetFL = 5;

const int8_t coxaOffsetMR = -2;
const int8_t femurOffsetMR = -4;
const int8_t tibiaOffsetMR = 8;

const int8_t coxaOffsetML = -10;
const int8_t femurOffsetML = -5;
const int8_t tibiaOffsetML = 4;

const int8_t coxaOffsetRR = -5;
const int8_t femurOffsetRR = -5;
const int8_t tibiaOffsetRR = -4;

const int8_t coxaOffsetRL = -2;
const int8_t femurOffsetRL = 0;
const int8_t tibiaOffsetRL = 5;

// pins of the push-buttons on every leg
const uint8_t buttonFR = 12;
const uint8_t buttonFL = 9;
const uint8_t buttonMR = 14;
const uint8_t buttonML = 10;
const uint8_t buttonRR = 8;
const uint8_t buttonRL = 11;

// home position of every leg. x,y,z - values
const float homePos[3] = { 0.0, 73.0, 80.0 };
//const float homePos[3] = {0.0, 68.0, 88.0}; // use this if you want 90° angles when at home position

// the loop is executed every [periodMs] milliseconds
const uint16_t periodMs = 10; // recommended for Arduino Nano 33 BLE Sense
//const uint16_t periodMS = 15; // recommended for Arduino Nano 33 IoT

const uint8_t pwm1Addr = 0x40;
const uint8_t pwm2Addr = 0x41;

#endif /* Config_H */