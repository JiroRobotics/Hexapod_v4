#include "SpeedTrig.h"
// Hexapod.cpp

#include "Hexapod.h"

bool Hexapod::moveHome() {
  /* 
   * moves all legs instantly to the home position.
   * returns true if successful
   */
  if (legFR.moveTo(homePos[0], homePos[1], homePos[2]) && legFL.moveTo(homePos[0], homePos[1], homePos[2])
      && legMR.moveTo(homePos[0], homePos[1], homePos[2]) && legML.moveTo(homePos[0], homePos[1], homePos[2])
      && legRR.moveTo(homePos[0], homePos[1], homePos[2]) && legRL.moveTo(homePos[0], homePos[1], homePos[2])) {
    return true;
  } else {
    return false;
  }
}

void Hexapod::calcBodyMovement(int prevPositions[6][3], int newPositions[6][3], int16_t xTrans, int16_t yTrans, int16_t zTrans, float roll, float pitch, float yaw, uint8_t legMask) {
  /*
   * calculates the next position for each leg in local x-y-z coordinates to achieve the specified translation and rotation
   * prevPositions: 6x3 array to get the leg position without translation and rotation
   * newPositions:  6x3 array to store the calculated coordinates in
   * xTrans:        distance (in mm) to move the center of the robot in x direction (global coordinates) (back/forth)
   * yTrans:        distance (in mm) to move the center of the robot in y direction (global coordinates) (sideways)
   * zTrans:        distance (in mm) to move the center of the robot in z direction (global coordinates) (up/down)
   * roll:          angle (in rad) to rotate around the x-axis
   * pitch:         angle (in rad) to rotate around the y-axis
   * yaw:           angle (in rad) to rotate around the z-axis
   * moveMask:      specifies which legs are moved. By default, all legs are moved (0b111111) 
   *
   * returns:     nothing
   *
   * runtime on Arduino Nano 33 IoT: <5.65ms (default), <8.2ms (useFloat)
   * runtime on Arduino Nano 33 BLE Sense: <3.85ms (default), <3.95ms (useFloat) (<- this is what a FPU is made for)
   */

  // code...
  // copy the previous positions (without translation and rotation) in the newPositions array
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 3; ++j) {
      newPositions[i][j] = prevPositions[i][j];
    }
  }

  // complete rotational matrix for roll pitch yaw angles. See https://de.wikipedia.org/wiki/Roll-Nick-Gier-Winkel
  const float rotMatrix[3][3] = { { cos(pitch) * cos(yaw), sin(roll) * sin(pitch) * cos(yaw) - cos(roll) * sin(yaw), cos(roll) * sin(pitch) * cos(yaw) + sin(roll) * sin(yaw) },
                                  { cos(pitch) * sin(yaw), sin(roll) * sin(pitch) * sin(yaw) + cos(roll) * cos(yaw), cos(roll) * sin(pitch) * sin(yaw) - sin(roll) * cos(yaw) },
                                  { -sin(pitch), sin(roll) * cos(pitch), cos(roll) * cos(pitch) } };
  // array to save the rotation result temporarily
  //float result[3] = { 0.0, 0.0, 0.0 };

  // new for-loop
  for (uint8_t k = 0; k < 6; ++k) {  // iterate over all legs

    // check if the leg should move (0b100000 is FR, 0b010000 is FL, ... 0b000001 is RL)
    if (legMask & (1 << (5 - k))) {

      // rotate the local coordinate system by the angle specified in legCoords[][]
      int temp = newPositions[k][0];
      newPositions[k][0] = newPositions[k][0] * cos(-legCoords[k][2]) - newPositions[k][1] * sin(-legCoords[k][2]);
      newPositions[k][1] = temp * sin(-legCoords[k][2]) + newPositions[k][1] * cos(-legCoords[k][2]);

      //shift the local coordinate system to the center of mass
      newPositions[k][0] += legCoords[k][0];
      newPositions[k][1] += legCoords[k][1];

      // translation of leg end point
      newPositions[k][0] -= xTrans;
      newPositions[k][1] -= yTrans;
      newPositions[k][2] += zTrans;

      // rotating the end point using RotMatrix
      float result[3] = { 0.0, 0.0, 0.0 };

      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          result[i] += (rotMatrix[i][j] * newPositions[k][j]);
        }
      }
      for (int i = 0; i < 3; ++i) {
        newPositions[k][i] = result[i];
      }

      //shift the local coordinate system back
      newPositions[k][0] -= legCoords[k][0];
      newPositions[k][1] -= legCoords[k][1];

      //rotate the coordinate system back by the mounting angle of the leg
      temp = newPositions[k][0];
      newPositions[k][0] = newPositions[k][0] * cos(legCoords[k][2]) - newPositions[k][1] * sin(legCoords[k][2]);
      newPositions[k][1] = temp * sin(legCoords[k][2]) + newPositions[k][1] * cos(legCoords[k][2]);
    }
  }
  /*
  // ++++++++++++++++++++++++++++++
  // leg mid right
  // ++++++++++++++++++++++++++++++
  if (legMask & 0b001000) {  // leg MR corresponds to the third bit of the bitmask (counting from MSB to LSB)
    // transformation from local to global coordinate system:
    newPositions[2][1] += centerLegYDist;
    // translation of leg end point
    newPositions[2][0] -= xTrans;
    newPositions[2][1] -= yTrans;
    newPositions[2][2] += zTrans;

    // rotating the end point using RotMatrix
    for (int i = 0; i < 3; ++i) {
      result[i] = 0.0;  //set the result array to 0
    }

    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        result[i] += (rotMatrix[i][j] * newPositions[2][j]);  // matrix-vector-multiplication: rotMatrix * (x, y, z)
      }
    }
    for (int i = 0; i < 3; ++i) {
      newPositions[2][i] = result[i];
    }

    // transformation back to the local coordinate system
    newPositions[2][1] -= centerLegYDist;
  }

  // ++++++++++++++++++++++++++++++
  // leg mid left
  // ++++++++++++++++++++++++++++++
  if (legMask & 0b000100) {  // leg ML corresponds to the fourth bit of the bitmask
    // transformation from global to local coordinate system:
    //shift
    newPositions[3][1] += centerLegYDist;
    //and rotate by 180°
    newPositions[3][0] = -newPositions[3][0];
    newPositions[3][1] = -newPositions[3][1];

    // translation of leg end point
    newPositions[3][0] -= xTrans;
    newPositions[3][1] -= yTrans;
    newPositions[3][2] += zTrans;

    // rotating the end point using RotMatrix
    for (int i = 0; i < 3; ++i) {
      result[i] = 0.0;
    }

    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        result[i] += (rotMatrix[i][j] * newPositions[3][j]);
      }
    }
    for (int i = 0; i < 3; ++i) {
      newPositions[3][i] = result[i];
    }

    // transformation back to the local coordinate system
    //rotate back by -180°
    newPositions[3][0] = -newPositions[3][0];
    newPositions[3][1] = -newPositions[3][1];
    // shift back
    newPositions[3][1] -= centerLegYDist;
  }

  // ++++++++++++++++++++++++++++++
  // leg front right
  // ++++++++++++++++++++++++++++++
  if (legMask & 0b100000) {  // leg FR corresponds to the MSB bit of the bitmask
    // rotate the local coordinate system by 30°
    int temp = newPositions[0][0];
    newPositions[0][0] = newPositions[0][0] * cos(-cornerLegAngle) - newPositions[0][1] * sin(-cornerLegAngle);
    newPositions[0][1] = temp * sin(-cornerLegAngle) + newPositions[0][1] * cos(-cornerLegAngle);

    //shift the local coordinate system to the center of mass
    newPositions[0][0] += cornerLegXDistGlobal;
    newPositions[0][1] += cornerLegYDistGlobal;

    // translation of leg end point
    newPositions[0][0] -= xTrans;
    newPositions[0][1] -= yTrans;
    newPositions[0][2] += zTrans;

    // rotating the end point using RotMatrix
    for (int i = 0; i < 3; ++i) {
      result[i] = 0.0;
    }

    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        result[i] += (rotMatrix[i][j] * newPositions[0][j]);
      }
    }
    for (int i = 0; i < 3; ++i) {
      newPositions[0][i] = result[i];
    }

    //shift the local coordinate system back
    newPositions[0][0] -= cornerLegXDistGlobal;
    newPositions[0][1] -= cornerLegYDistGlobal;

    //rotate the coordinate system back by -30°
    temp = newPositions[0][0];
    newPositions[0][0] = newPositions[0][0] * cos(cornerLegAngle) - newPositions[0][1] * sin(cornerLegAngle);
    newPositions[0][1] = temp * sin(cornerLegAngle) + newPositions[0][1] * cos(cornerLegAngle);
  }

  // ++++++++++++++++++++++++++++++
  // leg rear right
  // ++++++++++++++++++++++++++++++
  if (legMask & 0b000010) {  // leg RR corresponds to the fifth bit of the bitmask
    // rotate the local coordinate system by -30°
    int temp = newPositions[4][0];
    newPositions[4][0] = newPositions[4][0] * cos(cornerLegAngle) - newPositions[4][1] * sin(cornerLegAngle);
    newPositions[4][1] = temp * sin(cornerLegAngle) + newPositions[4][1] * cos(cornerLegAngle);

    //shift the local coordinate system to the center of mass
    newPositions[4][0] -= cornerLegXDistGlobal;
    newPositions[4][1] += cornerLegYDistGlobal;

    // translation of leg end point
    newPositions[4][0] -= xTrans;
    newPositions[4][1] -= yTrans;
    newPositions[4][2] += zTrans;

    // rotating the end point using RotMatrix
    for (int i = 0; i < 3; ++i) {
      result[i] = 0.0;
    }

    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        result[i] += (rotMatrix[i][j] * newPositions[4][j]);
      }
    }
    for (int i = 0; i < 3; ++i) {
      newPositions[4][i] = result[i];
    }

    //shift the local coordinate system back
    newPositions[4][0] += cornerLegXDistGlobal;
    newPositions[4][1] -= cornerLegYDistGlobal;

    //rotate the coordinate system back by 30°
    temp = newPositions[4][0];
    newPositions[4][0] = newPositions[4][0] * cos(-cornerLegAngle) - newPositions[4][1] * sin(-cornerLegAngle);
    newPositions[4][1] = temp * sin(-cornerLegAngle) + newPositions[4][1] * cos(-cornerLegAngle);
  }

  // ++++++++++++++++++++++++++++++
  // leg front left
  // ++++++++++++++++++++++++++++++
  if (legMask & 0b010000) {  // leg FL corresponds to the second bit of the bitmask
    // rotate the local coordinate system by 180°-30°
    int temp = newPositions[1][0];
    newPositions[1][0] = newPositions[1][0] * cos(-PI + cornerLegAngle) - newPositions[1][1] * sin(-PI + cornerLegAngle);
    newPositions[1][1] = temp * sin(-PI + cornerLegAngle) + newPositions[1][1] * cos(-PI + cornerLegAngle);

    //shift the local coordinate system to the center of mass
    newPositions[1][0] += cornerLegXDistGlobal;
    newPositions[1][1] -= cornerLegYDistGlobal;

    // translation of leg end point
    newPositions[1][0] -= xTrans;
    newPositions[1][1] -= yTrans;
    newPositions[1][2] += zTrans;

    // rotating the end point using RotMatrix
    for (int i = 0; i < 3; ++i) {
      result[i] = 0.0;
    }

    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        result[i] += (rotMatrix[i][j] * newPositions[1][j]);
      }
    }
    for (int i = 0; i < 3; ++i) {
      newPositions[1][i] = result[i];
    }

    //shift the local coordinate system back
    newPositions[1][0] -= cornerLegXDistGlobal;
    newPositions[1][1] += cornerLegYDistGlobal;

    //rotate the coordinate system back by -180°+30°
    temp = newPositions[1][0];
    newPositions[1][0] = newPositions[1][0] * cos(PI - cornerLegAngle) - newPositions[1][1] * sin(PI - cornerLegAngle);
    newPositions[1][1] = temp * sin(PI - cornerLegAngle) + newPositions[1][1] * cos(PI - cornerLegAngle);
  }

  // ++++++++++++++++++++++++++++++
  // leg rear left
  // ++++++++++++++++++++++++++++++
  if (legMask & 0b000001) {  // leg FL corresponds to the LSB of the bitmask
    // rotate the local coordinate system by 180°+30°
    int temp = newPositions[5][0];
    newPositions[5][0] = newPositions[5][0] * cos(-PI - cornerLegAngle) - newPositions[5][1] * sin(-PI - cornerLegAngle);
    newPositions[5][1] = temp * sin(-PI - cornerLegAngle) + newPositions[5][1] * cos(-PI - cornerLegAngle);

    //shift the local coordinate system to the center of mass
    newPositions[5][0] -= cornerLegXDistGlobal;
    newPositions[5][1] -= cornerLegYDistGlobal;

    // translation of leg end point
    newPositions[5][0] -= xTrans;
    newPositions[5][1] -= yTrans;
    newPositions[5][2] += zTrans;

    // rotating the end point using RotMatrix
    for (int i = 0; i < 3; ++i) {
      result[i] = 0.0;
    }

    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        result[i] += (rotMatrix[i][j] * newPositions[5][j]);
      }
    }
    for (int i = 0; i < 3; ++i) {
      newPositions[5][i] = result[i];
    }

    //shift the local coordinate system back
    newPositions[5][0] += cornerLegXDistGlobal;
    newPositions[5][1] += cornerLegYDistGlobal;

    //rotate the coordinate system back by -180°-30°
    temp = newPositions[5][0];
    newPositions[5][0] = newPositions[5][0] * cos(PI + cornerLegAngle) - newPositions[5][1] * sin(PI + cornerLegAngle);
    newPositions[5][1] = temp * sin(PI + cornerLegAngle) + newPositions[5][1] * cos(PI + cornerLegAngle);
  }*/
}

bool Hexapod::calcStep(int prevPositions[6][3], int newPositions[6][3], float stepDirection, uint8_t radius, uint16_t stepNumber, float overlayRotation, uint8_t stepHeight) {
  /*
   * Calculates the position of the leg end point in local coordinates for all legs during a step.
   * One set of legs moves the robot, the other set moves to the best spot for the next step, assuming the next step is taken in the same
   * direction. If the next step direction differs from the previous, the set of legs which can take the longest step in the new direction
   * moves the robot. The other set again moves to the optimal position for the next step.
   * It is possible to use this method for purely moving in any direction (without rotation) (so called crab walk). If overlayRotation != 0,
   * the robot also rotates around the z (yaw) axis during the step
   *
   * prevPositions:     array containing all leg positions prior to the step. Only gets updated in the last iteration when the step is finished
   * newPositions:      array to save the new leg positions to
   * stepDirection:     angle (in radians) of the step. 0 is forwards, PI is backwards, PI/2 is to the right.
   * radius:            the radius of the circle which the leg tip can't leave. Corresponds to the optimal step length. Recommended: 30 for crab walk without rotation, 20 with rotation. Min: 3 (rotating on the spot)
   * stepNumber:        number of iterations for one whole step. periodMS * stepNumber = duration of one step
   * overlayRotation:   angle (in rad) by which the robot rotates around the z (yaw) axis during one step. Must be between 0.02 and 0.3 or -0.02 and -0.3
   * stepHeight:        distance (in mm) by which the legs are raised to return home. Default: 20mm
   *
   * returns:           false if the robot is executing a step, true if the step has just finished
   */

  if (stepCounter == 1) {
    action = 5;  // the robot is executing a step
    // figure out which three legs will be lifted and which legs will stay on the ground
    // legs FR, ML, RR are lifted by default
    uint8_t liftingLegs[3] = { 0, 3, 4 };
    moveRightLeg = false;
    // the other legs touch the ground
    uint8_t stationaryLegs[3] = { 1, 2, 5 };
    // angles at which the legs are mounted (FR at 30°, FL at 150°, MR at 0, ...)
    const float legAngles[6] = { cornerLegAngle, cornerLegAngle * 5, 0, cornerLegAngle * 6, -cornerLegAngle, cornerLegAngle * 7 };
    // determine whether the other set of legs should be lifted, based on the previously moved legs and the new direction
    if ((prevRightLeg == false && abs(stepDirection - prevDirection) <= PI / 2) || (prevRightLeg == true && abs(stepDirection - prevDirection) > PI / 2)) {
      // lift legs FR, ML, RR
      liftingLegs[0] = 1;
      liftingLegs[1] = 2;
      liftingLegs[2] = 5;

      // the other legs are stationary
      stationaryLegs[0] = 0;
      stationaryLegs[1] = 3;
      stationaryLegs[2] = 4;

      moveRightLeg = true;
    }

    // calculate end positions for legs which are being lifted
    for (uint8_t i = 0; i < 3; ++i) {
      // move the leg to the optimal position for the next step, assuming the next step is in the same direction
      // the optimal leg position for the next step lies on the circle circumference in the opposite direction of movement
      // x value
      finalPositions[liftingLegs[i]][0] = homePos[0] + radius * cos(stepDirection + legAngles[liftingLegs[i]]) * 1.0;
      // y value
      finalPositions[liftingLegs[i]][1] = homePos[1] + radius * sin(stepDirection + legAngles[liftingLegs[i]]) * 1.0;
    }

    // calculate end positions for legs which are not lifted
    for (uint8_t i = 0; i < 3; ++i) {
      int intersections[2][2] = { 0 };  // array containing the intersections of the line of the leg tip and the circle

      // the leg tip moves from the current position in a straight line in the given direction. Find the intersection of this line with the circle
      // which is the furthest in this direction:
      lineCircleIntersect(homePos[0], homePos[1], radius, prevPositions[stationaryLegs[i]][0], prevPositions[stationaryLegs[i]][1], stepDirection + legAngles[stationaryLegs[i]], intersections);
      int index = getOppositeIntersection(prevPositions[stationaryLegs[i]][0], prevPositions[stationaryLegs[i]][1], stepDirection + legAngles[stationaryLegs[i]], intersections);
      // intersections[index][0] and intersections[index][1] are the final x/y leg positions (if they exist)
      if (intersections[index][0] != -1 && intersections[index][1] != -1) {
        finalPositions[stationaryLegs[i]][0] = intersections[index][0];
        finalPositions[stationaryLegs[i]][1] = intersections[index][1];
      } else {
        // if there is no intersection, just leave the legs where they are
        finalPositions[stationaryLegs[i]][0] = prevPositions[0][0];
        finalPositions[stationaryLegs[i]][1] = prevPositions[0][1];
      }
    }

    // The below code is necessary to let all legs start from random positions inside the circle
    // If the step lengths aren't equal across all 3 legs, the shortest step length is picked and finalPositions of the two other legs is adjusted
    // assert that all step lengths are equal
    uint16_t stepLengths[3] = { 0 };
    uint16_t avgLength = 0;
    for (uint8_t i = 0; i < 3; ++i) {
      // calculate the step lengths
      stepLengths[i] = (prevPositions[stationaryLegs[i]][0] - finalPositions[stationaryLegs[i]][0]) * (prevPositions[stationaryLegs[i]][0] - finalPositions[stationaryLegs[i]][0]) + (prevPositions[stationaryLegs[i]][1] - finalPositions[stationaryLegs[i]][1]) * (prevPositions[stationaryLegs[i]][1] - finalPositions[stationaryLegs[i]][1]);
      avgLength += stepLengths[i];
    }

    // check whether all distances are approximatly equal
    avgLength /= 3;

    // get the shortest distance
    uint16_t minLength = stepLengths[0];
    for (uint8_t i = 1; i < 3; ++i) {
      if (stepLengths[i] < minLength) {
        minLength = stepLengths[i];
      }
    }

    // Check whether one leg moves a significantly shorter distance
    const float tolerance = 0.1;  // 10%
    bool adjustSteps = false;
    for (uint8_t i = 0; i < 3; ++i) {
      if (abs(stepLengths[i] - avgLength) > avgLength * tolerance) {
        adjustSteps = true;
        break;
      }
    }

    if (adjustSteps == true) {
      // Adjust finalPositions, based on the shortest step length
      for (uint8_t i = 0; i < 3; ++i) {
        // get directional vector
        float dx = finalPositions[stationaryLegs[i]][0] - prevPositions[stationaryLegs[i]][0];
        float dy = finalPositions[stationaryLegs[i]][1] - prevPositions[stationaryLegs[i]][1];

        // scale factor based on minLength
        float scale = sqrt((float)minLength / (dx * dx + dy * dy));

        // calc new final positions based on minLength
        finalPositions[stationaryLegs[i]][0] = prevPositions[stationaryLegs[i]][0] + dx * scale;
        finalPositions[stationaryLegs[i]][1] = prevPositions[stationaryLegs[i]][1] + dy * scale;
      }
    }

    // the following allows for rotation to be added to the movement.
    //This isn't calculated to be inside of the circle anymore, therefore other precautions must be taken to avoid weird behavior / collisions
    if ((overlayRotation > 0.02 && overlayRotation < 0.3) || (overlayRotation < -0.02 && overlayRotation > -0.3)) {  // avoid too crazy turn angles and unnecessary calculation
      uint8_t mask = 0b100110;                                                                                       // apply rotation only to the stationary legs FR, ML, RR
      if (moveRightLeg == false) {
        mask = 0b011001;  // change if the other set of legs (FL, MR, RL) if they are stationary
      }
      calcBodyMovement(finalPositions, finalPositions, 0, 0, 0, 0.0, 0.0, overlayRotation, mask);
    }
  }


  // actually move the legs to the mapped position
  if (stepCounter > 0 && stepCounter <= stepNumber) {
    // interpolate the current leg position. Using the map() function will result in a linear motion to the final position
    interpolateStep(newPositions, prevPositions, finalPositions, stepHeight, stepCounter, stepNumber, moveRightLeg);
  }

  if (stepCounter == stepNumber) {
    // do this if the max number of iterations is reached
    if (moveRightLeg) {
      // that means that fr, ml, rl have moved
      prevRightLeg = true;
    } else {
      prevRightLeg = false;
    }
    // save the step direction
    prevDirection = stepDirection;

    // copy the final positions (without translation and rotation) in the current position array
    for (int i = 0; i < 6; ++i) {
      for (int j = 0; j < 3; ++j) {
        prevPositions[i][j] = finalPositions[i][j];
      }
    }
    // reset the step counter after one step is completed
    stepCounter = 1;
    // set action to 0, as the hexapod is now sleeping / doing nothing
    action = 0;
    prevAction = 5;
    return true;
  }
  stepCounter++;
  return false;
}

bool Hexapod::moveLegs(int positions[6][3]) {
  // move all legs to the new position and return true if every servo could reach the target
  bool flag0 = legFR.moveTo(positions[0][0], positions[0][1], positions[0][2]);
  bool flag1 = legFL.moveTo(positions[1][0], positions[1][1], positions[1][2]);
  bool flag2 = legMR.moveTo(positions[2][0], positions[2][1], positions[2][2]);
  bool flag3 = legML.moveTo(positions[3][0], positions[3][1], positions[3][2]);
  bool flag4 = legRR.moveTo(positions[4][0], positions[4][1], positions[4][2]);
  bool flag5 = legRL.moveTo(positions[5][0], positions[5][1], positions[5][2]);

  if (flag0 && flag1 && flag2 && flag3 && flag4 && flag5) {
    return true;
  } else {
    return false;
  }
}

[[nodiscard]] uint8_t Hexapod::getAction() {
  return action;
}

void Hexapod::lineCircleIntersect(int mX, int mY, int radius, int pX, int pY, float direction, int intersections[2][2]) {
  /*
   * calculates the two intersections of the line specified by pX and pY and direction with the 
   * circle centered around mX, mY with the given radius
   *
   * mX, mY:          center of the circle, in mm
   * radius:          radius of the circle, in mm
   * pX, pY:          point on the line, e.g. last/current position of the leg
   * direction:       heading of the line, [0, 2*PI]
   * intersections:   array containing the two intersections. (intersection[0][0], intersection[0][1]) is the first
   */
  // Shift the point (pX, pY) to the origin of the coordinate frame
  int shiftedPX = pX - mX;
  int shiftedPY = pY - mY;

  // Split the direction in x and y components. Integer for faster computation
  int scaleFactor = 100;
  int dx = cos(direction) * scaleFactor + 0.5;
  int dy = sin(direction) * scaleFactor + 0.5;

  // coefficents for qudratic formula
  int a = dx * dx + dy * dy;
  int b = 2 * (dx * shiftedPX + dy * shiftedPY);
  int c = shiftedPX * shiftedPX + shiftedPY * shiftedPY - radius * radius;

  int discriminant = b * b - 4 * a * c;

  if (discriminant < 0) {
    // if there are no coefficients, set intersections to -1
    intersections[0][0] = intersections[0][1] = -1;
    intersections[1][0] = intersections[1][1] = -1;
    return;
  }
  // approximate the sqrt
  int sqrt_discriminant = sqrt(discriminant) + 0.5;

  // calculate the quadratic formula
  float t1 = (-b + sqrt_discriminant) / (2.0 * a);
  float t2 = (-b - sqrt_discriminant) / (2.0 * a);

  // calculate and shift the intersections back
  intersections[0][0] = shiftedPX + t1 * dx + mX;
  intersections[0][1] = shiftedPY + t1 * dy + mY;

  // calculate the second intersection if there is one
  if (discriminant > 0) {
    intersections[1][0] = shiftedPX + t2 * dx + mX;
    intersections[1][1] = shiftedPY + t2 * dy + mY;
  } else {
    intersections[1][0] = intersections[1][1] = -1;
  }
}

int Hexapod::getOppositeIntersection(int pX, int pY, float direction, int intersections[2][2]) {
  /*
   * Returns the intersection which lies further in the passed direction.
   *
   * pX, pY:          the previous leg point (support point of the line)
   * direction:       direction of movement (heading of the line)
   * intersections:   the previously calculated intersections of the line with the circle
   */
  // reverse direction
  float oppositeDX = -cos(direction);
  float oppositeDY = -sin(direction);

  // project the vector from (pX, pY) to the first intersection onto the reverse direction
  float vector1X = intersections[0][0] - pX;
  float vector1Y = intersections[0][1] - pY;
  float projection1 = vector1X * oppositeDX + vector1Y * oppositeDY;

  // project the vector from (pX, pY) to the second intersectiononto the reverse direction
  float vector2X = intersections[1][0] - pX;
  float vector2Y = intersections[1][1] - pY;
  float projection2 = vector2X * oppositeDX + vector2Y * oppositeDY;

  // choose the point whose projection onto the direction is larger
  return (projection1 > projection2) ? 0 : 1;
}

void Hexapod::interpolateStep(int newPositions[6][3], int prevPositions[6][3], int finalPositions[6][3], uint8_t stepHeight, uint16_t stepCounter, uint16_t stepNumber, bool moveRightLeg, bool moveAllLegs) {
  /*
   * Generates the movement pattern for one step. Largely based on linear interpolation between prevPositions and finalPositions.
   * Three legs are also lifted (specified by moveRightLeg) and moved faster than the stationary legs. This allows them to reach their position
   * before they lower, thus making faster motion possible.
   *
   * newPositions[6][3]:      Array containing the new (calculated) positions for each leg in x-y-z local coordinates.
   * prevPositions[6][3]:     Array containing the start position in the beginning of the whole step. Used as starting point for the interpolation
   * finalPositions[6][3]:    Array containing the previously calculated final positions for each leg. The end point for interpolation
   * stepHeight:              Height (mm) by which the legs are being lifted
   * stepCounter:             The number of the current iteration of the step. Must be smaller than stepNumber
   * stepNumber:              Number of iterations in one whole step
   * moveRightLeg:            If true, legs FL, MR, RL are being lifted. Otherwise, legs FR, ML, RR
   * moveAllLegs:             If true, all six legs are being moved in a strait line. Otherwise, only the legs which are lifted are moved. 
   *                          Necessary if the legs touching the ground don't move in a strait line.
   *
   */
  // make sure stepCounter and stepNumber are valid
  if (stepCounter > stepNumber) {
    stepCounter = stepNumber;
  }

  // legs FL, MR, RL are lifted by default
  int liftingLegs[3] = { 1, 2, 5 };
  // the other legs are stationary
  int stationaryLegs[3] = { 0, 3, 4 };

  if (moveRightLeg == false) {
    // lift legs FR, ML, RR
    liftingLegs[0] = 0;
    liftingLegs[1] = 3;
    liftingLegs[2] = 4;

    // the other legs are stationary
    stationaryLegs[0] = 1;
    stationaryLegs[1] = 2;
    stationaryLegs[2] = 5;
  }

  // normal interpolation legs which aren't being lifted
  // don't move these legs if moveAllLegs == false (if these legs don't follow a simple line, for example while rotating on the spot)
  if (moveAllLegs == true) {
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        newPositions[stationaryLegs[i]][j] = map(stepCounter, 0, stepNumber, prevPositions[stationaryLegs[i]][j], finalPositions[stationaryLegs[i]][j]);
      }
    }
  }
  // the other legs are only moved in z direction (up and down) before being moved to their new position in the xy plane
  // this results in smoother walking patterns
  uint16_t startNumber = ceil(stepNumber * 0.2);     // number of iterations at which the legs start to move in the xy plane
  uint16_t targetNumber = floor(stepNumber * 0.75);  // number of iterations at which the legs reach their final xy position
  if (stepCounter >= startNumber && stepCounter <= targetNumber) {
    // z coordinate is calculated seperatly
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 2; ++j) {
        newPositions[liftingLegs[i]][j] = map(stepCounter, startNumber, targetNumber, prevPositions[liftingLegs[i]][j], finalPositions[liftingLegs[i]][j]);
      }
    }
  }

  // raise and lower the legs based on stepCounter. Only z coordinate is changed
  uint16_t upSteps = ceil(stepNumber * 0.15);     // legs are lifted in the first 15% of the step
  uint16_t downSteps = floor(stepNumber * 0.85);  // legs are lowered in the last 15% og the step
  if (stepCounter < upSteps) {
    //lifts legs
    for (int i = 0; i < 3; ++i) {
      newPositions[liftingLegs[i]][2] = map(stepCounter, 0, upSteps, prevPositions[liftingLegs[i]][2], finalPositions[liftingLegs[i]][2] - stepHeight);
    }
  } else if (stepCounter > downSteps) {
    // lower legs
    for (int i = 0; i < 3; ++i) {
      newPositions[liftingLegs[i]][2] = map(stepCounter, downSteps, stepNumber, prevPositions[liftingLegs[i]][2] - stepHeight, finalPositions[liftingLegs[i]][2]);
    }
  } else {
    // keep the legs lifted
    for (int i = 0; i < 3; ++i) {
      newPositions[liftingLegs[i]][2] = prevPositions[liftingLegs[i]][2] - stepHeight;
    }
  }
}