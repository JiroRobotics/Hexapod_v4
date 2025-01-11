// Hexapod.cpp

#include "Hexapod.h"

//#define DEBUG

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

void Hexapod::calcBodyMovement(float prevPositions[6][3], float newPositions[6][3], float xTrans, float yTrans, float zTrans, float roll, float pitch, float yaw, uint8_t legMask) {
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
   * legMask:       specifies which legs are moved. By default, all legs are moved (0b111111) 
   *
   * returns:     nothing
   */

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

  // calculating the new coordinates for each leg is done by:
  // 1. rotating the local coordinate frame to be aligned with the global frame
  // 2. shifting the local coordinate frame to the origin of the global frame
  // 3. shifting the previous leg end point by xTrans, yTrans, zTrans
  // 4. rotating the previous leg end point by roll, pitch, yaw
  // 5. shift the local coordinate frame back
  // 6. rotate the local coordinate frame back
  for (uint8_t k = 0; k < 6; ++k) {  // iterate over all legs

    // check if the leg should move (0b100000 is FR, 0b010000 is FL, ... 0b000001 is RL)
    if (legMask & (1 << (5 - k))) {

      // rotate the local coordinate system by the angle specified in legCoords[][]
      float temp = newPositions[k][0];
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
}

bool Hexapod::calcStep(float prevPositions[6][3], float newPositions[6][3], float stepDirection, uint8_t radius, uint16_t stepNumber, float overlayRotation, uint8_t stepHeight, bool useOffroad) {
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
   * useOffroad:        if true, uses each legs button to lower the legs until they touch the ground. Slower speed and larger stepHeight recommended
   *
   * returns:           false if the robot is executing a step, true if the step has just finished
   */

  if (stepCounter == 1) {
    action = 1;  // the robot is executing a step
    // figure out which three legs will be lifted and which legs will stay on the ground
    // legs FR, ML, RR are lifted by default
    uint8_t liftingLegs[3] = { 0, 3, 4 };
    moveRightLeg = false;
    // the other legs touch the ground
    uint8_t stationaryLegs[3] = { 1, 2, 5 };
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
      finalPositions[liftingLegs[i]][0] = homePos[0] + radius * cos(stepDirection + legCoords[liftingLegs[i]][2]) * 1.0;
      // y value
      finalPositions[liftingLegs[i]][1] = homePos[1] + radius * sin(stepDirection + legCoords[liftingLegs[i]][2]) * 1.0;
    }

    // calculate end positions for legs which are not lifted
    for (uint8_t i = 0; i < 3; ++i) {
      int intersections[2][2] = { 0 };  // array containing the intersections of the line of the leg tip and the circle

      // the leg tip moves from the current position in a straight line in the given direction. Find the intersection of this line with the circle
      // which is the furthest in this direction:
      lineCircleIntersect(homePos[0], homePos[1], radius, prevPositions[stationaryLegs[i]][0], prevPositions[stationaryLegs[i]][1], stepDirection + legCoords[stationaryLegs[i]][2], intersections);
      int index = getOppositeIntersection(prevPositions[stationaryLegs[i]][0], prevPositions[stationaryLegs[i]][1], stepDirection + legCoords[stationaryLegs[i]][2], intersections);
      // intersections[index][0] and intersections[index][1] are the final x/y leg positions (if they exist)
      if (intersections[index][0] != -1000 && intersections[index][1] != -1000) {
        finalPositions[stationaryLegs[i]][0] = intersections[index][0];
        finalPositions[stationaryLegs[i]][1] = intersections[index][1];
      } else {
        // if there is no intersection, just leave the legs where they are
        finalPositions[stationaryLegs[i]][0] = prevPositions[stationaryLegs[i]][0];
        finalPositions[stationaryLegs[i]][1] = prevPositions[stationaryLegs[i]][1];
      }
    }
    // The code below is necessary to let all legs start from random positions inside the circle
    // If the step lengths aren't equal across all 3 legs, the shortest step length is picked and finalPositions of the two other legs is adjusted
    // assert that all step lengths are equal
    float stepLengths[3] = { 0.0 };
    float avgLength = 0.0;
    for (uint8_t i = 0; i < 3; ++i) {
      // calculate the step lengths
      stepLengths[i] = (prevPositions[stationaryLegs[i]][0] - finalPositions[stationaryLegs[i]][0]) * (prevPositions[stationaryLegs[i]][0] - finalPositions[stationaryLegs[i]][0]) + (prevPositions[stationaryLegs[i]][1] - finalPositions[stationaryLegs[i]][1]) * (prevPositions[stationaryLegs[i]][1] - finalPositions[stationaryLegs[i]][1]);
      avgLength += stepLengths[i];
    }
    // check whether all distances are approximatly equal
    avgLength /= 3.0;

    // get the shortest distance
    float minLength = stepLengths[0];
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

        if (dx != 0.0 || dy != 0.0) {  // avoid division by zero
          // scale factor based on minLength
          float scale = sqrt(max(minLength / (dx * dx + dy * dy), 0.0));

          // calc new final positions based on minLength
          finalPositions[stationaryLegs[i]][0] = prevPositions[stationaryLegs[i]][0] + dx * scale;
          finalPositions[stationaryLegs[i]][1] = prevPositions[stationaryLegs[i]][1] + dy * scale;
        }
      }

      // Add the current step to the global position
      globalXPosition += sqrt(minLength) * cos(stepDirection + globalOrientation) * lengthAccuracy;
      globalYPosition += sqrt(minLength) * sin(stepDirection + globalOrientation) * lengthAccuracy;
    } else {
      // Add the current step to the global position
      globalXPosition += sqrt(avgLength) * cos(stepDirection + globalOrientation) * lengthAccuracy;
      globalYPosition += sqrt(avgLength) * sin(stepDirection + globalOrientation) * lengthAccuracy;
    }

    // the following allows for rotation to be added to the movement.
    // This isn't calculated to be inside of the circle anymore, therefore other precautions must be taken to avoid weird behavior / collisions
    if ((overlayRotation > 0.01 && overlayRotation < 0.3) || (overlayRotation < -0.01 && overlayRotation > -0.3)) {  // avoid too crazy turn angles and unnecessary calculation
      uint8_t mask = 0b100110;                                                                                       // apply rotation only to the stationary legs FR, ML, RR
      if (moveRightLeg == false) {
        mask = 0b011001;  // change if the other set of legs (FL, MR, RL) if they are stationary
      }
      // call calcBodyMovement with inverted rotation to get mathematically positive rotation
      calcBodyMovement(finalPositions, finalPositions, 0, 0, 0, 0.0, 0.0, -overlayRotation, mask);

      // keep track of the orientation compared to global coordinate system
      globalOrientation += overlayRotation * rotateAccuracy;
      // Only the Interval [PI, -PI] is used
      if (globalOrientation > PI) {
        globalOrientation -= 2 * PI;
      } else if (globalOrientation < -PI) {
        globalOrientation += 2 * PI;
      }
    }
  }


  // actually move the legs to the mapped position
  if (stepCounter > 0 && stepCounter <= stepNumber) {
    if (!useOffroad) {
      // interpolate the current leg position. Using the map() function will result in a linear motion to the final position
      interpolateStep(newPositions, prevPositions, finalPositions, stepHeight, stepCounter, stepNumber, moveRightLeg);
    } else {
      // use buttons on each leg
      interpolateOffroadStep(newPositions, prevPositions, finalPositions, stepHeight, stepCounter, stepNumber, moveRightLeg);
    }
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
    if (!useOffroad) {
      for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 3; ++j) {
          prevPositions[i][j] = finalPositions[i][j];
        }
      }
    } else {
      // don't use finalPositions as end positions when using offroad mode because legs might not reach the desired positions (especially z coord)
      for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 3; ++j) {
          prevPositions[i][j] = newPositions[i][j];
        }
      }
    }
    // reset the step counter after one step is completed
    stepCounter = 1;
    // set action to 0, as the hexapod is now sleeping / doing nothing
    action = 0;
    return true;
  }
  stepCounter++;
  return false;
}

bool Hexapod::moveLegs(float positions[6][3]) {
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

void Hexapod::step(float legPos[6][3], float stepDirection, uint8_t radius, uint16_t stepNumber, float overlayRotation, uint8_t stepHeight) {
  /*
   * Wrapper function to easily perform a step. Note that this function is blocking and doesn't allow for superimposition of other movement.
   * 
   * legPos[][]:      Array containing the positions of the legs before starting the step
   * stepDirection:   Direction (rad) in which the step is taken. 0 is forwards, PI is backwards, PI/2 is to the right.
   * radius:          Radius of the circle in which the legs will move. Equivalent to the maximum step length
   * stepNumber:      Number of iterations. periodMS * stepNumber = step duration
   * overlayRotation: Rotation around z axis during the step.
   * stepHeight:      Distance (mm) by which the legs are lifted.
   */
  // array to save the leg positions for each iteration
  float posArray[6][3];

  do {
    unsigned long startTime = millis();

    this->calcStep(legPos, posArray, stepDirection, radius, stepNumber, overlayRotation, stepHeight);
    this->moveLegs(posArray);

    while (millis() < startTime + periodMs) {
      // wait a bit so that the loop is executet every periodMS ms
    }
  } while (this->getAction() != 0);
}

bool Hexapod::travelPath(float legPos[6][3], int16_t pathPoints[][2], uint8_t numPoints, uint16_t iterationNumber, uint8_t pathType, float maxRotationPerStep, uint8_t stepHeight) {
  /*
   * Calculates and takes steps in order to reach the provided points one after another.
   * NOTE: This function is blocking, it therefore doesn't work in the standard main loop!
   * 
   * legPos[][]:      Array containing the leg positions before calling this method
   * pathPoints[][]:  Array containing the (x, y) points which the robot should travel to
   * numPoints:       number of points in the pathPoints array
   * iterationNumber: number of iterations per step. Fewer iterations result in faster movement
   * pathType:        either 0 or 1. 0: walking and rotating. 1: walking without rotation, maintaining the same heading
   *
   */

  // iterate over all provided points
  for (uint8_t i = 0; i < numPoints; ++i) {
#ifdef DEBUG
    Serial.print("Next Target Point (x, y): ");
    Serial.print(pathPoints[i][0]);
    Serial.print(", ");
    Serial.println(pathPoints[i][1]);
    Serial.println("=============================");
#endif
    // calculate the distance from the current position to the desired position
    float distToGoal = (this->getGlobalXPos() - pathPoints[i][0]) * (this->getGlobalXPos() - pathPoints[i][0]) + (this->getGlobalYPos() - pathPoints[i][1]) * (this->getGlobalYPos() - pathPoints[i][1]);
    distToGoal = sqrt(distToGoal);

    // accuracy = 30mm
    while (distToGoal > 30) {
      // compute the direction of the desired point
      float goalDirection = atan2(pathPoints[i][1] - this->getGlobalYPos(), pathPoints[i][0] - this->getGlobalXPos());

      // walk with rotation
      if (pathType == 0) {
        // get the difference between the current heading and the desired direction
        float directionError = goalDirection - this->getGlobalOrientation();
        if (directionError > PI) {
          directionError -= 2 * PI;
        } else if (directionError < -PI) {
          directionError += 2 * PI;
        }

        // rotation specifies by which amount the robot rotates during the next step
        float rotation = 0.0;
        if (directionError > maxRotationPerStep) {
          rotation = maxRotationPerStep;
        } else if (directionError < -maxRotationPerStep) {
          rotation = -maxRotationPerStep;
        } else {
          rotation = directionError;
        }
        // the radius determines the step length
        uint8_t maxRadius = 30;
        if (abs(rotation) > 0.08) {
          maxRadius = 20;
        }
        uint8_t radius = min(distToGoal / 2, maxRadius);

        // execute the step
        this->step(legPos, 0.0, radius, iterationNumber, rotation, stepHeight);
#ifdef DEBUG
        Serial.print("Position (x, y): ");
        Serial.print(this->getGlobalXPos());
        Serial.print(", ");
        Serial.println(this->getGlobalYPos());
        Serial.print("Heading: ");
        Serial.println(this->getGlobalOrientation());
#endif

      } else if (pathType == 1) {
        // walk while maintaining the same heading
        // get the difference between the current heading and the desired direction (transform direction to the next point in base coordinate system)
        float directionDiff = goalDirection - this->getGlobalOrientation();
        if (directionDiff > PI) {
          directionDiff -= 2 * PI;
        } else if (directionDiff < -PI) {
          directionDiff += 2 * PI;
        }

        uint8_t maxRadius = 30;
        // take smaller steps if moving sideways
        if (abs(directionDiff) > 0.08 && abs(directionDiff) < 3.06) {
          maxRadius = 20;
        }
        uint8_t radius = min(distToGoal / 2, maxRadius);

        // execute the step
        this->step(legPos, directionDiff, radius, iterationNumber, 0.0, stepHeight);

#ifdef DEBUG
        Serial.println(radius);
        Serial.print("Position (x, y): ");
        Serial.print(this->getGlobalXPos());
        Serial.print(", ");
        Serial.println(this->getGlobalYPos());
        Serial.print("Step Direction: ");
        Serial.println(directionDiff);
#endif
      } else {
        return false;
      }

      // recompute the distance to the goal position
      distToGoal = (this->getGlobalXPos() - pathPoints[i][0]) * (this->getGlobalXPos() - pathPoints[i][0]) + (this->getGlobalYPos() - pathPoints[i][1]) * (this->getGlobalYPos() - pathPoints[i][1]);
      distToGoal = sqrt(distToGoal);
#ifdef DEBUG
      Serial.print("New Distance to Goal: ");
      Serial.println(distToGoal);
      Serial.println("=============================");
#endif
    }
#ifdef DEBUG
    Serial.print("Sucessfully reached Point (x, y): ");
    Serial.print(pathPoints[i][0]);
    Serial.print(", ");
    Serial.println(pathPoints[i][1]);
#endif
  }
  return true;
}


void Hexapod::lineCircleIntersect(float mX, float mY, int radius, float pX, float pY, float direction, int intersections[2][2]) {
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
  float shiftedPX = pX - mX;
  float shiftedPY = pY - mY;

  // Split the direction in x and y components.
  float dx = cos(direction);
  float dy = sin(direction);

  // coefficents for qudratic formula
  float a = dx * dx + dy * dy;
  float b = 2 * (dx * shiftedPX + dy * shiftedPY);
  float c = shiftedPX * shiftedPX + shiftedPY * shiftedPY - radius * radius;

  float discriminant = b * b - 4 * a * c;

  if (discriminant < -1e-6) {
    // if there are no coefficients, set intersections to -1000
    intersections[0][0] = intersections[0][1] = -1000;
    intersections[1][0] = intersections[1][1] = -1000;
    return;
  }
  // approximate the sqrt
  float sqrt_discriminant = sqrt(max(0.0, discriminant));

  // calculate the quadratic formula
  float t1 = (-b + sqrt_discriminant) / (2.0 * a);
  float t2 = (-b - sqrt_discriminant) / (2.0 * a);

  // calculate and shift the intersections back
  intersections[0][0] = shiftedPX + t1 * dx + mX;
  intersections[0][1] = shiftedPY + t1 * dy + mY;

  // calculate the second intersection if there is one
  if (discriminant > 1e-6) {
    intersections[1][0] = shiftedPX + t2 * dx + mX;
    intersections[1][1] = shiftedPY + t2 * dy + mY;
  } else {
    intersections[1][0] = intersections[1][1] = -1000;
  }
}

int Hexapod::getOppositeIntersection(float pX, float pY, float direction, int intersections[2][2]) {
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

void Hexapod::interpolateStep(float newPositions[6][3], float prevPositions[6][3], float finalPositions[6][3], uint8_t stepHeight, uint16_t stepCounter, uint16_t stepNumber, bool moveRightLeg, bool moveAllLegs) {
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
        newPositions[stationaryLegs[i]][j] = mapFloat(stepCounter, 0.0, stepNumber * 1.0, prevPositions[stationaryLegs[i]][j], finalPositions[stationaryLegs[i]][j]);
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
        newPositions[liftingLegs[i]][j] = mapFloat(stepCounter, startNumber * 1.0, targetNumber * 1.0, prevPositions[liftingLegs[i]][j], finalPositions[liftingLegs[i]][j]);
      }
    }
  } else if (stepCounter < startNumber) {
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 2; ++j) {
        newPositions[liftingLegs[i]][j] = prevPositions[liftingLegs[i]][j];
      }
    }
  } else {
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 2; ++j) {
        newPositions[liftingLegs[i]][j] = finalPositions[liftingLegs[i]][j];
      }
    }
  }

  // raise and lower the legs based on stepCounter. Only z coordinate is changed
  uint16_t upSteps = ceil(stepNumber * 0.15);     // legs are lifted in the first 15% of the step
  uint16_t downSteps = floor(stepNumber * 0.85);  // legs are lowered in the last 15% og the step
  if (stepCounter < upSteps) {
    //lifts legs
    for (int i = 0; i < 3; ++i) {
      newPositions[liftingLegs[i]][2] = mapFloat(stepCounter, 0.0, upSteps * 1.0, prevPositions[liftingLegs[i]][2], finalPositions[liftingLegs[i]][2] - stepHeight);
    }
  } else if (stepCounter > downSteps) {
    // lower legs
    for (int i = 0; i < 3; ++i) {
      newPositions[liftingLegs[i]][2] = mapFloat(stepCounter, downSteps * 1.0, stepNumber * 1.0, prevPositions[liftingLegs[i]][2] - stepHeight, finalPositions[liftingLegs[i]][2]);
    }
  } else {
    // keep the legs lifted
    for (int i = 0; i < 3; ++i) {
      newPositions[liftingLegs[i]][2] = prevPositions[liftingLegs[i]][2] - stepHeight;
    }
  }
}

void Hexapod::interpolateOffroadStep(float newPositions[6][3], float prevPositions[6][3], float finalPositions[6][3], uint8_t stepHeight, uint16_t stepCounter, uint16_t stepNumber, bool moveRightLeg, bool moveAllLegs) {
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

  // bitmask for leg averaging
  uint8_t bitmask = 0b100110;

  if (moveRightLeg == false) {
    // lift legs FR, ML, RR
    liftingLegs[0] = 0;
    liftingLegs[1] = 3;
    liftingLegs[2] = 4;

    // the other legs are stationary
    stationaryLegs[0] = 1;
    stationaryLegs[1] = 2;
    stationaryLegs[2] = 5;

    bitmask = 0b011001;
  }

  // normal interpolation legs which aren't being lifted
  // don't move these legs if moveAllLegs == false (if these legs don't follow a simple line, for example while rotating on the spot)
  if (moveAllLegs == true) {
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        newPositions[stationaryLegs[i]][j] = mapFloat(stepCounter, 0.0, stepNumber * 1.0, prevPositions[stationaryLegs[i]][j], finalPositions[stationaryLegs[i]][j]);
      }
    }
    // adjust body height of the three legs
    averageLegs(newPositions, homePos[2], bitmask);
  }
  // the other legs are only moved in z direction (up and down) before being moved to their new position in the xy plane
  // this results in smoother walking patterns
  uint16_t startNumber = ceil(stepNumber * 0.2);     // number of iterations at which the legs start to move in the xy plane
  uint16_t targetNumber = floor(stepNumber * 0.75);  // number of iterations at which the legs reach their final xy position
  if (stepCounter >= startNumber && stepCounter <= targetNumber) {
    // z coordinate is calculated seperatly
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 2; ++j) {
        newPositions[liftingLegs[i]][j] = mapFloat(stepCounter, startNumber * 1.0, targetNumber * 1.0, prevPositions[liftingLegs[i]][j], finalPositions[liftingLegs[i]][j]);
      }
    }
  } else if (stepCounter < startNumber) {
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 2; ++j) {
        newPositions[liftingLegs[i]][j] = prevPositions[liftingLegs[i]][j];
      }
    }
  } else {
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 2; ++j) {
        newPositions[liftingLegs[i]][j] = finalPositions[liftingLegs[i]][j];
      }
    }
  }

  // raise and lower the legs based on stepCounter. Only z coordinate is changed
  uint16_t upSteps = ceil(stepNumber * 0.15);     // legs are lifted in the first 15% of the step
  uint16_t downSteps = floor(stepNumber * 0.80);  // legs are lowered in the last 20% of the step
  if (stepCounter < upSteps) {
    //lifts legs
    for (int i = 0; i < 3; ++i) {
      newPositions[liftingLegs[i]][2] = mapFloat(stepCounter, 0.0, upSteps * 1.0, prevPositions[liftingLegs[i]][2], finalPositions[liftingLegs[i]][2] - stepHeight);
    }
  } else if (stepCounter > downSteps) {
    // lower legs
    for (int i = 0; i < 3; ++i) {
      // only lower the legs while they don't touch the ground
      if (!legs[liftingLegs[i]]->touchesGround()) {
        // lower the leg further down than usual (max = standard + stepHeight)
        newPositions[liftingLegs[i]][2] = mapFloat(stepCounter, downSteps * 1.0, stepNumber * 1.0, prevPositions[liftingLegs[i]][2] - stepHeight, finalPositions[liftingLegs[i]][2] + stepHeight);
      }
    }
  } else {
    // keep the legs lifted
    for (int i = 0; i < 3; ++i) {
      newPositions[liftingLegs[i]][2] = prevPositions[liftingLegs[i]][2] - stepHeight;
    }
  }
}

float Hexapod::mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

[[nodiscard]] uint8_t Hexapod::getAction() {
  return action;
}

[[nodiscard]] float Hexapod::getGlobalOrientation() {
  return globalOrientation;
}

[[nodiscard]] float Hexapod::getGlobalXPos() {
  return globalXPosition;
}

[[nodiscard]] float Hexapod::getGlobalYPos() {
  return globalYPosition;
}

void Hexapod::setStartPoint(float xGlobal, float yGlobal, float heading) {
  /*
   * Sets the starting point of the robot in the global coordinate system.
   * 
   * xGlobal:   x value in mm, set as starting point
   * yGlobal:   y value in mm, set as starting point
   * heading:   orientation (in rad) of the hexapod. 0.0 is default (x-axis pointing forward)
   */
  globalXPosition = xGlobal;
  globalYPosition = yGlobal;
  globalOrientation = heading;
}

void Hexapod::setHeading(float heading) {
  /*
   * function to set the heading seperatly
   * 
   * heading:   orientation (in rad) of the hexapod. 0.0 is default (x-axis pointing forward)
   */
  globalOrientation = heading;
}

bool Hexapod::startIMU() {
  /*
   * starts the IMU of the Arduino Nano 33 BLE (Sense). If sucessfull, the green LED will blink 3 times.
   * If unsuccessful, the red LED will blink forever
   *
   * Returns: true if sucessful
   */
#ifdef ARDUINO_ARDUINO_NANO33BLE
  if (!IMU.begin()) {
#ifdef DEBUG
    Serial.println("IMU-Sensor couldn't be initialized!");
#endif
    pinMode(LEDR, OUTPUT);
    while (1) {
      digitalWrite(LEDR, LOW);
      delay(200);
      digitalWrite(LEDR, HIGH);
      delay(200);
    }
  } else {
    pinMode(LEDG, OUTPUT);
    for (uint8_t i = 0; i < 3; ++i) {
      digitalWrite(LEDG, LOW);
      delay(100);
      digitalWrite(LEDG, HIGH);
      delay(100);
    }
  }
  return true;
#else
  return false;
#endif
}

void Hexapod::calibrateIMU(uint16_t numSamples) {
  /*
   * Calibrates the IMU by calculating offsets for roll and pitch angles (rad)
   * simple average is calculated and saved as offset
   * The blue LED is on while calibrating
   *
   * numSamples:    number of samples over which the average is taken
   */
#ifdef ARDUINO_ARDUINO_NANO33BLE
  uint16_t counter = 0;
  float rollAvg = 0.0;
  float pitchAvg = 0.0;
  float x, y, z;

  pinMode(LEDB, OUTPUT);
  digitalWrite(LEDB, LOW);
  while (counter < numSamples) {
    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(x, y, z);
      rollAvg += atan2(y, x);
      pitchAvg += atan2(z, sqrt(y * y + x * x));
      counter++;
    }
  }
  rollAvg = rollAvg / counter;
  pitchAvg = pitchAvg / counter;

  // safe the offsets in class variables
  this->pitchOffset = pitchAvg;
  this->rollOffset = rollAvg;

#ifdef DEBUG
  Serial.print("Roll Average (rad): ");
  Serial.println(rollAvg);
  Serial.print("Pitch Average (rad): ");
  Serial.println(pitchAvg);
#endif

  digitalWrite(LEDB, HIGH);
#endif
}

bool Hexapod::readRollPitch(float &roll, float &pitch) {
/*
   * calculates current roll and pitch, applies IIR filter and passes values to roll and pitch variables.
   * Because of the IIR filter, it should be used frequently (ideally with fixed periodicity)
   *
   * roll:    parameter to which the filtered roll value is passed
   * pitch:   parameter to which the filtered pitch value is passed
   *
   * Returns: true if sucessful, false otherwise
   */
// check availability
#ifdef ARDUINO_ARDUINO_NANO33BLE
  if (IMU.accelerationAvailable()) {
    float x, y, z;
    IMU.readAcceleration(x, y, z);
    // calc current roll & pitch
    float newRoll = atan2(y, x);
    float newPitch = atan2(z, sqrt(y * y + x * x));

    // IIR filter with coefficients 0.3, 0.7
    roll = 0.2 * (newRoll - this->rollOffset) + 0.8 * this->prevRoll;
    pitch = 0.2 * (newPitch - this->pitchOffset) + 0.8 * this->prevPitch;
    this->prevRoll = roll;
    this->prevPitch = pitch;

    return true;
  } else {
    // return false if no data is available
    return false;
  }
#else
#warning "IMU only supported for Arduino Nano 33 BLE"
  return false;
#endif
}

bool Hexapod::balance(float legPos[6][3], float newPos[6][3]) {
  float roll, pitch;
  // proportional gain
  float Kp = 0.12;
  if (this->readRollPitch(roll, pitch)) {
    //deadband for roll and pitch (approx. 2 deg)
    if (abs(roll) < 0.035) {
      roll = 0.0;
    }
    if (abs(pitch) < 0.035) {
      pitch = 0.0;
    }
    // proportional controller adjusting the change in roll and pitch output
    this->prevRollOutput += Kp * roll;
    this->prevPitchOutput += Kp * pitch;

    // limits for roll at +-0.25rad
    if (this->prevRollOutput > 0.25) {
      this->prevRollOutput = 0.25;
    } else if (this->prevRollOutput < -0.25) {
      this->prevRollOutput = -0.25;
    }

    // limits for pitch at +-0.25rad
    if (this->prevPitchOutput > 0.25) {
      this->prevPitchOutput = 0.25;
    } else if (this->prevPitchOutput < -0.25) {
      this->prevPitchOutput = -0.25;
    }
#ifdef DEBUG
    Serial.print("RollOutput:");
    Serial.print(this->prevRollOutput);
    Serial.print("\tPitchOutput:");
    Serial.println(this->prevPitchOutput);
#endif
    // update leg positions
    this->calcBodyMovement(legPos, newPos, 0.0, 0.0, 0.0, this->prevRollOutput, this->prevPitchOutput, 0.0);
    return true;
  } else {
    // important so that newPos is valid/initialized no matter what
    for (uint8_t i = 0; i < 3; ++i) {
      for (uint8_t j = 0; j < 6; ++j) {
        legPos[j][i] = newPos[j][i];
      }
    }
    return false;
  }
}

void Hexapod::averageLegs(float legPos[6][3], float avgHeight, uint8_t legMask) {
  /*
   * Calculates the current average over all z coordinate and adjusts all z values to get the desired average
   *
   * legPos[6][3]:  current leg positions, will be changed
   * avgHeight:     the desired z value in mm
   * legMask:       bitmask to select which legs to average and adjust
   */
  float currentAverage = 0.0;
  uint8_t counter = 0;
  // calculate the current average
  for (uint8_t i = 0; i < 6; ++i) {
    // check whether the leg should be taken into account
    if (legMask & (1 << (5 - i))) {
      currentAverage += legPos[i][2];
      counter++;
    }
  }
  currentAverage /= counter;
  // adjust the z coordinate slowly, by 0.5 mm per function call
  for (uint8_t i = 0; i < 6; ++i) {
    // only adjust the specified legs
    if (legMask & (1 << (5 - i))) {

      if (currentAverage > avgHeight + 0.25) {
        // lower the body if average is too high
        legPos[i][2] -= 0.5;
      } else if (currentAverage < avgHeight - 0.25) {
        // raise the body if average is too low
        legPos[i][2] += 0.5;
      }
    }
  }
}