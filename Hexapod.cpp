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

void Hexapod::calcBodyMovement(int prevPositions[6][3], int newPositions[6][3], int16_t xTrans, int16_t yTrans, int16_t zTrans, float roll, float pitch, float yaw) {
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
  float result[3] = { 0.0, 0.0, 0.0 };

  // ++++++++++++++++++++++++++++++
  // leg mid right
  // ++++++++++++++++++++++++++++++
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


  // ++++++++++++++++++++++++++++++
  // leg mid left
  // ++++++++++++++++++++++++++++++
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

  // ++++++++++++++++++++++++++++++
  // leg front right
  // ++++++++++++++++++++++++++++++
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

  // ++++++++++++++++++++++++++++++
  // leg rear right
  // ++++++++++++++++++++++++++++++
  // rotate the local coordinate system by -30°
  temp = newPositions[4][0];
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

  // ++++++++++++++++++++++++++++++
  // leg front left
  // ++++++++++++++++++++++++++++++
  // rotate the local coordinate system by 180°-30°
  temp = newPositions[1][0];
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

  // ++++++++++++++++++++++++++++++
  // leg rear left
  // ++++++++++++++++++++++++++++++
  // rotate the local coordinate system by 180°+30°
  temp = newPositions[5][0];
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
}

bool Hexapod::calcCrabwalk(int prevPositions[6][3], int newPositions[6][3], uint16_t stepDist, float stepDirection, uint16_t stepNumber, uint8_t stepHeight, bool useOffroad) {
  /*
   * calculates the next position for a crab step in any direction.
   * 
   * prevPositions: array containing all leg positions prior to the step. Only gets updated in the last iteration when the step is finished
   * newPositions:  array to save the new leg positions to
   * stepDist:      distance (in mm) of the step
   * stepDirection: angle (in radians) of the step
   * stepNumber:    number of iterations for one whole step
   * stepHeight:    distance (in mm) by which the legs are raised to return home. Default: 20mm
   * useOffroad:    if true, the robot will raise and lower the legs based on the push-button on the leg
   * 
   */

  if (rightLegHome && stepCounter == 1) {
    // do this in the first iteration if three legs are at home position
    leftLegHome = false;
    moveRightLeg = true;  // the legs FR, ML and RR are the ones who move on the floor while FL, MR and RL return to home position
    action = 1;           // the hexapod is executing a step

    // calculate the x and y translation from stepDist and stepDirection
    int16_t xDist = round(cos(stepDirection) * stepDist);
    int16_t yDist = round(sin(stepDirection) * stepDist);

    // calculate the end position for each leg and save it along with the starting position and later interpolate the position for each iteration
    // FR, ML and RR will move to a new position while FL, MR and RL will return to home position
    for (int i = 0; i < 2; ++i) {
      for (int j = 0; j < 6; ++j) {
        finalPositions[j][i] = homePos[i];
      }
    }
    // the z coordinate will be equal to homePos[2] for the legs returning home
    finalPositions[1][2] = homePos[2];
    finalPositions[2][2] = homePos[2];
    finalPositions[5][2] = homePos[2];
    // the legs moving will keep their z coordinate during the step
    finalPositions[0][2] = prevPositions[0][2];
    finalPositions[3][2] = prevPositions[3][2];
    finalPositions[4][2] = prevPositions[4][2];

    // ++++++++++++++++++++++++++++++
    // leg front right
    // ++++++++++++++++++++++++++++++
    // rotate the local coordinate system by 30°
    int temp = finalPositions[0][0];
    finalPositions[0][0] = finalPositions[0][0] * cos(-cornerLegAngle) - finalPositions[0][1] * sin(-cornerLegAngle);
    finalPositions[0][1] = temp * sin(-cornerLegAngle) + finalPositions[0][1] * cos(-cornerLegAngle);

    // translation of leg end point
    finalPositions[0][0] -= xDist;
    finalPositions[0][1] -= yDist;

    //rotate the coordinate system back by -30°
    temp = finalPositions[0][0];
    finalPositions[0][0] = finalPositions[0][0] * cos(cornerLegAngle) - finalPositions[0][1] * sin(cornerLegAngle);
    finalPositions[0][1] = temp * sin(cornerLegAngle) + finalPositions[0][1] * cos(cornerLegAngle);

    // ++++++++++++++++++++++++++++++
    // leg mid left
    // ++++++++++++++++++++++++++++++
    finalPositions[3][0] += xDist;
    finalPositions[3][1] += yDist;

    // ++++++++++++++++++++++++++++++
    // leg rear right
    // ++++++++++++++++++++++++++++++
    // rotate the local coordinate system by -30°
    temp = finalPositions[4][0];
    finalPositions[4][0] = finalPositions[4][0] * cos(cornerLegAngle) - finalPositions[4][1] * sin(cornerLegAngle);
    finalPositions[4][1] = temp * sin(cornerLegAngle) + finalPositions[4][1] * cos(cornerLegAngle);

    // translation of leg end point
    finalPositions[4][0] -= xDist;
    finalPositions[4][1] -= yDist;

    //rotate the coordinate system back by 30°
    temp = finalPositions[4][0];
    finalPositions[4][0] = finalPositions[4][0] * cos(-cornerLegAngle) - finalPositions[4][1] * sin(-cornerLegAngle);
    finalPositions[4][1] = temp * sin(-cornerLegAngle) + finalPositions[4][1] * cos(-cornerLegAngle);

  } else if (leftLegHome && stepCounter == 1) {
    rightLegHome = false;
    moveRightLeg = false;
    action = 1;

    // calculate the x and y translation from stepDist and stepDirection
    int16_t xDist = round(cos(stepDirection) * stepDist);
    int16_t yDist = round(sin(stepDirection) * stepDist);

    // calculate the end position for each leg and save it along with the starting position and later interpolate the position for each iteration
    // FL, MR and RL will move to a new position while FR, ML and RR will return to home position
    for (int i = 0; i < 2; ++i) {
      for (int j = 0; j < 6; ++j) {
        finalPositions[j][i] = homePos[i];
      }
    }

    // the z coordinate will be equal to homePos[2] for the legs returning home
    finalPositions[0][2] = homePos[2];
    finalPositions[3][2] = homePos[2];
    finalPositions[4][2] = homePos[2];
    // the legs moving will keep their z coordinate during the step
    finalPositions[1][2] = prevPositions[1][2];
    finalPositions[2][2] = prevPositions[2][2];
    finalPositions[5][2] = prevPositions[5][2];

    // ++++++++++++++++++++++++++++++
    // leg front left
    // ++++++++++++++++++++++++++++++
    // rotate the local coordinate system by 180°-30°
    int temp = finalPositions[1][0];
    finalPositions[1][0] = finalPositions[1][0] * cos(-PI + cornerLegAngle) - finalPositions[1][1] * sin(-PI + cornerLegAngle);
    finalPositions[1][1] = temp * sin(-PI + cornerLegAngle) + finalPositions[1][1] * cos(-PI + cornerLegAngle);

    // translation of leg end point
    finalPositions[1][0] -= xDist;
    finalPositions[1][1] -= yDist;

    //rotate the coordinate system back by -180°+30°
    temp = finalPositions[1][0];
    finalPositions[1][0] = finalPositions[1][0] * cos(PI - cornerLegAngle) - finalPositions[1][1] * sin(PI - cornerLegAngle);
    finalPositions[1][1] = temp * sin(PI - cornerLegAngle) + finalPositions[1][1] * cos(PI - cornerLegAngle);

    // ++++++++++++++++++++++++++++++
    // leg mid right
    // ++++++++++++++++++++++++++++++
    // translation of leg end point
    finalPositions[2][0] -= xDist;
    finalPositions[2][1] -= yDist;

    // ++++++++++++++++++++++++++++++
    // leg rear left
    // ++++++++++++++++++++++++++++++
    // rotate the local coordinate system by 180°+30°
    temp = finalPositions[5][0];
    finalPositions[5][0] = finalPositions[5][0] * cos(-PI - cornerLegAngle) - finalPositions[5][1] * sin(-PI - cornerLegAngle);
    finalPositions[5][1] = temp * sin(-PI - cornerLegAngle) + finalPositions[5][1] * cos(-PI - cornerLegAngle);

    // translation of leg end point
    finalPositions[5][0] -= xDist;
    finalPositions[5][1] -= yDist;

    //rotate the coordinate system back by -180°-30°
    temp = finalPositions[5][0];
    finalPositions[5][0] = finalPositions[5][0] * cos(PI + cornerLegAngle) - finalPositions[5][1] * sin(PI + cornerLegAngle);
    finalPositions[5][1] = temp * sin(PI + cornerLegAngle) + finalPositions[5][1] * cos(PI + cornerLegAngle);

  } else if (stepCounter == 1) {
    //do this if the legs are not at home position
  }

  if (stepCounter > 0 && stepCounter <= stepNumber && !useOffroad) {
    // interpolate the current leg position. Using the map() function will result in a linear motion to the final position
    for (int i = 0; i < 6; ++i) {
      for (int j = 0; j < 3; ++j) {
        newPositions[i][j] = map(stepCounter, 1, stepNumber, prevPositions[i][j], finalPositions[i][j]);
      }
    }
    // this is necessary to lift the three legs returning home of the ground
    if (stepCounter < ceil(stepNumber * 0.15)) {
      // lift the three legs returning to home position
      if (leftLegHome) {
        // lift legs FR, ML, RR
        newPositions[0][2] = map(stepCounter, 1, ceil(stepNumber * 0.15), prevPositions[0][2], prevPositions[0][2] - stepHeight);
        newPositions[3][2] = map(stepCounter, 1, ceil(stepNumber * 0.15), prevPositions[3][2], prevPositions[3][2] - stepHeight);
        newPositions[4][2] = map(stepCounter, 1, ceil(stepNumber * 0.15), prevPositions[4][2], prevPositions[4][2] - stepHeight);
      } else {
        // lift legs FL, MR, RL
        newPositions[1][2] = map(stepCounter, 1, ceil(stepNumber * 0.15), prevPositions[1][2], prevPositions[1][2] - stepHeight);
        newPositions[2][2] = map(stepCounter, 1, ceil(stepNumber * 0.15), prevPositions[2][2], prevPositions[2][2] - stepHeight);
        newPositions[5][2] = map(stepCounter, 1, ceil(stepNumber * 0.15), prevPositions[5][2], prevPositions[5][2] - stepHeight);
      }
    } else if (stepCounter > ceil(stepNumber * 0.85)) {
      if (leftLegHome) {
        // lower legs FR, ML, RR
        newPositions[0][2] = map(stepCounter, ceil(stepNumber * 0.85), stepNumber, prevPositions[0][2] - stepHeight, finalPositions[0][2]);
        newPositions[3][2] = map(stepCounter, ceil(stepNumber * 0.85), stepNumber, prevPositions[3][2] - stepHeight, finalPositions[3][2]);
        newPositions[4][2] = map(stepCounter, ceil(stepNumber * 0.85), stepNumber, prevPositions[4][2] - stepHeight, finalPositions[4][2]);
      } else {
        // lower legs FL, MR, RL
        newPositions[1][2] = map(stepCounter, ceil(stepNumber * 0.85), stepNumber, prevPositions[1][2] - stepHeight, finalPositions[1][2]);
        newPositions[2][2] = map(stepCounter, ceil(stepNumber * 0.85), stepNumber, prevPositions[2][2] - stepHeight, finalPositions[2][2]);
        newPositions[5][2] = map(stepCounter, ceil(stepNumber * 0.85), stepNumber, prevPositions[5][2] - stepHeight, finalPositions[5][2]);
      }
    } else {
      if (leftLegHome) {
        // lower legs FR, ML, RR
        newPositions[0][2] = prevPositions[0][2] - stepHeight;
        newPositions[3][2] = prevPositions[3][2] - stepHeight;
        newPositions[4][2] = prevPositions[4][2] - stepHeight;
      } else {
        // lower legs FL, MR, RL
        newPositions[1][2] = prevPositions[1][2] - stepHeight;
        newPositions[2][2] = prevPositions[2][2] - stepHeight;
        newPositions[5][2] = prevPositions[5][2] - stepHeight;
      }
    }
  } else if (stepCounter > 0 && stepCounter <= stepNumber && useOffroad) {
    // do this if the offroad mode is activated.

    if (stepCounter <= ceil(stepNumber * 0.80)) {
      //Only for the first 85% of steps, the last 15% are soley for lowering the leg
      // interpolate the current leg position. Using the map() function will result in a linear motion to the final position
      for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 3; ++j) {
          newPositions[i][j] = map(stepCounter, 1, ceil(stepNumber * 0.8), prevPositions[i][j], finalPositions[i][j]);
        }
      }
      if (stepCounter <= ceil(stepNumber * 0.15)) {
        // raise the corresponding legs in the first 15% time
        if (leftLegHome) {
          // lift legs FR, ML, RR
          newPositions[0][2] = map(stepCounter, 1, ceil(stepNumber * 0.15), prevPositions[0][2], prevPositions[0][2] - stepHeight);
          newPositions[3][2] = map(stepCounter, 1, ceil(stepNumber * 0.15), prevPositions[3][2], prevPositions[3][2] - stepHeight);
          newPositions[4][2] = map(stepCounter, 1, ceil(stepNumber * 0.15), prevPositions[4][2], prevPositions[4][2] - stepHeight);
        } else {
          // lift legs FL, MR, RL
          newPositions[1][2] = map(stepCounter, 1, ceil(stepNumber * 0.15), prevPositions[1][2], prevPositions[1][2] - stepHeight);
          newPositions[2][2] = map(stepCounter, 1, ceil(stepNumber * 0.15), prevPositions[2][2], prevPositions[2][2] - stepHeight);
          newPositions[5][2] = map(stepCounter, 1, ceil(stepNumber * 0.15), prevPositions[5][2], prevPositions[5][2] - stepHeight);
        }
      } else {
        // do this for steps 15%-85%
        if (leftLegHome) {
          // lower legs FR, ML, RR
          newPositions[0][2] = prevPositions[0][2] - stepHeight;
          newPositions[3][2] = prevPositions[3][2] - stepHeight;
          newPositions[4][2] = prevPositions[4][2] - stepHeight;
        } else {
          // lower legs FL, MR, RL
          newPositions[1][2] = prevPositions[1][2] - stepHeight;
          newPositions[2][2] = prevPositions[2][2] - stepHeight;
          newPositions[5][2] = prevPositions[5][2] - stepHeight;
        }
      }
    } else if (stepCounter <= stepNumber - 3) {
      // do this for 80%-n-3 of the step. Only lowers the legs
      if (leftLegHome) {
        // lower legs FR, ML, RR
        if (!legFR.touchesGround()) {
          // lower the leg if it doesn't touch the ground
          newPositions[0][2] = map(stepCounter, ceil(stepNumber * 0.8), stepNumber - 3, prevPositions[0][2] - stepHeight, finalPositions[0][2] + stepHeight);
        }
        if (!legML.touchesGround()) {
          newPositions[3][2] = map(stepCounter, ceil(stepNumber * 0.8), stepNumber - 3, prevPositions[3][2] - stepHeight, finalPositions[3][2] + stepHeight);
        }
        if (!legRR.touchesGround()) {
          newPositions[4][2] = map(stepCounter, ceil(stepNumber * 0.8), stepNumber - 3, prevPositions[4][2] - stepHeight, finalPositions[4][2] + stepHeight);
        }

      } else {
        // lower legs FL, MR, RL
        if (!legFL.touchesGround()) {
          newPositions[1][2] = map(stepCounter, ceil(stepNumber * 0.8), stepNumber - 3, prevPositions[1][2] - stepHeight, finalPositions[1][2] + stepHeight);
        }
        if (!legMR.touchesGround()) {
          newPositions[2][2] = map(stepCounter, ceil(stepNumber * 0.8), stepNumber - 3, prevPositions[2][2] - stepHeight, finalPositions[2][2] + stepHeight);
        }
        if (!legRL.touchesGround()) {
          newPositions[5][2] = map(stepCounter, ceil(stepNumber * 0.8), stepNumber - 3, prevPositions[5][2] - stepHeight, finalPositions[5][2] + stepHeight);
        }
      }
    } else {
      // do this the last 3 iterations of the step
      // levels the robot and adjusts the height to an average of finalPositions[x][2]

      // do this only once (the first time this case is called)
      if (stepCounter == stepNumber - 2) {
        averageHeight = 0.0f;
        for (int i = 0; i < 6; ++i) {
          averageHeight += newPositions[i][2];
        }
        averageHeight /= 6.0f;
        // get the difference between target value and actual value
        averageHeight -= homePos[2];

        // divide by 3 since the legs are adjusted over a period of three iterations
        averageHeight /= 3.0;
      }
      // adjust the z coordinate of all legs by the difference
      newPositions[0][2] -= averageHeight;
      newPositions[3][2] -= averageHeight;
      newPositions[4][2] -= averageHeight;
      newPositions[1][2] -= averageHeight;
      newPositions[2][2] -= averageHeight;
      newPositions[5][2] -= averageHeight;
    }
  }

  if (stepCounter == stepNumber) {
    // do this if the max number of iterations is reached
    // if the legs FR, ML and RR were at home position prior to the step, FL, MR and RL will now be at home position and vice versa
    if (rightLegHome) {
      leftLegHome = true;
      rightLegHome = false;
    } else if (leftLegHome) {
      leftLegHome = false;
      rightLegHome = true;
    }
    // copy the final positions (without translation and rotation) in the current position array
    for (int i = 0; i < 6; ++i) {
      for (int j = 0; j < 3; ++j) {
        prevPositions[i][j] = newPositions[i][j];
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

bool Hexapod::calcCrabwalkFlush(int prevPositions[6][3], int newPositions[6][3], float stepDirection, uint8_t radius, uint16_t stepNumber, uint8_t stepHeight) {
  /*
   * new method for the second crad walk version
   *
   *
   */


  // figure out, which three legs will be lifted and which legs will stay on the ground
  if (stepCounter == 1) {
    action = 5;
    if ((prevRightLeg == false && abs(stepDirection - prevDirection) <= PI / 2) || (prevRightLeg == true && abs(stepDirection - prevDirection) > PI / 2)) {
      // move fr, ml, rr and lift fl, mr, rl
      moveRightLeg = true;
      int intersections[2][2] = { 0 };
      //------------------------------
      // Front Left Leg
      //------------------------------
      finalPositions[1][0] = homePos[0] + radius * cos(stepDirection + cornerLegAngle * 5) * 1.0;
      finalPositions[1][1] = homePos[1] + radius * sin(stepDirection + cornerLegAngle * 5) * 1.0;
      //------------------------------
      // Middle Right Leg
      //------------------------------
      // move the leg to the optimal position for the next step, assuming the next step is in the same direction
      finalPositions[2][0] = homePos[0] + radius * cos(stepDirection) * 1.0;
      finalPositions[2][1] = homePos[1] + radius * sin(stepDirection) * 1.0;
      //------------------------------
      // Rear Left Leg
      //------------------------------
      finalPositions[5][0] = homePos[0] + radius * cos(stepDirection + cornerLegAngle * 7) * 1.0;
      finalPositions[5][1] = homePos[1] + radius * sin(stepDirection + cornerLegAngle * 7) * 1.0;

      //------------------------------
      // Front Right Leg
      //------------------------------
      lineCircleIntersect(homePos[0], homePos[1], radius, prevPositions[0][0], prevPositions[0][1], stepDirection + cornerLegAngle, intersections);
      int index = getOppositeIntersection(prevPositions[0][0], prevPositions[0][1], stepDirection + cornerLegAngle, intersections);
      // intersections[index][0] and intersections[index][1] are the final x/y leg positions (if they exist)
      if (intersections[index][0] != -1 && intersections[index][1] != -1) {
        finalPositions[0][0] = intersections[index][0];
        finalPositions[0][1] = intersections[index][1];
      } else {
        // if there is no intersection, just leave the legs where they are
        finalPositions[0][0] = prevPositions[0][0];
        finalPositions[0][1] = prevPositions[0][1];
      }

      //------------------------------
      // Middle Left Leg
      //------------------------------
      lineCircleIntersect(homePos[0], homePos[1], radius, prevPositions[3][0], prevPositions[3][1], stepDirection + cornerLegAngle * 6, intersections);
      index = getOppositeIntersection(prevPositions[3][0], prevPositions[3][1], stepDirection + cornerLegAngle * 6, intersections);
      // intersections[index][0] and intersections[index][1] are the final x/y leg positions (if they exist)
      if (intersections[index][0] != -1 && intersections[index][1] != -1) {
        finalPositions[3][0] = intersections[index][0];
        finalPositions[3][1] = intersections[index][1];
      } else {
        // if there is no intersection, just leave the legs where they are
        finalPositions[3][0] = prevPositions[3][0];
        finalPositions[3][1] = prevPositions[3][1];
      }

      //------------------------------
      // Rear Right Leg
      //------------------------------
      lineCircleIntersect(homePos[0], homePos[1], radius, prevPositions[4][0], prevPositions[4][1], stepDirection - cornerLegAngle, intersections);
      index = getOppositeIntersection(prevPositions[4][0], prevPositions[4][1], stepDirection - cornerLegAngle, intersections);
      // intersections[index][0] and intersections[index][1] are the final x/y leg positions (if they exist)
      if (intersections[index][0] != -1 && intersections[index][1] != -1) {
        finalPositions[4][0] = intersections[index][0];
        finalPositions[4][1] = intersections[index][1];
      } else {
        // if there is no intersection, just leave the legs where they are
        finalPositions[4][0] = prevPositions[4][0];
        finalPositions[4][1] = prevPositions[4][1];
      }

    } else {
      // move fl, mr, rl and lift fr, ml, rr
      moveRightLeg = false;
      int intersections[2][2] = { 0 };

      //------------------------------
      // Front Right Leg
      //------------------------------
      finalPositions[0][0] = homePos[0] + radius * cos(stepDirection + cornerLegAngle) * 1.0;
      finalPositions[0][1] = homePos[1] + radius * sin(stepDirection + cornerLegAngle) * 1.0;
      //------------------------------
      // Middle Left Leg
      //------------------------------
      finalPositions[3][0] = homePos[0] + radius * cos(stepDirection + cornerLegAngle * 6) * 1.0;
      finalPositions[3][1] = homePos[1] + radius * sin(stepDirection + cornerLegAngle * 6) * 1.0;
      //------------------------------
      // Rear Right Leg
      //------------------------------
      finalPositions[4][0] = homePos[0] + radius * cos(stepDirection - cornerLegAngle) * 1.0;
      finalPositions[4][1] = homePos[1] + radius * sin(stepDirection - cornerLegAngle) * 1.0;
      //------------------------------
      // Front Left Leg
      //------------------------------
      lineCircleIntersect(homePos[0], homePos[1], radius, prevPositions[1][0], prevPositions[1][1], stepDirection + cornerLegAngle * 5, intersections);
      int index = getOppositeIntersection(prevPositions[1][0], prevPositions[1][1], stepDirection + cornerLegAngle * 5, intersections);
      // intersections[index][0] and intersections[index][1] are the final x/y leg positions (if they exist)
      if (intersections[index][0] != -1 && intersections[index][1] != -1) {
        finalPositions[1][0] = intersections[index][0];
        finalPositions[1][1] = intersections[index][1];
      } else {
        // if there is no intersection, just leave the legs where they are
        finalPositions[1][0] = prevPositions[1][0];
        finalPositions[1][1] = prevPositions[1][1];
      }
      //------------------------------
      // Middle Right Leg
      //------------------------------
      lineCircleIntersect(homePos[0], homePos[1], radius, prevPositions[2][0], prevPositions[2][1], stepDirection, intersections);
      index = getOppositeIntersection(prevPositions[2][0], prevPositions[2][1], stepDirection, intersections);
      // intersections[index][0] and intersections[index][1] are the final x/y leg positions (if they exist)
      if (intersections[index][0] != -1 && intersections[index][1] != -1) {
        finalPositions[2][0] = intersections[index][0];
        finalPositions[2][1] = intersections[index][1];
      } else {
        // if there is no intersection, just leave the legs where they are
        finalPositions[2][0] = prevPositions[2][0];
        finalPositions[2][1] = prevPositions[2][1];
      }
      //------------------------------
      // Rear Left Leg
      //------------------------------
      lineCircleIntersect(homePos[0], homePos[1], radius, prevPositions[5][0], prevPositions[5][1], stepDirection + cornerLegAngle * 7, intersections);
      index = getOppositeIntersection(prevPositions[5][0], prevPositions[5][1], stepDirection + cornerLegAngle * 7, intersections);
      // intersections[index][0] and intersections[index][1] are the final x/y leg positions (if they exist)
      if (intersections[index][0] != -1 && intersections[index][1] != -1) {
        finalPositions[5][0] = intersections[index][0];
        finalPositions[5][1] = intersections[index][1];
      } else {
        // if there is no intersection, just leave the legs where they are
        finalPositions[5][0] = prevPositions[5][0];
        finalPositions[5][1] = prevPositions[5][1];
      }
    }
  }

  // actually move the legs to the mapped position
  if (stepCounter > 0 && stepCounter <= stepNumber) {
    // interpolate the current leg position. Using the map() function will result in a linear motion to the final position
    for (int i = 0; i < 6; ++i) {
      for (int j = 0; j < 3; ++j) {
        newPositions[i][j] = map(stepCounter, 1, stepNumber, prevPositions[i][j], finalPositions[i][j]);
      }
    }
    // this is necessary to lift the three legs returning home of the ground
    if (stepCounter < ceil(stepNumber * 0.15)) {
      // lift the three legs returning to home position
      if (moveRightLeg == false) {
        // lift legs FR, ML, RR
        newPositions[0][2] = map(stepCounter, 1, ceil(stepNumber * 0.15), prevPositions[0][2], prevPositions[0][2] - stepHeight);
        newPositions[3][2] = map(stepCounter, 1, ceil(stepNumber * 0.15), prevPositions[3][2], prevPositions[3][2] - stepHeight);
        newPositions[4][2] = map(stepCounter, 1, ceil(stepNumber * 0.15), prevPositions[4][2], prevPositions[4][2] - stepHeight);
      } else {
        // lift legs FL, MR, RL
        newPositions[1][2] = map(stepCounter, 1, ceil(stepNumber * 0.15), prevPositions[1][2], prevPositions[1][2] - stepHeight);
        newPositions[2][2] = map(stepCounter, 1, ceil(stepNumber * 0.15), prevPositions[2][2], prevPositions[2][2] - stepHeight);
        newPositions[5][2] = map(stepCounter, 1, ceil(stepNumber * 0.15), prevPositions[5][2], prevPositions[5][2] - stepHeight);
      }
    } else if (stepCounter > ceil(stepNumber * 0.85)) {
      if (moveRightLeg == false) {
        // lower legs FR, ML, RR
        newPositions[0][2] = map(stepCounter, ceil(stepNumber * 0.85), stepNumber, prevPositions[0][2] - stepHeight, finalPositions[0][2]);
        newPositions[3][2] = map(stepCounter, ceil(stepNumber * 0.85), stepNumber, prevPositions[3][2] - stepHeight, finalPositions[3][2]);
        newPositions[4][2] = map(stepCounter, ceil(stepNumber * 0.85), stepNumber, prevPositions[4][2] - stepHeight, finalPositions[4][2]);
      } else {
        // lower legs FL, MR, RL
        newPositions[1][2] = map(stepCounter, ceil(stepNumber * 0.85), stepNumber, prevPositions[1][2] - stepHeight, finalPositions[1][2]);
        newPositions[2][2] = map(stepCounter, ceil(stepNumber * 0.85), stepNumber, prevPositions[2][2] - stepHeight, finalPositions[2][2]);
        newPositions[5][2] = map(stepCounter, ceil(stepNumber * 0.85), stepNumber, prevPositions[5][2] - stepHeight, finalPositions[5][2]);
      }
    } else {
      if (moveRightLeg == false) {
        // keep FR, ML, RR lifted
        newPositions[0][2] = prevPositions[0][2] - stepHeight;
        newPositions[3][2] = prevPositions[3][2] - stepHeight;
        newPositions[4][2] = prevPositions[4][2] - stepHeight;
      } else {
        // keep FL, MR, RL lifted
        newPositions[1][2] = prevPositions[1][2] - stepHeight;
        newPositions[2][2] = prevPositions[2][2] - stepHeight;
        newPositions[5][2] = prevPositions[5][2] - stepHeight;
      }
    }
  }


  // transform the final point in the three local frames
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
    return true;
  }
  stepCounter++;
  return false;
}

bool Hexapod::calcRotatingStep(int prevPositions[6][3], int newPositions[6][3], float stepAngle, uint16_t stepNumber, uint8_t stepHeight) {
  /*
   * calculates the next position for a rotating step on the spot
   * 
   * prevPositions: array containing all leg positions prior to the step. Only gets updated in the last iteration when the step is finished
   * newPositions:  array to save the new leg positions to
   * stepAngle:     angle by which the robot is rotated
   * stepNumber:    number of iterations for one whole step
   * stepHeight:    distance (in mm) by which the legs are raised to return home. Default: 20mm
   * 
   */

  if (rightLegHome && stepCounter == 1) {
    // do this in the first iteration if three legs are at home position
    leftLegHome = false;
    moveRightLeg = true;  // the legs FR, ML and RR are the ones who move on the floor while FL, MR and RL return to home position
    action = 2;           // the hexapod is executing a step
    float result[3] = { 0.0, 0.0, 0.0 };

    float rotMatrix[3][3] = { { cos(stepAngle), -sin(stepAngle), 0 },
                              { sin(stepAngle), cos(stepAngle), 0 },
                              { 0, 0, 1 } };

    // calculate the end position for each leg and save it along with the starting position and later interpolate the position for each iteration
    // FR, ML and RR will move to a new position while FL, MR and RL will return to home position
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 6; ++j) {
        finalPositions[j][i] = homePos[i];
      }
    }

    // ++++++++++++++++++++++++++++++
    // leg front right
    // ++++++++++++++++++++++++++++++
    // rotate the local coordinate system by 30°
    int temp = finalPositions[0][0];
    finalPositions[0][0] = finalPositions[0][0] * cos(-cornerLegAngle) - finalPositions[0][1] * sin(-cornerLegAngle);
    finalPositions[0][1] = temp * sin(-cornerLegAngle) + finalPositions[0][1] * cos(-cornerLegAngle);

    //shift the local coordinate system to the center of mass
    finalPositions[0][0] += cornerLegXDistGlobal;
    finalPositions[0][1] += cornerLegYDistGlobal;

    // rotating the end point using RotMatrix
    for (int i = 0; i < 3; ++i) {
      result[i] = 0.0;
    }

    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        result[i] += (rotMatrix[i][j] * finalPositions[0][j]);
      }
    }
    for (int i = 0; i < 3; ++i) {
      finalPositions[0][i] = result[i];
    }

    //shift the local coordinate system back
    finalPositions[0][0] -= cornerLegXDistGlobal;
    finalPositions[0][1] -= cornerLegYDistGlobal;

    //rotate the coordinate system back by -30°
    temp = finalPositions[0][0];
    finalPositions[0][0] = finalPositions[0][0] * cos(cornerLegAngle) - finalPositions[0][1] * sin(cornerLegAngle);
    finalPositions[0][1] = temp * sin(cornerLegAngle) + finalPositions[0][1] * cos(cornerLegAngle);

    // ++++++++++++++++++++++++++++++
    // leg mid left
    // ++++++++++++++++++++++++++++++
    // transformation from global to local coordinate system:
    //shift
    finalPositions[3][1] += centerLegYDist;
    //and rotate by 180°
    finalPositions[3][0] = -finalPositions[3][0];
    finalPositions[3][1] = -finalPositions[3][1];

    // rotating the end point using RotMatrix
    for (int i = 0; i < 3; ++i) {
      result[i] = 0.0;
    }

    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        result[i] += (rotMatrix[i][j] * finalPositions[3][j]);
      }
    }
    for (int i = 0; i < 3; ++i) {
      finalPositions[3][i] = result[i];
    }

    // transformation back to the local coordinate system
    //rotate back by -180°
    finalPositions[3][0] = -finalPositions[3][0];
    finalPositions[3][1] = -finalPositions[3][1];
    // shift back
    finalPositions[3][1] -= centerLegYDist;

    // ++++++++++++++++++++++++++++++
    // leg rear right
    // ++++++++++++++++++++++++++++++
    // rotate the local coordinate system by -30°
    temp = finalPositions[4][0];
    finalPositions[4][0] = finalPositions[4][0] * cos(cornerLegAngle) - finalPositions[4][1] * sin(cornerLegAngle);
    finalPositions[4][1] = temp * sin(cornerLegAngle) + finalPositions[4][1] * cos(cornerLegAngle);

    //shift the local coordinate system to the center of mass
    finalPositions[4][0] -= cornerLegXDistGlobal;
    finalPositions[4][1] += cornerLegYDistGlobal;

    // rotating the end point using RotMatrix
    for (int i = 0; i < 3; ++i) {
      result[i] = 0.0;
    }

    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        result[i] += (rotMatrix[i][j] * finalPositions[4][j]);
      }
    }
    for (int i = 0; i < 3; ++i) {
      finalPositions[4][i] = result[i];
    }

    //shift the local coordinate system back
    finalPositions[4][0] += cornerLegXDistGlobal;
    finalPositions[4][1] -= cornerLegYDistGlobal;

    //rotate the coordinate system back by 30°
    temp = finalPositions[4][0];
    finalPositions[4][0] = finalPositions[4][0] * cos(-cornerLegAngle) - finalPositions[4][1] * sin(-cornerLegAngle);
    finalPositions[4][1] = temp * sin(-cornerLegAngle) + finalPositions[4][1] * cos(-cornerLegAngle);

  } else if (leftLegHome && stepCounter == 1) {
    rightLegHome = false;
    moveRightLeg = false;
    action = 2;  // the hexapod is executing a step
    float result[3] = { 0.0, 0.0, 0.0 };

    float rotMatrix[3][3] = { { cos(stepAngle), -sin(stepAngle), 0 },
                              { sin(stepAngle), cos(stepAngle), 0 },
                              { 0, 0, 1 } };

    // calculate the end position for each leg and save it along with the starting position and later interpolate the position for each iteration
    // FL, MR and RL will move to a new position while FR, ML and RR will return to home position
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 6; ++j) {
        finalPositions[j][i] = homePos[i];
      }
    }

    // ++++++++++++++++++++++++++++++
    // leg front left
    // ++++++++++++++++++++++++++++++
    // rotate the local coordinate system by 180°-30°
    int temp = finalPositions[1][0];
    finalPositions[1][0] = finalPositions[1][0] * cos(-PI + cornerLegAngle) - finalPositions[1][1] * sin(-PI + cornerLegAngle);
    finalPositions[1][1] = temp * sin(-PI + cornerLegAngle) + finalPositions[1][1] * cos(-PI + cornerLegAngle);

    //shift the local coordinate system to the center of mass
    finalPositions[1][0] += cornerLegXDistGlobal;
    finalPositions[1][1] -= cornerLegYDistGlobal;

    // rotating the end point using RotMatrix
    for (int i = 0; i < 3; ++i) {
      result[i] = 0.0;
    }

    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        result[i] += (rotMatrix[i][j] * finalPositions[1][j]);
      }
    }
    for (int i = 0; i < 3; ++i) {
      finalPositions[1][i] = result[i];
    }

    //shift the local coordinate system back
    finalPositions[1][0] -= cornerLegXDistGlobal;
    finalPositions[1][1] += cornerLegYDistGlobal;

    //rotate the coordinate system back by -180°+30°
    temp = finalPositions[1][0];
    finalPositions[1][0] = finalPositions[1][0] * cos(PI - cornerLegAngle) - finalPositions[1][1] * sin(PI - cornerLegAngle);
    finalPositions[1][1] = temp * sin(PI - cornerLegAngle) + finalPositions[1][1] * cos(PI - cornerLegAngle);

    // ++++++++++++++++++++++++++++++
    // leg mid right
    // ++++++++++++++++++++++++++++++
    // transformation from local to global coordinate system:
    finalPositions[2][1] += centerLegYDist;

    // rotating the end point using RotMatrix
    for (int i = 0; i < 3; ++i) {
      result[i] = 0.0;  //set the result array to 0
    }

    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        result[i] += (rotMatrix[i][j] * finalPositions[2][j]);  // matrix-vector-multiplication: rotMatrix * (x, y, z)
      }
    }
    for (int i = 0; i < 3; ++i) {
      finalPositions[2][i] = result[i];
    }

    // transformation back to the local coordinate system
    finalPositions[2][1] -= centerLegYDist;

    // ++++++++++++++++++++++++++++++
    // leg rear left
    // ++++++++++++++++++++++++++++++
    // rotate the local coordinate system by 180°+30°
    temp = finalPositions[5][0];
    finalPositions[5][0] = finalPositions[5][0] * cos(-PI - cornerLegAngle) - finalPositions[5][1] * sin(-PI - cornerLegAngle);
    finalPositions[5][1] = temp * sin(-PI - cornerLegAngle) + finalPositions[5][1] * cos(-PI - cornerLegAngle);

    //shift the local coordinate system to the center of mass
    finalPositions[5][0] -= cornerLegXDistGlobal;
    finalPositions[5][1] -= cornerLegYDistGlobal;

    // rotating the end point using RotMatrix
    for (int i = 0; i < 3; ++i) {
      result[i] = 0.0;
    }

    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        result[i] += (rotMatrix[i][j] * finalPositions[5][j]);
      }
    }
    for (int i = 0; i < 3; ++i) {
      finalPositions[5][i] = result[i];
    }

    //shift the local coordinate system back
    finalPositions[5][0] += cornerLegXDistGlobal;
    finalPositions[5][1] += cornerLegYDistGlobal;

    //rotate the coordinate system back by -180°-30°
    temp = finalPositions[5][0];
    finalPositions[5][0] = finalPositions[5][0] * cos(PI + cornerLegAngle) - finalPositions[5][1] * sin(PI + cornerLegAngle);
    finalPositions[5][1] = temp * sin(PI + cornerLegAngle) + finalPositions[5][1] * cos(PI + cornerLegAngle);

  } else if (stepCounter == 1) {
    //do this if the legs are not at home position
  }

  if (stepCounter > 0 && stepCounter <= stepNumber) {
    // interpolate the current leg position. Using the map() function will result in a linear motion to the final position
    for (int i = 0; i < 6; ++i) {
      for (int j = 0; j < 3; ++j) {
        newPositions[i][j] = map(stepCounter, 1, stepNumber, prevPositions[i][j], finalPositions[i][j]);
      }
    }
    // this is necessary to lift the three legs returning home of the ground
    if (stepCounter < ceil(stepNumber * 0.15)) {
      // lift the three legs returning to home position
      if (leftLegHome) {
        // lift legs FR, ML, RR
        newPositions[0][2] = map(stepCounter, 1, ceil(stepNumber * 0.15), prevPositions[0][2], prevPositions[0][2] - stepHeight);
        newPositions[3][2] = map(stepCounter, 1, ceil(stepNumber * 0.15), prevPositions[3][2], prevPositions[3][2] - stepHeight);
        newPositions[4][2] = map(stepCounter, 1, ceil(stepNumber * 0.15), prevPositions[4][2], prevPositions[4][2] - stepHeight);
      } else {
        // lift legs FL, MR, RL
        newPositions[1][2] = map(stepCounter, 1, ceil(stepNumber * 0.15), prevPositions[1][2], prevPositions[1][2] - stepHeight);
        newPositions[2][2] = map(stepCounter, 1, ceil(stepNumber * 0.15), prevPositions[2][2], prevPositions[2][2] - stepHeight);
        newPositions[5][2] = map(stepCounter, 1, ceil(stepNumber * 0.15), prevPositions[5][2], prevPositions[5][2] - stepHeight);
      }
    } else if (stepCounter > ceil(stepNumber * 0.85)) {
      if (leftLegHome) {
        // lower legs FR, ML, RR
        newPositions[0][2] = map(stepCounter, ceil(stepNumber * 0.85), stepNumber, prevPositions[0][2] - stepHeight, finalPositions[0][2]);
        newPositions[3][2] = map(stepCounter, ceil(stepNumber * 0.85), stepNumber, prevPositions[3][2] - stepHeight, finalPositions[3][2]);
        newPositions[4][2] = map(stepCounter, ceil(stepNumber * 0.85), stepNumber, prevPositions[4][2] - stepHeight, finalPositions[4][2]);
      } else {
        // lower legs FL, MR, RL
        newPositions[1][2] = map(stepCounter, ceil(stepNumber * 0.85), stepNumber, prevPositions[1][2] - stepHeight, finalPositions[1][2]);
        newPositions[2][2] = map(stepCounter, ceil(stepNumber * 0.85), stepNumber, prevPositions[2][2] - stepHeight, finalPositions[2][2]);
        newPositions[5][2] = map(stepCounter, ceil(stepNumber * 0.85), stepNumber, prevPositions[5][2] - stepHeight, finalPositions[5][2]);
      }
    } else {
      if (leftLegHome) {
        // lower legs FR, ML, RR
        newPositions[0][2] = prevPositions[0][2] - stepHeight;
        newPositions[3][2] = prevPositions[3][2] - stepHeight;
        newPositions[4][2] = prevPositions[4][2] - stepHeight;
      } else {
        // lower legs FL, MR, RL
        newPositions[1][2] = prevPositions[1][2] - stepHeight;
        newPositions[2][2] = prevPositions[2][2] - stepHeight;
        newPositions[5][2] = prevPositions[5][2] - stepHeight;
      }
    }
  }


  if (stepCounter == stepNumber) {
    // do this if the max number of iterations is reached
    // if the legs FR, ML and RR were at home position prior to the step, FL, MR and RL will now be at home position and vice versa
    if (rightLegHome) {
      leftLegHome = true;
      rightLegHome = false;
    } else if (leftLegHome) {
      leftLegHome = false;
      rightLegHome = true;
    }
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
    return true;
  }
  stepCounter++;
  return false;
}

bool Hexapod::calcRadiusStep(int prevPositions[6][3], int newPositions[6][3], uint8_t stepDist, float stepAngle, bool direction, uint16_t stepNumber, uint8_t stepHeight) {
  /*
   * calculates the next position for a step (forwards or backwards) with a specified turning radius
   * 
   * prevPositions: array containing all leg positions prior to the step. Only gets updated in the last iteration when the step is finished
   * newPositions:  array to save the new leg positions to
   * stepDist:      distance of the step (measured along the radius), only positive values
   * stepAngle:     angle by which the robot turns during the step. Positive = left, negative = right
   * direction:     true: step forwards, false: step backwards
   * stepNumber:    number of iterations for one whole step
   * stepHeight:    distance (in mm) by which the legs are raised to return home. Default: 20mm
   * 
   */

  if (rightLegHome && stepCounter == 1) {
    // do this in the first iteration if three legs are at home position
    leftLegHome = false;
    moveRightLeg = true;  // the legs FR, ML and RR are the ones who move on the floor while FL, MR and RL return to home position
    action = 3;           // the hexapod is executing a step

    // catch some errors
    // stepAngle is only allowed in the intervals [-0.2, -0.0001] and [0.0001, 0.2] otherwise radius will be too big or turning angle will be too big
    if (stepAngle > 0.0f && stepAngle < 0.0001f) {
      stepAngle = 0.0001f;
    } else if (stepAngle < 0.0f && stepAngle > -0.0001f) {
      stepAngle = -0.0001f;
    } else if (stepAngle > 0.2f) {
      stepAngle = 0.2f;
    } else if (stepAngle < -0.2f) {
      stepAngle = -0.2f;
    }

    // calculate the radius of the circle
    int32_t radius = (float)stepDist / (2.0 * sin(stepAngle / 2.0));

    // make a step backwards if direction is false
    if (!direction) {
      stepAngle = -stepAngle;
    }

    float result[3] = { 0.0, 0.0, 0.0 };

    float rotMatrix[3][3] = { { cos(stepAngle), -sin(stepAngle), 0 },
                              { sin(stepAngle), cos(stepAngle), 0 },
                              { 0, 0, 1 } };

    // calculate the end position for each leg and save it along with the starting position and later interpolate the position for each iteration
    // FR, ML and RR will move to a new position while FL, MR and RL will return to home position
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 6; ++j) {
        finalPositions[j][i] = homePos[i];
      }
    }

    // ++++++++++++++++++++++++++++++
    // leg front right
    // ++++++++++++++++++++++++++++++
    // rotate the local coordinate system by 30°
    int temp = finalPositions[0][0];
    finalPositions[0][0] = finalPositions[0][0] * cos(-cornerLegAngle) - finalPositions[0][1] * sin(-cornerLegAngle);
    finalPositions[0][1] = temp * sin(-cornerLegAngle) + finalPositions[0][1] * cos(-cornerLegAngle);

    //shift the local coordinate system to the center of mass
    finalPositions[0][0] += cornerLegXDistGlobal;
    finalPositions[0][1] += cornerLegYDistGlobal;

    // shift the local coordinate system so that the rotation will be performed around the centre of the turning circle
    finalPositions[0][1] += radius;

    // rotating the end point using RotMatrix
    for (int i = 0; i < 3; ++i) {
      result[i] = 0.0;
    }

    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        result[i] += (rotMatrix[i][j] * finalPositions[0][j]);
      }
    }
    for (int i = 0; i < 3; ++i) {
      finalPositions[0][i] = result[i];
    }

    // shift the local coordinate system back by the radius ammount
    finalPositions[0][1] -= radius;

    //shift the local coordinate system back
    finalPositions[0][0] -= cornerLegXDistGlobal;
    finalPositions[0][1] -= cornerLegYDistGlobal;

    //rotate the coordinate system back by -30°
    temp = finalPositions[0][0];
    finalPositions[0][0] = finalPositions[0][0] * cos(cornerLegAngle) - finalPositions[0][1] * sin(cornerLegAngle);
    finalPositions[0][1] = temp * sin(cornerLegAngle) + finalPositions[0][1] * cos(cornerLegAngle);

    // ++++++++++++++++++++++++++++++
    // leg mid left
    // ++++++++++++++++++++++++++++++
    // transformation from global to local coordinate system:
    //shift
    finalPositions[3][1] += centerLegYDist;

    // shift the local coordinate system so that the rotation will be performed around the centre of the turning circle
    // this leg is shifted in the other direction
    finalPositions[3][1] -= radius;
    //and rotate by 180°
    finalPositions[3][0] = -finalPositions[3][0];
    finalPositions[3][1] = -finalPositions[3][1];

    // rotating the end point using RotMatrix
    for (int i = 0; i < 3; ++i) {
      result[i] = 0.0;
    }

    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        result[i] += (rotMatrix[i][j] * finalPositions[3][j]);
      }
    }
    for (int i = 0; i < 3; ++i) {
      finalPositions[3][i] = result[i];
    }

    // transformation back to the local coordinate system
    //rotate back by -180°
    finalPositions[3][0] = -finalPositions[3][0];
    finalPositions[3][1] = -finalPositions[3][1];
    // shift back
    finalPositions[3][1] -= centerLegYDist;
    // shift the local coordinate system back by radius
    finalPositions[3][1] += radius;

    // ++++++++++++++++++++++++++++++
    // leg rear right
    // ++++++++++++++++++++++++++++++
    // rotate the local coordinate system by -30°
    temp = finalPositions[4][0];
    finalPositions[4][0] = finalPositions[4][0] * cos(cornerLegAngle) - finalPositions[4][1] * sin(cornerLegAngle);
    finalPositions[4][1] = temp * sin(cornerLegAngle) + finalPositions[4][1] * cos(cornerLegAngle);

    //shift the local coordinate system to the center of mass
    finalPositions[4][0] -= cornerLegXDistGlobal;
    finalPositions[4][1] += cornerLegYDistGlobal;

    // shift the local coordinate system so that the rotation will be performed around the centre of the turning circle
    finalPositions[4][1] += radius;

    // rotating the end point using RotMatrix
    for (int i = 0; i < 3; ++i) {
      result[i] = 0.0;
    }

    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        result[i] += (rotMatrix[i][j] * finalPositions[4][j]);
      }
    }
    for (int i = 0; i < 3; ++i) {
      finalPositions[4][i] = result[i];
    }

    //shift the local coordinate system back
    finalPositions[4][0] += cornerLegXDistGlobal;
    finalPositions[4][1] -= cornerLegYDistGlobal;

    // shift the local coordinate system back by radius
    finalPositions[4][1] -= radius;

    //rotate the coordinate system back by 30°
    temp = finalPositions[4][0];
    finalPositions[4][0] = finalPositions[4][0] * cos(-cornerLegAngle) - finalPositions[4][1] * sin(-cornerLegAngle);
    finalPositions[4][1] = temp * sin(-cornerLegAngle) + finalPositions[4][1] * cos(-cornerLegAngle);

  } else if (leftLegHome && stepCounter == 1) {
    rightLegHome = false;
    moveRightLeg = false;
    action = 3;  // the hexapod is executing a step

    // catch some errors
    // stepAngle is only allowed in the intervals [-0.2, -0.0001] and [0.0001, 0.2] otherwise radius will be too big or turning angle will be too big
    if (stepAngle > 0.0f && stepAngle < 0.0001f) {
      stepAngle = 0.0001f;
    } else if (stepAngle < 0.0f && stepAngle > -0.0001f) {
      stepAngle = -0.0001f;
    } else if (stepAngle > 0.2f) {
      stepAngle = 0.2f;
    } else if (stepAngle < -0.2f) {
      stepAngle = -0.2f;
    }

    // calculate the radius using simple trigonomics
    int32_t radius = (float)stepDist / (2.0 * sin(stepAngle / 2.0));
    radius = -radius;

    // make a step backwards if direction is false
    if (!direction) {
      stepAngle = -stepAngle;
    }
    float result[3] = { 0.0, 0.0, 0.0 };

    float rotMatrix[3][3] = { { cos(stepAngle), -sin(stepAngle), 0 },
                              { sin(stepAngle), cos(stepAngle), 0 },
                              { 0, 0, 1 } };

    // calculate the end position for each leg and save it along with the starting position and later interpolate the position for each iteration
    // FL, MR and RL will move to a new position while FR, ML and RR will return to home position
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 6; ++j) {
        finalPositions[j][i] = homePos[i];
      }
    }

    // ++++++++++++++++++++++++++++++
    // leg front left
    // ++++++++++++++++++++++++++++++
    // rotate the local coordinate system by 180°-30°
    int temp = finalPositions[1][0];
    finalPositions[1][0] = finalPositions[1][0] * cos(-PI + cornerLegAngle) - finalPositions[1][1] * sin(-PI + cornerLegAngle);
    finalPositions[1][1] = temp * sin(-PI + cornerLegAngle) + finalPositions[1][1] * cos(-PI + cornerLegAngle);

    //shift the local coordinate system to the center of mass
    finalPositions[1][0] += cornerLegXDistGlobal;
    finalPositions[1][1] -= cornerLegYDistGlobal;

    // shift the local coordinate system so that the rotation will be performed around the centre of the turning circle
    finalPositions[1][1] -= radius;

    // rotating the end point using RotMatrix
    for (int i = 0; i < 3; ++i) {
      result[i] = 0.0;
    }

    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        result[i] += (rotMatrix[i][j] * finalPositions[1][j]);
      }
    }
    for (int i = 0; i < 3; ++i) {
      finalPositions[1][i] = result[i];
    }

    // shift the local coordinate system back
    finalPositions[1][1] += radius;

    //shift the local coordinate system back
    finalPositions[1][0] -= cornerLegXDistGlobal;
    finalPositions[1][1] += cornerLegYDistGlobal;

    //rotate the coordinate system back by -180°+30°
    temp = finalPositions[1][0];
    finalPositions[1][0] = finalPositions[1][0] * cos(PI - cornerLegAngle) - finalPositions[1][1] * sin(PI - cornerLegAngle);
    finalPositions[1][1] = temp * sin(PI - cornerLegAngle) + finalPositions[1][1] * cos(PI - cornerLegAngle);

    // ++++++++++++++++++++++++++++++
    // leg mid right
    // ++++++++++++++++++++++++++++++
    // transformation from local to global coordinate system:
    finalPositions[2][1] += centerLegYDist;

    // shift the local coordinate system so that the rotation will be performed around the centre of the turning circle
    // this leg is shifted in the other direction
    finalPositions[2][1] -= radius;

    // rotating the end point using RotMatrix
    for (int i = 0; i < 3; ++i) {
      result[i] = 0.0;  //set the result array to 0
    }

    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        result[i] += (rotMatrix[i][j] * finalPositions[2][j]);  // matrix-vector-multiplication: rotMatrix * (x, y, z)
      }
    }
    for (int i = 0; i < 3; ++i) {
      finalPositions[2][i] = result[i];
    }

    // shift the local coordinate system back
    finalPositions[2][1] += radius;

    // transformation back to the local coordinate system
    finalPositions[2][1] -= centerLegYDist;

    // ++++++++++++++++++++++++++++++
    // leg rear left
    // ++++++++++++++++++++++++++++++
    // rotate the local coordinate system by 180°+30°
    temp = finalPositions[5][0];
    finalPositions[5][0] = finalPositions[5][0] * cos(-PI - cornerLegAngle) - finalPositions[5][1] * sin(-PI - cornerLegAngle);
    finalPositions[5][1] = temp * sin(-PI - cornerLegAngle) + finalPositions[5][1] * cos(-PI - cornerLegAngle);

    //shift the local coordinate system to the center of mass
    finalPositions[5][0] -= cornerLegXDistGlobal;
    finalPositions[5][1] -= cornerLegYDistGlobal;

    // shift the local coordinate system so that the rotation will be performed around the centre of the turning circle
    finalPositions[5][1] -= radius;

    // rotating the end point using RotMatrix
    for (int i = 0; i < 3; ++i) {
      result[i] = 0.0;
    }

    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        result[i] += (rotMatrix[i][j] * finalPositions[5][j]);
      }
    }
    for (int i = 0; i < 3; ++i) {
      finalPositions[5][i] = result[i];
    }

    // shift the local coordinate system back
    finalPositions[5][1] += radius;

    //shift the local coordinate system back
    finalPositions[5][0] += cornerLegXDistGlobal;
    finalPositions[5][1] += cornerLegYDistGlobal;

    //rotate the coordinate system back by -180°-30°
    temp = finalPositions[5][0];
    finalPositions[5][0] = finalPositions[5][0] * cos(PI + cornerLegAngle) - finalPositions[5][1] * sin(PI + cornerLegAngle);
    finalPositions[5][1] = temp * sin(PI + cornerLegAngle) + finalPositions[5][1] * cos(PI + cornerLegAngle);

  } else if (stepCounter == 1) {
    //do this if the legs are not at home position
  }

  if (stepCounter > 0 && stepCounter <= stepNumber) {
    // interpolate the current leg position. Using the map() function will result in a linear motion to the final position
    for (int i = 0; i < 6; ++i) {
      for (int j = 0; j < 3; ++j) {
        newPositions[i][j] = map(stepCounter, 1, stepNumber, prevPositions[i][j], finalPositions[i][j]);
      }
    }
    // this is necessary to lift the three legs returning home of the ground
    if (stepCounter < ceil(stepNumber * 0.15)) {
      // lift the three legs returning to home position
      if (leftLegHome) {
        // lift legs FR, ML, RR
        newPositions[0][2] = map(stepCounter, 1, ceil(stepNumber * 0.15), prevPositions[0][2], prevPositions[0][2] - stepHeight);
        newPositions[3][2] = map(stepCounter, 1, ceil(stepNumber * 0.15), prevPositions[3][2], prevPositions[3][2] - stepHeight);
        newPositions[4][2] = map(stepCounter, 1, ceil(stepNumber * 0.15), prevPositions[4][2], prevPositions[4][2] - stepHeight);
      } else {
        // lift legs FL, MR, RL
        newPositions[1][2] = map(stepCounter, 1, ceil(stepNumber * 0.15), prevPositions[1][2], prevPositions[1][2] - stepHeight);
        newPositions[2][2] = map(stepCounter, 1, ceil(stepNumber * 0.15), prevPositions[2][2], prevPositions[2][2] - stepHeight);
        newPositions[5][2] = map(stepCounter, 1, ceil(stepNumber * 0.15), prevPositions[5][2], prevPositions[5][2] - stepHeight);
      }
    } else if (stepCounter > ceil(stepNumber * 0.85)) {
      if (leftLegHome) {
        // lower legs FR, ML, RR
        newPositions[0][2] = map(stepCounter, ceil(stepNumber * 0.85), stepNumber, prevPositions[0][2] - stepHeight, finalPositions[0][2]);
        newPositions[3][2] = map(stepCounter, ceil(stepNumber * 0.85), stepNumber, prevPositions[3][2] - stepHeight, finalPositions[3][2]);
        newPositions[4][2] = map(stepCounter, ceil(stepNumber * 0.85), stepNumber, prevPositions[4][2] - stepHeight, finalPositions[4][2]);
      } else {
        // lower legs FL, MR, RL
        newPositions[1][2] = map(stepCounter, ceil(stepNumber * 0.85), stepNumber, prevPositions[1][2] - stepHeight, finalPositions[1][2]);
        newPositions[2][2] = map(stepCounter, ceil(stepNumber * 0.85), stepNumber, prevPositions[2][2] - stepHeight, finalPositions[2][2]);
        newPositions[5][2] = map(stepCounter, ceil(stepNumber * 0.85), stepNumber, prevPositions[5][2] - stepHeight, finalPositions[5][2]);
      }
    } else {
      if (leftLegHome) {
        // lower legs FR, ML, RR
        newPositions[0][2] = prevPositions[0][2] - stepHeight;
        newPositions[3][2] = prevPositions[3][2] - stepHeight;
        newPositions[4][2] = prevPositions[4][2] - stepHeight;
      } else {
        // lower legs FL, MR, RL
        newPositions[1][2] = prevPositions[1][2] - stepHeight;
        newPositions[2][2] = prevPositions[2][2] - stepHeight;
        newPositions[5][2] = prevPositions[5][2] - stepHeight;
      }
    }
  }


  if (stepCounter == stepNumber) {
    // do this if the max number of iterations is reached
    // if the legs FR, ML and RR were at home position prior to the step, FL, MR and RL will now be at home position and vice versa
    if (rightLegHome) {
      leftLegHome = true;
      rightLegHome = false;
    } else if (leftLegHome) {
      leftLegHome = false;
      rightLegHome = true;
    }
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
  // Verschiebe den Punkt (pX, pY) in ein Koordinatensystem mit dem Kreis im Ursprung
  int shiftedPX = pX - mX;
  int shiftedPY = pY - mY;

  // Richtung in Integer-Komponenten aufteilen mit weniger Skalierung
  int scaleFactor = 100;
  int dx = cos(direction) * scaleFactor + 0.5;
  int dy = sin(direction) * scaleFactor + 0.5;

  // Quadratische Koeffizienten
  int a = dx * dx + dy * dy;
  int b = 2 * (dx * shiftedPX + dy * shiftedPY);
  int c = shiftedPX * shiftedPX + shiftedPY * shiftedPY - radius * radius;

  int discriminant = b * b - 4 * a * c;

  if (discriminant < 0) {
    // Kein Schnittpunkt, leere Ergebnisse setzen (z.B., -1 als Kennzeichnung)
    intersections[0][0] = intersections[0][1] = -1;
    intersections[1][0] = intersections[1][1] = -1;
    return;
  }

  int sqrt_discriminant = sqrt(discriminant) + 0.5;

  // Erst teilen, dann skalieren für präzisere t-Werte
  float t1 = (-b + sqrt_discriminant) / (2.0 * a);
  float t2 = (-b - sqrt_discriminant) / (2.0 * a);

  // Berechne und verschiebe den ersten Schnittpunkt zurück
  intersections[0][0] = shiftedPX + t1 * dx + mX;
  intersections[0][1] = shiftedPY + t1 * dy + mY;

  // Berechne und verschiebe den zweiten Schnittpunkt zurück, falls vorhanden
  if (discriminant > 0) {
    intersections[1][0] = shiftedPX + t2 * dx + mX;
    intersections[1][1] = shiftedPY + t2 * dy + mY;
  } else {
    intersections[1][0] = intersections[1][1] = -1;  // Kein zweiter Schnittpunkt
  }
}


// Funktion, die den Schnittpunkt bestimmt, der weiter entgegengesetzt zur Richtung liegt
int Hexapod::getOppositeIntersection(int pX, int pY, float direction, int intersections[2][2]) {
  // Umgekehrter Richtungsvektor
  float oppositeDX = -cos(direction);
  float oppositeDY = -sin(direction);

  // Projektion des Vektors vom Punkt p zum ersten Schnittpunkt auf den umgekehrten Richtungsvektor
  float vector1X = intersections[0][0] - pX;
  float vector1Y = intersections[0][1] - pY;
  float projection1 = vector1X * oppositeDX + vector1Y * oppositeDY;

  // Projektion des Vektors vom Punkt p zum zweiten Schnittpunkt auf den umgekehrten Richtungsvektor
  float vector2X = intersections[1][0] - pX;
  float vector2Y = intersections[1][1] - pY;
  float projection2 = vector2X * oppositeDX + vector2Y * oppositeDY;

  // Wähle den Punkt mit der größeren Projektion in die entgegengesetzte Richtung
  return (projection1 > projection2) ? 0 : 1;
}
