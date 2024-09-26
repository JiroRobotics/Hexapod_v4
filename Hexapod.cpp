#include "SpeedTrig.h"
// Hexapod.cpp

#include "Hexapod.h"

bool Hexapod::bodyMovement(int16_t xTrans, int16_t yTrans, int16_t zTrans, float roll, float pitch, float yaw) {
  /*
   * moves and rotates the body of the hexapod by the specified amount, while all legs stay stationary on the ground.
   * xTrans:      distance (in mm) to move the center of the robot in x direction (global coordinates) (back/forth)
   * yTrans:      distance (in mm) to move the center of the robot in y direction (global coordinates) (sideways)
   * zTrans:      distance (in mm) to move the center of the robot in z direction (global coordinates) (up/down)
   * roll:        angle (in rad) to rotate around the x-axis
   * pitch:       angle (in rad) to rotate around the y-axis
   * yaw:         angle (in rad) to rotate around the z-axis
   *
   * returns:     true if all servos could reach their position, false otherwise
   */

  // code...
  // initialize an array with all local coordinates for all legs
  // +++++++++++++++++++++++++++++++++++++++++++++++ TODO: change to current leg position!!! +++++++++++++++++++++++++++++++++++++++++++++++
  float newPositions[6][3] = { { homePos[0], homePos[1], homePos[2] },    // front right
                               { homePos[0], homePos[1], homePos[2] },    // front left
                               { homePos[0], homePos[1], homePos[2] },    // mid right
                               { homePos[0], homePos[1], homePos[2] },    // mid left
                               { homePos[0], homePos[1], homePos[2] },    // rear right
                               { homePos[0], homePos[1], homePos[2] } };  // rear left

  // ----------------------------------------------------------------------------
  // calculate the new position due to the translational portion of the movement
  // ----------------------------------------------------------------------------
  xTrans *= -1;  // multiply by -1 so that we have a right hand coordinate system
  // middle legs are easy... just add/subtract xTrans and yTrans as they are not rotated relative to the centerline
  newPositions[2][0] += xTrans;
  newPositions[2][1] += yTrans;
  newPositions[3][0] -= xTrans;
  newPositions[3][1] -= yTrans;

  // corner legs are rotated by 30° (PI/6) relative to the centerline. +/- depending on the side and orientation
  // calculation using a rotation matrix
  float zRotMatrix[2][2] = { { cos(cornerLegAngle), -sin(cornerLegAngle) },
                             { sin(cornerLegAngle), cos(cornerLegAngle) } };
  newPositions[0][0] += zRotMatrix[0][0] * xTrans + zRotMatrix[0][1] * yTrans;
  newPositions[0][1] += zRotMatrix[1][0] * xTrans + zRotMatrix[1][1] * yTrans;

  newPositions[1][0] += zRotMatrix[0][0] * -xTrans + zRotMatrix[0][1] * yTrans;
  newPositions[1][1] += zRotMatrix[1][0] * xTrans + zRotMatrix[1][1] * -yTrans;

  newPositions[4][0] += zRotMatrix[0][0] * xTrans + zRotMatrix[0][1] * -yTrans;
  newPositions[4][1] += zRotMatrix[1][0] * -xTrans + zRotMatrix[1][1] * yTrans;

  newPositions[5][0] += zRotMatrix[0][0] * -xTrans + zRotMatrix[0][1] * -yTrans;
  newPositions[5][1] += zRotMatrix[1][0] * -xTrans + zRotMatrix[1][1] * -yTrans;

  // z-coordinate (for translation) is the same for all legs. Difference between previous height and zTrans is used.
  for (int i = 0; i < 6; ++i) {
    newPositions[i][2] = zHeight + zTrans;
  }

  // ----------------------------------------------------------------------------
  // calculate the new position due to the rotation around the z axis
  // ----------------------------------------------------------------------------
  // find the distance from each leg end point to the center of the robot using pythagoras
  // float distLegCenterFR = sqrt((newPositions[0][0] + cornerLegXDist) * (newPositions[0][0] + cornerLegXDist) + (newPositions[0][1] + cornerLegYDist) * (newPositions[0][1] + cornerLegYDist));
  // calculate the distance between the current end point and the endpoint after rotation using the law of cosine with a = b: (c^2 = a^2 + b^2 - 2ab*cos(yaw) === c^2 = 2*a^2 (1-cos(yaw)) )
  // float rotationDistance = sqrt(2 * distLegCenterFR * distLegCenterFR * (1 - cos(yaw)));

  // shift the plane so that the center of the robot is in the center of the local coordinate system.
  // then rotate around the center and shift back
  float yawRotMatrix[2][2] = { { cos(yaw), -sin(yaw) },
                               { sin(yaw), cos(yaw) } };

  // front right leg
  //shift the center of mass to local coordinate center
  newPositions[0][0] += cornerLegXDist;
  newPositions[0][1] += cornerLegYDist;

  float temp = newPositions[0][0];
  // rotate using the rotation matrix (temp because newPosition[0][0] is overwritten)
  newPositions[0][0] = newPositions[0][0] * yawRotMatrix[0][0] + newPositions[0][1] * yawRotMatrix[0][1];
  newPositions[0][1] = temp * yawRotMatrix[1][0] + newPositions[0][1] * yawRotMatrix[1][1];

  // shift the plane back
  newPositions[0][0] -= cornerLegXDist;
  newPositions[0][1] -= cornerLegYDist;
  // some debug info
  Serial.print("At yaw-value ");
  Serial.println(yaw);
  Serial.print("x-Value: ");
  Serial.print(newPositions[0][0]);
  Serial.print("\t y-Value: ");
  Serial.println(newPositions[0][1]);
  Serial.print("\n");

  // repeat the same for every leg
  // front left leg
  newPositions[1][0] -= cornerLegXDist;
  newPositions[1][1] += cornerLegYDist;

  temp = newPositions[1][0];
  newPositions[1][0] = newPositions[1][0] * yawRotMatrix[0][0] + newPositions[1][1] * yawRotMatrix[0][1];
  newPositions[1][1] = temp * yawRotMatrix[1][0] + newPositions[1][1] * yawRotMatrix[1][1];

  newPositions[1][0] += cornerLegXDist;
  newPositions[1][1] -= cornerLegYDist;

  // mid right leg
  newPositions[2][1] += centerLegYDist;

  temp = newPositions[2][0];
  newPositions[2][0] = newPositions[2][0] * yawRotMatrix[0][0] + newPositions[2][1] * yawRotMatrix[0][1];
  newPositions[2][1] = temp * yawRotMatrix[1][0] + newPositions[2][1] * yawRotMatrix[1][1];

  newPositions[2][1] -= centerLegYDist;

  // mid left leg
  newPositions[3][1] += centerLegYDist;

  temp = newPositions[3][0];
  newPositions[3][0] = newPositions[3][0] * yawRotMatrix[0][0] + newPositions[3][1] * yawRotMatrix[0][1];
  newPositions[3][1] = temp * yawRotMatrix[1][0] + newPositions[3][1] * yawRotMatrix[1][1];

  newPositions[3][1] -= centerLegYDist;

  // rear right leg
  newPositions[4][0] -= cornerLegXDist;
  newPositions[4][1] += cornerLegYDist;

  temp = newPositions[4][0];
  newPositions[4][0] = newPositions[4][0] * yawRotMatrix[0][0] + newPositions[4][1] * yawRotMatrix[0][1];
  newPositions[4][1] = temp * yawRotMatrix[1][0] + newPositions[4][1] * yawRotMatrix[1][1];

  newPositions[4][0] += cornerLegXDist;
  newPositions[4][1] -= cornerLegYDist;

  // rear left leg
  newPositions[5][0] += cornerLegXDist;
  newPositions[5][1] += cornerLegYDist;

  temp = newPositions[5][0];
  newPositions[5][0] = newPositions[5][0] * yawRotMatrix[0][0] + newPositions[5][1] * yawRotMatrix[0][1];
  newPositions[5][1] = temp * yawRotMatrix[1][0] + newPositions[5][1] * yawRotMatrix[1][1];

  newPositions[5][0] -= cornerLegXDist;
  newPositions[5][1] -= cornerLegYDist;

  // ----------------------------------------------------------------------------
  // calculate the new position due to roll and pitch
  // ----------------------------------------------------------------------------
  /*
  // front right leg
  // total y distance to leg end point in global coordinates
  float totalYDist = cornerLegYDistGlobal + sin(PI / 6 + atan(newPositions[0][0] / newPositions[0][1])) * sqrt(newPositions[0][0] * newPositions[0][0] + newPositions[0][1] * newPositions[0][1]);
  float totalXDist = cornerLegXDistGlobal + cos(PI / 6 + atan(newPositions[0][0] / newPositions[0][1])) * sqrt(newPositions[0][0] * newPositions[0][0] + newPositions[0][1] * newPositions[0][1]);
  // adjust the z coordinate based on the distance to the center and roll/pitch angle
  newPositions[0][2] += tan(pitch) * totalXDist + tan(roll) * totalYDist;

  // front left leg
  totalYDist = cornerLegYDistGlobal + sin(PI / 6 - atan(newPositions[1][0] / newPositions[1][1])) * sqrt(newPositions[1][0] * newPositions[1][0] + newPositions[1][1] * newPositions[1][1]);
  totalXDist = cornerLegXDistGlobal + cos(PI / 6 - atan(newPositions[1][0] / newPositions[1][1])) * sqrt(newPositions[1][0] * newPositions[1][0] + newPositions[1][1] * newPositions[1][1]);
  // adjust the z coordinate based on the distance to the center and roll/pitch angle
  newPositions[1][2] += tan(pitch) * totalXDist + tan(-roll) * totalYDist;

  // middle right leg
  totalYDist = centerLegYDist + newPositions[2][1];
  totalXDist = newPositions[2][0];
  newPositions[2][2] += tan(pitch) * totalXDist + tan(roll) * totalYDist;

  //middle left leg
  totalYDist = centerLegYDist + newPositions[3][1];
  totalXDist = -newPositions[3][0];
  newPositions[3][2] += tan(pitch) * totalXDist + tan(-roll) * totalYDist;

  // rear right leg
  totalYDist = cornerLegYDistGlobal + sin(PI / 6 - atan(newPositions[4][0] / newPositions[4][1])) * sqrt(newPositions[4][0] * newPositions[4][0] + newPositions[4][1] * newPositions[4][1]);
  totalXDist = cornerLegXDistGlobal + cos(PI / 6 - atan(newPositions[4][0] / newPositions[4][1])) * sqrt(newPositions[4][0] * newPositions[4][0] + newPositions[4][1] * newPositions[4][1]);
  // adjust the z coordinate based on the distance to the center and roll/pitch angle
  newPositions[4][2] += tan(-pitch) * totalXDist + tan(roll) * totalYDist;

  // rear left leg
  totalYDist = cornerLegYDistGlobal + sin(PI / 6 + atan(newPositions[5][0] / newPositions[5][1])) * sqrt(newPositions[5][0] * newPositions[5][0] + newPositions[5][1] * newPositions[5][1]);
  totalXDist = cornerLegXDistGlobal + cos(PI / 6 + atan(newPositions[5][0] / newPositions[5][1])) * sqrt(newPositions[5][0] * newPositions[5][0] + newPositions[5][1] * newPositions[5][1]);
  // adjust the z coordinate based on the distance to the center and roll/pitch angle
  newPositions[5][2] += tan(-pitch) * totalXDist + tan(-roll) * totalYDist;*/

  // rotation for the middle legs


  // move all legs to the new position and return true if every servo could reach the target
  bool flag = true;
  if (legFR.moveTo(newPositions[0][0], newPositions[0][1], newPositions[0][2]) && legFL.moveTo(newPositions[1][0], newPositions[1][1], newPositions[1][2])
      && legMR.moveTo(newPositions[2][0], newPositions[2][1], newPositions[2][2]) && legML.moveTo(newPositions[3][0], newPositions[3][1], newPositions[3][2])
      && legRR.moveTo(newPositions[4][0], newPositions[4][1], newPositions[4][2]) && legRL.moveTo(newPositions[5][0], newPositions[5][1], newPositions[5][2])) {
    return true;
  } else {
    return false;
  }
}

bool Hexapod::moveHome() {
  /* 
   * moves all legs instantly to the home position.
   * returns true if successful
   */
  bool flag = true;
  if (legFR.moveTo(homePos[0], homePos[1], homePos[2]) && legFL.moveTo(homePos[0], homePos[1], homePos[2])
      && legMR.moveTo(homePos[0], homePos[1], homePos[2]) && legML.moveTo(homePos[0], homePos[1], homePos[2])
      && legRR.moveTo(homePos[0], homePos[1], homePos[2]) && legRL.moveTo(homePos[0], homePos[1], homePos[2])) {
    return true;
  } else {
    return false;
  }
}



bool Hexapod::moveBody(int16_t xTrans, int16_t yTrans, int16_t zTrans, float roll, float pitch, float yaw) {
  /*
   * moves and rotates the body of the hexapod by the specified amount, while all legs stay stationary on the ground.
   * xTrans:      distance (in mm) to move the center of the robot in x direction (global coordinates) (back/forth)
   * yTrans:      distance (in mm) to move the center of the robot in y direction (global coordinates) (sideways)
   * zTrans:      distance (in mm) to move the center of the robot in z direction (global coordinates) (up/down)
   * roll:        angle (in rad) to rotate around the x-axis
   * pitch:       angle (in rad) to rotate around the y-axis
   * yaw:         angle (in rad) to rotate around the z-axis
   *
   * returns:     true if all servos could reach their position, false otherwise
   *
   * runtime on Arduino Nano 33 IoT: <5.65ms (default), <8.2ms (useFloat)
   * runtime on Arduino Nano 33 BLE Sense: <3.85ms (default), <3.95ms (useFloat) (<- this is what a FPU is made for)
   */

  // code...
  // initialize an array with all local coordinates for all legs
  // +++++++++++++++++++++++++++++++++++++++++++++++ TODO: change to current leg position!!! +++++++++++++++++++++++++++++++++++++++++++++++
  float newPositions[6][3] = { { homePos[0], homePos[1], homePos[2] },    // front right
                               { homePos[0], homePos[1], homePos[2] },    // front left
                               { homePos[0], homePos[1], homePos[2] },    // mid right
                               { homePos[0], homePos[1], homePos[2] },    // mid left
                               { homePos[0], homePos[1], homePos[2] },    // rear right
                               { homePos[0], homePos[1], homePos[2] } };  // rear left

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
  float temp = newPositions[0][0];
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

  // move all legs to the new position and return true if every servo could reach the target
  bool flag = true;
  if (legFR.moveTo(newPositions[0][0], newPositions[0][1], newPositions[0][2]) && legFL.moveTo(newPositions[1][0], newPositions[1][1], newPositions[1][2])
      && legMR.moveTo(newPositions[2][0], newPositions[2][1], newPositions[2][2]) && legML.moveTo(newPositions[3][0], newPositions[3][1], newPositions[3][2])
      && legRR.moveTo(newPositions[4][0], newPositions[4][1], newPositions[4][2]) && legRL.moveTo(newPositions[5][0], newPositions[5][1], newPositions[5][2])) {
    return true;
  } else {
    return false;
  }
}
