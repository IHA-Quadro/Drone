#include "Kinematics.h"

// This class is responsible for calculating vehicle attitude
byte kinematicsType = 0;
float kinematicsAngle[3] = {0.0,0.0,0.0};
float gyroAngle[2] = {0.0,0.0};
float correctedRateVector[3] = {0.0,0.0,0.0};
float earthAccel[3] = {0.0,0.0,0.0};

float accelCutoff = 0.0;

void initializeBaseKinematicsParam() {

  for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
    kinematicsAngle[axis] = 0.0;
  }
  gyroAngle[XAXIS] = 0;
  gyroAngle[YAXIS] = 0;
}

const float kinematicsGetDegreesHeading(byte axis) 
{
  float tDegrees;
    
  tDegrees = degrees(kinematicsAngle[axis]);
  if (tDegrees < 0.0)
    return (tDegrees + 360.0);
  else
    return (tDegrees);
}