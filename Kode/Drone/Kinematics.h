#ifndef _AQ_KINEMATICS_
#define _AQ_KINEMATICS_

#include <Arduino.h>
#include "GlobalDefined.h"

#define CF 0
#define KF 1
#define DCM 2
#define ARG 3
#define MARG 4

// This class is responsible for calculating vehicle attitude
extern byte kinematicsType;
extern float kinematicsAngle[3];
extern float gyroAngle[2];
extern float correctedRateVector[3];
extern float earthAccel[3];
extern float accelCutoff;

void initializeBaseKinematicsParam();

void initializeKinematics(float hdgX, float hdgY);
void calculateKinematics(float rollRate, float pitchRate, float yawRate, float longitudinalAccel, float lateralAccel, float verticalAccel, float G_Dt);
float getGyroUnbias(byte axis);
void calibrateKinematics();
 
  // returns the kinematicsAngle of a specific axis in SI units (radians)
//  const float getData(byte axis) {
//    return kinematicsAngle[axis];
//  }
  // return heading as +PI/-PI
//  const float getHeading(byte axis) {
//    return(kinematicsAngle[axis]);
//  }
  
  // This really needs to be in Radians to be consistent
  // I'll fix later - AKA
  // returns heading in degrees as 0-360
const float kinematicsGetDegreesHeading(byte axis);
  
//  const byte getType(void) {
    // This is set in each subclass to identify which algorithm used
//    return kinematicsType;
//  }


#endif

