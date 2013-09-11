#ifndef _AQ_KINEMATICS_
#define _AQ_KINEMATICS_

#include "GlobalDefined.h"

#define CF 0
#define KF 1
#define DCM 2
#define ARG 3
#define MARG 4

void ResetKinematicData();
void initializeBaseKinematicsParam();
void initializeKinematics(float hdgX, float hdgY);
void calculateKinematics(float rollRate, float pitchRate, float yawRate, float longitudinalAccel, float lateralAccel, float verticalAccel, float G_Dt);
float getGyroUnbias(byte axis);
void calibrateKinematics();


// This really needs to be in Radians to be consistent
// I'll fix later - AKA
// returns heading in degrees as 0-360
const float kinematicsGetDegreesHeading(byte axis);

#endif

