#ifndef _AQ_KINEMATICS_
#define _AQ_KINEMATICS_

#include "AeroQuad.h"
#include "GlobalDefined.h"

#define CF 0
#define KF 1
#define DCM 2
#define ARG 3
#define MARG 4

extern float kinematicsAngle[3];
extern float gyroAngle[2];
extern float correctedRateVector[3];
extern float earthAccel[3];

extern float accelCutoff;

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

