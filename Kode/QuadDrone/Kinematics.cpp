#include "Kinematics.h"
// This class is responsible for calculating vehicle attitude

float kinematicsAngle[3];
float gyroAngle[2];
float correctedRateVector[3];
float earthAccel[3];

float accelCutoff;


void initializeBaseKinematicsParam() 
{
	byte i;
	byte axis;

	for(i = XAXIS ; i < 3; i++)
	{
		kinematicsAngle[i] = 0;
		correctedRateVector[i] = 0;
		earthAccel[i] = 0;
	}

	for(i = XAXIS ; i< 2; i++)
	{
		gyroAngle[i] = 0;
	}

	accelCutoff = 0.0;

	for (axis = XAXIS; axis <= ZAXIS; axis++) 
	{
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