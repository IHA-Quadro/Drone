// ReceiveCommandTestData.h

#ifndef _RECEIVECOMMANDTESTDATA_h
#define _RECEIVECOMMANDTESTDATA_h

#include "Arduino.h"
#include "ControlFaker.h"

#define lower 1450
#define upper 1550
#define steady 1500
int _stepSize = 5;
bool _inverseSteps = false;

void TestAxis(byte axis)
{
	int step = _stepSize;
	_controllerInput[THROTTLE] = 1250;

	_controllerInput[axis] += step;

	//Inverse step on lower an upper
	_inverseSteps = (_controllerInput[axis] < lower || _controllerInput[axis] > upper ? true : false);

	if(_inverseSteps == true)
		_stepSize = _stepSize*(-1);

	KillMotor();
}

void RotateDrone(int zaxis)
{
	_controllerInput[ZAXIS] = zaxis;
}

void MoveDrone(int xaxis, int yaxis)
{
	_controllerInput[XAXIS] = xaxis;
	_controllerInput[YAXIS] = yaxis;
}

void ThrottleDrone(int throttle)
{
	_controllerInput[THROTTLE] = throttle;
}


#endif

