#include "ReceiveCommandTestData.h"

int _stepSize = 5;
bool _inverseSteps = false;
int _starterSpeed = 1350;
int _deciSeconds = 0;
int throttleSpeed = _starterSpeed;

void ResetReceiveCommandTestData()
{
	_deciSeconds = 0;
	_inverseSteps = false;
}

void TestAxis(byte axis)
{
	int step = _stepSize;
	_controllerInput[THROTTLE] = 1250;

	_controllerInput[axis] += step;

	//Inverse step on lower an upper
	_inverseSteps = (_controllerInput[axis] < lower || _controllerInput[axis] > upper ? true : false);

	if(_inverseSteps == true)
		_stepSize = _stepSize*(-1);

	IsMotorKilled();
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

//void ThrottleDrone(int throttle)
//{
//	_controllerInput[THROTTLE] = throttle;
//}

void AccelerateSpeed()
{
	if(_deciSeconds < 40) //Wait for 4 seconds before real speed is applied
	{
		if(spinSpeed < _starterSpeed)
			spinSpeed += 5;

		_deciSeconds += 1; //10 pr sec.
	}
	else
	{
		if(spinSpeed >= throttleSpeed)
			spinSpeed = throttleSpeed;

		else
			spinSpeed +=5;
	}
}

void RunProgram(int program)
{
	switch (program)
	{
	case 1:
		AccelerateSpeed();
		break;

	default:
		break;
	}
}
