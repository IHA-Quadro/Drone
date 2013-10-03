#include "ReceiveCommandTestData.h"

int _stepSize = 5;
bool _inverseSteps = false;
int _starterSpeed = 1200;
int throttleSpeed = _starterSpeed;
int miliSecCounter = 0;
bool miliSecCounterActive = false;
ProgramInput programInput = {0,0,0,0};
ProgramInput _previousProgram = {0,0,0,0};

void ResetReceiveCommandTestData()
{
	_inverseSteps = false;
	_previousProgram.ProgramID = 0;
	miliSecCounterActive = false;
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

static void MiliSecOverflow(int timeSpanInMiliSec)
{
	if(miliSecCounter > timeSpanInMiliSec+10)
	{
		miliSecCounterActive =false;
	}
}

//void ThrottleDrone(int throttle)
//{
//	_controllerInput[THROTTLE] = throttle;
//}

void AccelerateSpeed(int maxSpeed, int initTime)
{
	if(miliSecCounter < initTime) //Wait before real speed is applied
	{
		if(spinSpeed < 1250)
			spinSpeed += 5;
	}
	else
	{
		if(spinSpeed >= maxSpeed)
			spinSpeed = maxSpeed;
		else
			spinSpeed += 10;

		MiliSecOverflow(initTime);
	}
}

void KeepRunningProgram()
{
	RunProgram(_previousProgram);
}

void RunProgram(ProgramInput input)
{
	miliSecCounterActive = true;

	if(input.ProgramID != _previousProgram.ProgramID)
	{
		miliSecCounter = 0;
		_previousProgram = input;
	}

	switch (input.ProgramID)
	{
	case 1:
		AeroQuadSetup();
		break;

	case 2:
		AccelerateSpeed(input.MaxSpeed, input.TimeSpanInMiliSec);
		break;

	default:
		miliSecCounter = 0;
		break;
	}
}
