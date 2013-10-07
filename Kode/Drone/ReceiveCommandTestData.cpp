#include "ReceiveCommandTestData.h"

int _stepSize = 5;
bool _inverseSteps = false;
int _starterSpeed = 1200;
int throttleSpeed = _starterSpeed;
int miliSecCounter = 0;
int miliSecCounterStamp = 0;
bool miliSecCounterActive = false;
ProgramInput programInput = {0,0,0,0};
ProgramInput _previousProgram = {0,0,0,0};

void ResetReceiveCommandTestData()
{
	_inverseSteps = false;
	_previousProgram.ProgramID = 0;
	miliSecCounterActive = false;
	inititalizeRangeFinders();
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

int MeasureSonar(byte sonarId)
{
	updateRangeFinders();
	return (int)((float)rangeFinderRange[sonarId] *100); 
}

void PrintSonarData(byte sonarID)
{
	printNewLine(MeasureSonar(sonarID), STATUSMODE);
}

void KeepRunningProgram()
{
	RunProgram(_previousProgram);
}

static void KeepHeightBySonar(int steadyHeight, int initTime)
{
	int sonarHeight = MeasureSonar(ALTITUDE_RANGE_FINDER_INDEX); //Bottom sonar

	if( (sonarHeight + STEADYTOLERANCE > steadyHeight) && (sonarHeight - STEADYTOLERANCE < steadyHeight))
	{
		//All green - keep level
	}

	else if(sonarHeight + STEADYTOLERANCE < steadyHeight) //Not high enough
	{
		if(miliSecCounter > initTime) //Still not high enough after initTime
		{
			miliSecCounter = 0;
			spinSpeed += 5;
			printInLine("SpinSpeed = ", STATUSMODE);
			printInLine(spinSpeed, STATUSMODE);
			printInLine(" ; ", STATUSMODE);
			printInLine(sonarHeight, STATUSMODE);
			println(STATUSMODE);
		}
	}
	else if(sonarHeight  - STEADYTOLERANCE > steadyHeight) //Too high 
	{
		if(miliSecCounter > initTime) //Still too high after initTime
		{
			miliSecCounter = 0;
			spinSpeed -= 5;
			printInLine("SpinSpeed = ", STATUSMODE);
			printInLine(spinSpeed, STATUSMODE);
			printInLine(" ; ", STATUSMODE);
			printInLine(sonarHeight, STATUSMODE);
			println(STATUSMODE);
		}
	}
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
		AccelerateSpeed(input.data, input.TimeSpanInMiliSec);
		break;

	case 3:
		PrintSonarData(input.data);
		break;

	case 4:
		KeepHeightBySonar(input.data, input.TimeSpanInMiliSec);
		break;

	default:
		miliSecCounter = 0;
		break;
	}
}
