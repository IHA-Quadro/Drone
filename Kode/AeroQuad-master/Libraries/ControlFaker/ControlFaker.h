
#ifndef _CONTROLFAKER_H_
#define _CONTROLFAKER_H_

#include "Arduino.h"
#include "GlobalDefined.h"

#define channelsInUse 6
int _hzCounter = 1000;
int _maxHzCounter = 1200;
int _commonHeading = 1500;

bool _initialized = false;
bool _calibrationPerformed = false;
bool _motorsArmed = false;
bool _safetyChecked = false;

bool _killMotors = false;

int _controllerInput[channelsInUse] = {0,0,0,0,0,0};

void KillMotor()
{
	if(_killMotors)
	{
		for(byte axis = XAXIS; axis < AUX5 + 1; axis++)
		{
			_controllerInput[axis] = 0;
		}
	}
}

void FMSignal()
{

}

void SelectProgram()
{

}

void SonarCheck()
{

}

void CalculateAltitude()
{

}

void ApplyHeading()
{
	_controllerInput[XAXIS] = _commonHeading;	
	_controllerInput[YAXIS] = _commonHeading;
	_controllerInput[ZAXIS] = _commonHeading;

	KillMotor();
}

void ArmMotors()
{
	_controllerInput[ZAXIS] = 2000;
	_motorsArmed = true;
}

void SafetyCheck()
{
	_controllerInput[ZAXIS] = 1150;
	_safetyChecked = true;
}

void PerformCalibration()
{
	_controllerInput[ZAXIS] = 1000;
	_controllerInput[XAXIS] = 2000;
	_controllerInput[YAXIS] = 1000;
	_controllerInput[THROTTLE] = 1000;

	_calibrationPerformed = true;
}

void ApplySpeed()
{
	if(_hzCounter > _maxHzCounter)
		_hzCounter = _maxHzCounter;

	//Speed increased 1 in 10Hz
	_controllerInput[THROTTLE] = _hzCounter;

	//Still not sure what MODE does..
	//See FlightCommandProcessor::readPilotCommands()
	_controllerInput[MODE] = _hzCounter;

	KillMotor();
}

void ResetInputData()
{
	_controllerInput[XAXIS]			= 1500;
	_controllerInput[YAXIS]			= 1500;
	_controllerInput[ZAXIS]			= 1500;
	_controllerInput[THROTTLE]	= 1000;
	_controllerInput[MODE]			= 1000;
	_controllerInput[AUX1]			= 1000;
	_controllerInput[AUX2]			= 1000;
	_controllerInput[AUX3]			= 1000;
	_controllerInput[AUX4]			= 1000;
	_controllerInput[AUX5]			= 1000;
}

#endif