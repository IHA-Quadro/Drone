#include "ControlFaker.h"
#include "stdbool.h"

bool _initialized;
bool _calibrationPerformed;
bool _motorsArmed;
bool _safetyChecked;

int _controllerInput[channelsInUse];

int _hzCounter;
int _maxHzCounter;
int _commonHeading;

void SetupControlFaker()
{
	_initialized = false;
	_calibrationPerformed = true;
	_motorsArmed = false;
	_safetyChecked = false;

	int i;
	for(i = 0; channelsInUse ; i++)
	{
		_controllerInput[channelsInUse] = 0;
	}

	_hzCounter = 1000;
	_maxHzCounter = 1200;
	_commonHeading = 1500;
}

void KillMotor()
{
	if(getMotorStatus())
	{
		//Reset axis
		byte axis;
		for(axis = XAXIS; axis < THROTTLE; axis++)
		{
			_controllerInput[axis] = 1500;
		}

		//Thottle killed
		_controllerInput[THROTTLE] = 1000;
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
	_controllerInput[THROTTLE]	= 1200;
	_controllerInput[MODE]			= 1000;
	_controllerInput[AUX1]			= 1000;
	_controllerInput[AUX2]			= 1000;
	_controllerInput[AUX3]			= 1000;
	_controllerInput[AUX4]			= 1000;
	_controllerInput[AUX5]			= 1000;

	ResetAccelData();
	ResetKinematicData();
	ResetGyroData();
	//ResetHeadingData();
}
