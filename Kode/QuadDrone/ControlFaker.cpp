#include "ControlFaker.h"

int _controllerInput[channelsInUse];
boolean _initialized;
boolean _calibrationPerformed;
boolean _motorsArmed;
boolean _safetyChecked;

void DronePrint(String s)
{
	printNewLine(s, DEBUGMODE);
}

void SetupControlFaker()
{
	printNewLine("Init ControlFaker", STATUSMODE);

	_initialized = false;
	_calibrationPerformed = false;
	_motorsArmed = false;
	_safetyChecked = false;

	
	for(byte i = 0; i < channelsInUse ; i++)
	{
		_controllerInput[0] = 0;
	}
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
	//Code from Rasmus1
}

void SelectProgram()
{
	//Run test program or a specific pattern program
}

void SonarCheck()
{
	//Needs implementation
}

void CalculateAltitude()
{
	//A function may already exist for this
}

void ApplyHeading()
{
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
	//Specific for the program.
	//Input is missing but not known what format it will be
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

	//ResetAccelData();
	//ResetKinematicData();
	//ResetGyroData();
	//ResetHeadingData();
}
