
#ifndef _CONTROLFAKER_H_
#define _CONTROLFAKER_H_

#include "Arduino.h"
#include "GlobalDefined.h"

#define channelsInUse 6
int _hzCounter = 1000;
int _maxHzCounter = 1250;
int _commonHeading = 1550;
bool _startUp = true;
bool _calibrationPerformed = false;
bool _motorsArmed = false;
bool _safetyChecked = false;

int _controllerInput[channelsInUse] = {0,0,0,0,0,0};

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
}

void ArmMotors()
{
	Serial.println("Motors armed", SERIALPRINT);
	_controllerInput[ZAXIS] = 2000;
	_startUp = false;
}

void SafetyCheck()
{
	Serial.println("Safetycheck", SERIALPRINT);
	_controllerInput[ZAXIS] = 1150;
	_safetyChecked = true;
}

void PerformCalibration()
{
	Serial.println("Performing calibration", SERIALPRINT);
	_controllerInput[ZAXIS] = 1000;
	_controllerInput[XAXIS] = 2000;
	_controllerInput[YAXIS] = 1000;
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
}

//Not in use
void AssignTestData()
{
	_controllerInput[XAXIS]			= 1500;
	_controllerInput[YAXIS]			= 1500;
	_controllerInput[ZAXIS]			= 1500;
	_controllerInput[THROTTLE]	= 1210;
	_controllerInput[MODE]			= 1000;
	_controllerInput[AUX1]			= 1000;
	_controllerInput[AUX2]			= 1000;
	_controllerInput[AUX3]			= 1000;
	_controllerInput[AUX4]			= 1000;
	_controllerInput[AUX5]			= 1000;
}

#endif