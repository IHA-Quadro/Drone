#include "ControlFaker.h"

int _controllerInput[channelsInUse];
bool _initialized;
bool _calibrationPerformed;
bool _motorsArmed;
bool _safetyChecked;

int maxSpinSpeed = 2000;
int spinSpeed = 1200;

void SetupControlFaker()
{
	printNewLine("Init ControlFaker", STATUSMODE);
	SetupPrintDrone();

	_initialized = false;
	_calibrationPerformed = false;
	_motorsArmed = false;
	_safetyChecked = false;
	spinSpeed = 1200;

	for(byte i = 0; i < channelsInUse ; i++)
	{
		_controllerInput[0] = 0;
	}
	KillMotor(false);
}

void KillMotor()
{
	KillMotor(true);

	SilenceSerial();

	//Reset axis
	byte axis;
	for(axis = XAXIS; axis < THROTTLE; axis++)
	{
		_controllerInput[axis] = 1500;
	}

	//Thottle killed
	_controllerInput[THROTTLE] = MINCOMMAND+100;
}

void SerialOutput(bool active)
{

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
}

void ArmMotors()
{
	printNewLine("Prep. Motors armed", STATUSMODE);
	_controllerInput[ZAXIS] = 2000;
	_motorsArmed = true;
}

void SafetyCheck()
{
	printNewLine("Prep. Safetycheck", STATUSMODE);
	_controllerInput[ZAXIS] = 1150;
	_safetyChecked = true;
}

void PerformCalibration()
{
	printNewLine("Prep. Calibration performed", STATUSMODE);

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
	if(IsMotorKilled())
	{
		for(byte i = 0 ; i < THROTTLE; i++)
		{
			_controllerInput[i] = 1500;
		}
		_controllerInput[THROTTLE] = 1000;
	}
	else
	{
		for(int i = 0; i < 4; i++)
		{
			if(motorCommand[i] > maxSpinSpeed) // Spinning too fast
			{
				PrintConfig[STATUSMODE] = true;
				printInLine("-->", STATUSMODE);
				PrintMotorOutput();
				KillMotor();
			}
		}
	}
	_controllerInput[THROTTLE] = spinSpeed;
}

void ResetInputData()
{
	_controllerInput[XAXIS]			= 1500;
	_controllerInput[YAXIS]			= 1500;
	_controllerInput[ZAXIS]			= 1500;
	_controllerInput[THROTTLE]	= 1100;
	_controllerInput[MODE]			= 1000;
	_controllerInput[AUX1]			= 1000;

	_initialized = true;

	printNewLine("Drone intialized", STATUSMODE);

	//ResetAccelData();
	//ResetKinematicData();
	//ResetGyroData();
	//ResetHeadingData();
}

void PrintMotorOutput()
{
	if(!IsMotorKilled())
	{
		printInLine("Motor output: ", MOTORMODE);
		printInLine(motorCommand[0], MOTORMODE);
		printInLine(", ", MOTORMODE);
		printInLine(motorCommand[1], MOTORMODE);
		printInLine(", ", MOTORMODE);
		printInLine(motorCommand[2], MOTORMODE);
		printInLine(", ", MOTORMODE);
		printInLine(motorCommand[3], MOTORMODE);
		printInLine(", ", MOTORMODE);
		printInLine(_controllerInput[THROTTLE], MOTORMODE);
		println(MOTORMODE);
	}
}
