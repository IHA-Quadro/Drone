#include "ControlFaker.h"

int _controllerInput[channelsInUse] = {1500, 1500, 1500, 1100, 0, 0, 0 ,0};
bool _initialized;
bool _calibrationPerformed;
bool _motorsArmed;
bool _safetyChecked;

int maxSpinSpeed = 2000;



//Setup file for ControlFaker
void SetupControlFaker()
{
	printNewLine("Init ControlFaker", STATUSMODE);
	SetupPrintDrone();

	_initialized = false;
	_motorsArmed = false;
	_safetyChecked = false;
	_calibrationPerformed = false;

	for(byte i = XAXIS; i < THROTTLE; i++)
	{
		_controllerInput[i] = MINCOMMAND+100;
	}

	KillMotor(false);
}

//Set motors throttle to 1100 making it not spinning and silence all Serial output
void KillMotor()
{
	KillMotor(true);

	SilenceSerial();

	//Reset axis
	for(byte axis = XAXIS; axis < THROTTLE; axis++)
	{
		_controllerInput[axis] = 1500;
	}

	//Thottle killed
	_controllerInput[THROTTLE] = MINCOMMAND+100;
}

//Take the values of the current program's values and apply them for output
void ApplyProgram()
{
	_controllerInput[XAXIS] = programInput.data.xAxis;
	_controllerInput[YAXIS] = programInput.data.yAxis;
	_controllerInput[ZAXIS] = programInput.data.zAxis;
	_controllerInput[AUX1] = programInput.data.aux1;
	_controllerInput[AUX3] = programInput.data.aux3;
}

//Arm motors so they will fly (safety mechanism)
void ArmMotors()
{
	printNewLine("Prep. Motors armed", STATUSMODE);
	_controllerInput[ZAXIS] = 2000;
	_motorsArmed = true;
}

//Set motors to "not-spinning"
void SafetyCheck()
{
	printNewLine("Prep. Safetycheck", STATUSMODE);
	_controllerInput[ZAXIS] = 1150;
	_safetyChecked = true;
}

//Active a calibration sequence
void PerformCalibration()
{
	printNewLine("Prep. Calibration performed", STATUSMODE);

	_controllerInput[ZAXIS] = 1000;
	_controllerInput[XAXIS] = 2000;
	_controllerInput[YAXIS] = 1000;
	_controllerInput[THROTTLE] = 1000;

	_calibrationPerformed = true;
}

//Apply speed check. Do not fly faster than 'maxSpinSpeed' and kill motor if needed
void ApplySpeed()
{
	//Specific for the program.
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
}

//Resets data for the input data.
void ResetFakerData()
{
	_controllerInput[XAXIS]			= 1500;
	_controllerInput[YAXIS]			= 1500;
	_controllerInput[ZAXIS]			= 1500;
	_controllerInput[THROTTLE]	= 1210; //1200 be minimum value
	_controllerInput[MODE]			= 2000; //Over 1500 gives Flight Mode = Altitude (FlightCommandProcessor.cpp)
	_controllerInput[AUX1]			= ALTITUDEHOLDFALSE; 
	_controllerInput[AUX2]			= 1200; //For GPS program to get "home" - keep low
	_controllerInput[AUX3]			= AUTOLANDFALSE; 

	_initialized = true;

	printNewLine("Drone intialized", STATUSMODE);
}

void PrintMotorOutput()
{
	if(!IsMotorKilled())
	{
		//printInLine("Motor output: ", MOTORMODE);
		//printInLine(motorCommand[0], MOTORMODE);
		//printInLine(", ", MOTORMODE);
		//printInLine(motorCommand[1], MOTORMODE);
		//printInLine(", ", MOTORMODE);
		//printInLine(motorCommand[2], MOTORMODE);
		//printInLine(", ", MOTORMODE);
		//printInLine(motorCommand[3], MOTORMODE);
		//printInLine(", ", MOTORMODE);
		//printInLine(_controllerInput[THROTTLE], MOTORMODE);
		//println(MOTORMODE);
	}
}
