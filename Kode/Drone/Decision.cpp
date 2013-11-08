#include "Decision.h"

ProgramInput _serialProgram;
bool _autoLandMessage;
bool _startMessage;
bool StartTakeOff;

static void ResetMessages()
{
	_autoLandMessage = false;
	_startMessage = false;
	StartTakeOff = false;
}

ProgramInput GetActualProgram()
{
	return programInput;
}

void SerialComRequest(ProgramInput input)
{
	_serialProgram = input;
}

//Resets InputAnalysis
void ResetDecisions()
{
	ResetInputAnalysis();
}

//Choose program based on sonars
static void SonarCheck()
{
	AnalyseSonarInput();

	if(GetLeftWarning()) //Left warning
	{
		if(GetRightWarning())
		{
			if(GetFrontWarning()) //Left + Right + front
			{
				//Stop, rotate CW and run again
				KeepSteady(); //Stop
				RotateRightSlow();
			}
			else //Left + Right
			{
				//Stop, rotate 90 CW and run again			
				KeepSteady();
				RotateRightSlow();
			}
		}
		else
		{
			if(GetFrontWarning()) //Left + front
			{
				//Stop, rotate 60 CW and run again
				KeepSteady();
				RotateRightSlow();
			}
			else //Only Left
			{
				//Rotate 45 CW 
				ForwardSlow();
				RotateLeftSlow();
			}
		}
	}
	else //No left warning
	{
		if(GetRightWarning())
		{
			if(GetFrontWarning()) //Right+front
			{
				//Stop, rotate 60 CCW and run again
				KeepSteady();
				RotateLeftSlow();
			}
			else //Only right
			{
				//Rotate 45 CCW 
				ForwardSlow();
				RotateLeftSlow();
			}
		}
		else
		{
			if(GetFrontWarning()) //Only front
			{
				//Stop, rotate 90 CW and run again
				KeepSteady();
				RotateRightSlow();
			}
			else //No warnings at all
			{
				//Straight ahead
				//ForwardFast();
			}
		}
	}
}


//Not correct function name
static void SerialComSignal()
{
	if(_serialProgram.ProgramID == PROGRAM_AUTOLAND || GetRadioProgram() == PROGRAM_AUTOLAND)
	{
		AutoLand();
		if(!_autoLandMessage)
		{
			printNewLine("Autoland due to program select!", STATUSMODE);
			ResetMessages();
			_autoLandMessage = true;
		}
	}

	if(_serialProgram.ProgramID == PROGRAM_START || GetRadioProgram() == PROGRAM_START)
	{
			StartTakeOff = true;

		//if(deltaFloatTime < 2)
		//	GroundStart(deltaFloatTime);
		//else
			Start();

		if(!_startMessage)
		{
			printNewLine("Starting drone due to program select!", STATUSMODE);
			ResetMessages();
			_startMessage = true;
			StartTakeOff = false;
		}
	}
}

void DecidedProgram(float timer)
{
	AnalyseRadioInput();
	SerialComSignal();
	SonarCheck(); //Check the drone is not flying into something
}