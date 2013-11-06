#include "Decision.h"

ProgramInput GetActualProgram();
void SerialComRequest(ProgramInput input);

//Resets InputAnalysis
void ResetDecisions()
{
	ResetInputAnalysis();
}

void SonarCheck()
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
				ForwardFast();
			}
		}
	}
}

//Call Analysis.ReadRadioQueue() to get new input
void FMSignal()
{
	AnalyseRadioInput();
}

void DecidedProgram()
{
	//FMSignal();
	SonarCheck(); //Check the drone is not flying into something

	if(GetSerialProgram() == 2 || GetRadioProgram() == 2)
	{
		AutoLand();
		printNewLine("Autoland due to program select!", STATUSMODE);
	}

	if(GetSerialProgram() == 1 || GetRadioProgram() == 1)
	{
		Start();
		printNewLine("Starting drone due to program select!", STATUSMODE);
	}
}