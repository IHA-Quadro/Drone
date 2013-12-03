#include "Decision.h"

#define START_LEVEL 1200
#define WANTED_LEVEL 1450
#define TIME_FACTOR 1.2
#define STEADYTOLERANCE 20
#define INC_DELAY 350
#define IN_FLIGHT_TOLERANCE 40
#define MAX_HEIGHT 100
#define INC_TIME 300 //sec for increasing throttle

enum Timers { 
	IncHeight, 
	DecHeight, 
	Program,
	GrdStart, 
	GrdTakeOff //Leave last
};


ProgramInput _serialProgram;
bool _autoLandMessage;
bool _startMessage;
bool StartTakeOff;

unsigned long _previousTime[GrdTakeOff], _currentTime[GrdTakeOff], _deltaTime[GrdTakeOff];
float _previousFloatTime[GrdTakeOff], _startTime[GrdTakeOff], _deltaFloatTime[GrdTakeOff], _SteadyStamp[GrdTakeOff];
bool _groundStart[GrdTakeOff];
bool stableHeight = false;

void ResetMessages()
{
	_autoLandMessage = false;
	_startMessage = false;
	StartTakeOff = false;

	for( int i = 0; i < GrdTakeOff+1; i++)
	{
		_currentTime[i] = 0.0;
		_previousTime[i] = 0.0;
		_deltaTime[i] = 0.0;
		_previousFloatTime[i] = 0.0;
		_startTime[i] = 0.0;
		_deltaFloatTime[i] = 0.0;
		_SteadyStamp[i] = 0.0;
		_groundStart[i] = true;
	}

	ResetProgramData();
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
	//ResetMessages();
}

static void PrintWarnings()
{
	bool r = false, l = false, c = false;
	if(GetLeftWarning())
		l = true;

	if(GetRightWarning())
		r = true;

	if(GetFrontWarning())
		c = true;

	if(!r && !l && !c)
		return;

	printInLine("Warning: ", WARNINGMODE);
	printInLine((l ? "Left, " : ""), WARNINGMODE);
	printInLine((c ? "Center, " : ""), WARNINGMODE);
	printInLine((r ? "Right, " : ""), WARNINGMODE);
	printNewLine("", WARNINGMODE);
}

//Choose program based on sonars
static void SonarCheck()
{
	AnalyseSonarInput();

	if(_controllerInput[AUX1] == ALTITUDEHOLDTRUE )
	{
		//PrintWarnings();

		if(GetLeftWarning()) //Left warning
		{
			if(GetRightWarning())
			{
				if(GetFrontWarning()) //Left + Right + front
				{
					//Stop, rotate CW and run again
					StopMidAir();
					KeepSteady(); //Stop
					RotateLeftSlow();
				}
				else //Left + Right
				{
					//Stop, rotate 90 CW and run again			
					StopMidAir();
					KeepSteady();
					RotateLeftSlow();
				}
			}
			else
			{
				if(GetFrontWarning()) //Left + front
				{
					//Stop, rotate 60 CW and run again
					StopMidAir();
					KeepSteady();
					RotateRightSlow();
				}
				else //Only Left warning
				{
					//Rotate 45 CW 
					RotateRightSlow();
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
					StopMidAir();
					KeepSteady();
					RotateLeftSlow();
				}
				else //Only right
				{
					//Rotate 45 CCW 
					RotateLeftSlow();
				}
			}
			else
			{
				if(GetFrontWarning()) //Only front
				{
					//Stop, rotate 90 CW and run again
					StopMidAir();
					KeepSteady();
					RotateRightSlow();
				}
				else //No warnings at all
				{
					//Straight ahead
					KeepSteady();
					StopMidAir();
				}
			}
		}
	}
}

static void ProcessSignal()
{
	if(_serialProgram.ProgramID == PROGRAM_AUTOLAND || GetRadioProgram() == PROGRAM_AUTOLAND)
		SelectProgram(2);

	else if(_serialProgram.ProgramID == PROGRAM_START || GetRadioProgram() == PROGRAM_START)
		SelectProgram(1);

	else
		SelectProgram(GetRadioProgram());
}

void DecideProgram()
{
	AnalyseRadioInput();
	ProcessSignal();
	SonarCheck(); //Check the drone is not flying into something
}

static void SelectProgram(int programID)
{
	programInput.ProgramID = programID;
	bool _altitudeHoldActivated = false;

	switch (programID)
	{
	case 1: //Let
		StartTakeOff = true;
		Start();

		if(_controllerInput[AUX1] == ALTITUDEHOLDFALSE) //Not avaiable if in automode
			IncreaseHeight();

		if(!_startMessage)
		{
			printNewLine("Starting drone due to program select!", STATUSMODE);
			ResetMessages();
			_startMessage = true;
			StartTakeOff = false;
		}
		break;

	case 2: //Autoland
		AutoLand();
		if(!_autoLandMessage)
		{
			printNewLine("Autoland due to program select!", STATUSMODE);
			ResetMessages();
			_autoLandMessage = true;
		}
		break;

	case 3: //Forward
		if(_controllerInput[AUX3] != AUTOLANDTRUE)
		{
			StopMidAir();
			DecreaseHeight();
			//ForwardSlow();
		}
		break;

	case 4://backwards
		if(_controllerInput[AUX3] != AUTOLANDTRUE)
		{
			StopMidAir(); 
			ForwardSlow();
			//BackwardsSlow();
		}
		break;

	case 5: //RotateRight
		if(_controllerInput[AUX3] != AUTOLANDTRUE)
		{
			_altitudeHoldActivated = _controllerInput[AUX1];
			StopMidAir();
			RotateRightSlow();
		}
		break;

	case 6: //RotateLeft
		if(_controllerInput[AUX3] != AUTOLANDTRUE)
		{
			StopMidAir();
			RotateLeftSlow();
		}
		break;

	case 7: // Stop mid air / Hover
		if(_controllerInput[AUX3] != AUTOLANDTRUE)
		{
			StopMidAir();
			HoldHeight();
		}
		break;

	case 8: //Kill drone - CAREFULL!
		KillMotor();
		break;

	case 9: //Up
		programInput.height += 5;
		break;

	case 10: //Down
		programInput.height -= 5;
		break;

	default:
		break;
	}



	if (programID != 1 && programID != 2 && programID != 8)
	{
		//If program is in AUX1 = true, disable it and perform program for
		//programInput.TimeSpanInMiliSec for the selected program, then enable agin
	}
}

//Returns miliSeconds (1000 ms/s)
float TimeSpend(Timers timer)
{
	_currentTime[timer] = micros(); //1.000.000 µSec/s
	_deltaTime[timer] = (_currentTime[timer] - _previousTime[timer])/1000; //1000 ms/s

	_previousTime[timer] = _currentTime[timer];

	if(_groundStart[timer] == true)
	{
		_startTime[timer] = (float)_currentTime[timer]/1000;
		_groundStart[timer] = false;
	}

	_previousFloatTime[timer] = (float)_previousTime[timer]/1000;
	return _previousFloatTime[timer] - _startTime[timer];
}

//Take off from ground with (a-b)*(1-exp(-t/tau))+b
//a = wanted motor throttle, b = motor min-throttle
void GroundTakeOff()
{
	if(GetActualProgram().ProgramID == PROGRAM_START)
	{
		float spend = TimeSpend(GrdTakeOff);
		//printInLine("Spend: ", RADIOMODE);
		//printInLine(spend, RADIOMODE);
		//printInLine(" <= Max: ", RADIOMODE);
		//printNewLine(programInput.TimeSpanInMiliSec, RADIOMODE);

		if(_controllerInput[THROTTLE] > WANTED_LEVEL - 10)
			_SteadyStamp[GrdTakeOff] = spend;

		if(spend < programInput.TimeSpanInMiliSec && _controllerInput[AUX1] == ALTITUDEHOLDFALSE)
		{
			printNewLine("Calculate new throttle: ", RADIOMODE);
			printInLine(spend/100, RADIOMODE);
			int throttle = (WANTED_LEVEL-START_LEVEL) * (1- exp(-(spend/100)/TIME_FACTOR))+START_LEVEL;

			_controllerInput[THROTTLE] = throttle;
			printInLine( " => ", RADIOMODE);
			printNewLine(throttle, RADIOMODE);

			if(_SteadyStamp[GrdTakeOff] > 2)
			{
				if(throttle > WANTED_LEVEL - 3)
					_controllerInput[AUX1] = ALTITUDEHOLDTRUE;
			}
		}
	}
}

void GroundStart()
{
	if(GetActualProgram().ProgramID == PROGRAM_START)
	{
		float spend = TimeSpend(GrdStart);

		int sonarHeight = (int)(RangerAverage[ALTITUDE_RANGE_FINDER_INDEX].average *100) + 8; //Bottom sonar

		if((sonarHeight + IN_FLIGHT_TOLERANCE > programInput.height) && (sonarHeight + STEADYTOLERANCE < programInput.height))
		{
			stableHeight = false;

			//if(spend > INC_DELAY) //Still not high enough after initTime
			//{
			//	_groundStart = true;
			//	_controllerInput[THROTTLE] += 5;
			//	printInLine("Throttle = ", STATUSMODE);
			//	printInLine(_controllerInput[THROTTLE], STATUSMODE);
			//	printInLine(" ; ", STATUSMODE);
			//	printInLine(sonarHeight, STATUSMODE);
			//	println(STATUSMODE);
			//}
		}
		else if( (sonarHeight + STEADYTOLERANCE > programInput.height) && (sonarHeight - STEADYTOLERANCE < programInput.height))
		{
			if(!stableHeight)
				printNewLine("Enabling Altitude Hold", STATUSMODE);

			_controllerInput[AUX1] = ALTITUDEHOLDTRUE;
			stableHeight = true;
		}

		else if(sonarHeight + STEADYTOLERANCE < programInput.height) //Not high enough
		{
			stableHeight = false;

			if(spend > INC_DELAY) //Still not high enough after initTime
			{
				_groundStart[GrdStart] = true;
				_controllerInput[THROTTLE] += 10;
				printInLine("Throttle = ", STATUSMODE);
				printInLine(_controllerInput[THROTTLE], STATUSMODE);
				printInLine(" ; ", STATUSMODE);
				printInLine(sonarHeight, STATUSMODE);
				println(STATUSMODE);
			}
		}
		else if(sonarHeight  - STEADYTOLERANCE > programInput.height) //Too high 
		{
			stableHeight = false;

			if(spend > INC_DELAY) //Still too high after initTime
			{
				_groundStart[GrdStart] = true;
				_controllerInput[THROTTLE] -= 5;
				printInLine("Throttle = ", STATUSMODE);
				printInLine(_controllerInput[THROTTLE], STATUSMODE);
				printInLine(" ; ", STATUSMODE);
				printInLine(sonarHeight, STATUSMODE);
				println(STATUSMODE);
			}
		}
	}
}

void IncreaseHeight()
{
	programInput.data.aux1 = ALTITUDEHOLDFALSE;

	if(GetActualProgram().ProgramID == PROGRAM_START)
	{
		float spend = TimeSpend(IncHeight);

		if(spend > INC_TIME)
		{
			_controllerInput[THROTTLE] += 10;
			_groundStart[IncHeight] = true; //Reset timer
		}
	}
}

void DecreaseHeight()
{
	programInput.data.aux1 = ALTITUDEHOLDFALSE;

	float spend = TimeSpend(DecHeight);

	if(spend > INC_TIME)
	{
		_controllerInput[THROTTLE] -= 10;
		_groundStart[DecHeight] = true; //Reset timer
	}
}

void HoldHeight()
{
	int sonarHeight = (int)(RangerAverage[ALTITUDE_RANGE_FINDER_INDEX].average *100) + 8; //Bottom sonar

	printInLine("Sonar Height: ", RADIOMODE);
	printNewLine(sonarHeight, RADIOMODE);

	programInput.height = sonarHeight;
	programInput.data.aux1 = ALTITUDEHOLDTRUE;
	programInput.data.aux3 = AUTOLANDFALSE;
}

void MaxHeightAction()
{
	int sonarHeight = (int)(RangerAverage[ALTITUDE_RANGE_FINDER_INDEX].average *100) + 8; //Bottom sonar

	if(sonarHeight > MAX_HEIGHT)
	{
		programInput.data.aux1 = ALTITUDEHOLDTRUE;
		programInput.height = 60;
	}
}