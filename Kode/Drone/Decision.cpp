#include "Decision.h"

#define START_LEVEL 1200
#define WANTED_LEVEL 1450
#define TIME_FACTOR 1.2
#define STEADYTOLERANCE 2
#define INC_DELAY 350

ProgramInput _serialProgram;
bool _autoLandMessage;
bool _startMessage;
bool StartTakeOff;

unsigned long _previousTime, _currentTime, _deltaTime;
float _previousFloatTime, _startTime, _deltaFloatTime, _SteadyStamp;
bool _groundStart;
bool stableHeight = false;

void ResetMessages()
{
	_autoLandMessage = false;
	_startMessage = false;
	StartTakeOff = false;
	_currentTime = 0.0;
	_groundStart = true;

	_previousTime = 0.0;
	_currentTime = 0.0;
	_deltaTime = 0.0;
	_previousFloatTime = 0.0;
	_startTime = 0.0;
	_deltaFloatTime = 0.0;
	_SteadyStamp = 0.0;

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

	if(_controllerInput[AUX1] != ALTITUDEHOLDFALSE )
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

	switch (programID)
	{
	case 1: //Start
		StartTakeOff = true;
		Start();

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
		StopMidAir();
		ForwardSlow();
		break;

	case 4://backwards
		StopMidAir(); 
		BackwardsSlow();
		break;

	case 5: //RotateRight
		StopMidAir();
		RotateRightSlow();
		break;

	case 6: //RotateLeft
		StopMidAir();
		RotateLeftSlow();
		break;

	case 7: // Stop mid air
		StopMidAir();
		break;

	case 8: //Kill drone - CAREFULL!
		KillMotor();
		break;

	default:
		break;
	}
}

//Take off from ground with (a-b)*(1-exp(-t/tau))+b
//a = wanted motor throttle, b = motor min-throttle
void GroundTakeOff()
{
	if(GetActualProgram().ProgramID == PROGRAM_START)
	{
		_currentTime = micros(); //1.000.000 µSec/s
		_deltaTime = (_currentTime - _previousTime)/1000; //1000 ms/s

		_previousTime = _currentTime;

		if(_groundStart == true)
		{
			_startTime = (float)_currentTime/1000;
			_groundStart = false;
		}
		_previousFloatTime = (float)_previousTime/1000;
		float spend = _previousFloatTime - _startTime;

		//printInLine("Spend: ", RADIOMODE);
		//printInLine(spend, RADIOMODE);
		//printInLine(" <= Max: ", RADIOMODE);
		//printNewLine(programInput.TimeSpanInMiliSec, RADIOMODE);

		if(_controllerInput[THROTTLE] > WANTED_LEVEL - 10)
			_SteadyStamp = spend;

		if(spend < programInput.TimeSpanInMiliSec && _controllerInput[AUX1] == ALTITUDEHOLDFALSE)
		{
			printNewLine("Calculate new throttle: ", RADIOMODE);
			printInLine(spend/100, RADIOMODE);
			int throttle = (WANTED_LEVEL-START_LEVEL) * (1- exp(-(spend/100)/TIME_FACTOR))+START_LEVEL;

			_controllerInput[THROTTLE] = throttle;
			printInLine( " => ", RADIOMODE);
			printNewLine(throttle, RADIOMODE);

			if(_SteadyStamp > 2)
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
		_currentTime = micros(); //1.000.000 µSec/s
		_deltaTime = (_currentTime - _previousTime)/1000; //1000 ms/s

		_previousTime = _currentTime;

		if(_groundStart == true)
		{
			_startTime = (float)_currentTime/1000;
			_groundStart = false;
		}
		_previousFloatTime = (float)_previousTime/1000;
		float spend = _previousFloatTime - _startTime;

		int sonarHeight = (int)(RangerAverage[ALTITUDE_RANGE_FINDER_INDEX].average *100) + 8; //Bottom sonar

		if( (sonarHeight + STEADYTOLERANCE > programInput.height) && (sonarHeight - STEADYTOLERANCE < programInput.height))
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
				_groundStart = true;
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
				_groundStart = true;
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
