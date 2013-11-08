#include "Programs.h"

#define SLOW 30
#define FAST 70
#define STEADY 1500
#define STANDARD_HEIGHT 50
#define STEADYTOLERANCE 2



struct ProgramData programData;
struct ProgramInput programInput;

static void ResetData()
{
	programData.xAxis = STEADY;
	programData.yAxis = STEADY;
	programData.zAxis = STEADY;
	programData.aux1 = ALTITUDEHOLDTRUE;
	programData.aux3 = AUTOLANDFALSE;

	programInput.height = STANDARD_HEIGHT;
	programInput.TimeSpanInMiliSec = 0;
	programInput.data = programData;
}

void SelectProgram(int programID)
{
	switch (programID)
	{
	case 1:
		ForwardSlow();
		break;

	case 2:
		ForwardFast();
		break;

	case 3:
		BackwardsSlow();
		break;

	case 4:
		RotateLeftSlow();
		break;

	case 5:
		RotateRightSlow();
		break;

	case 6:
		Start();
		break;

	case 7:
		AutoLand();
		break;

	case 8:
		KeepSteady();
		break;

	default:
		break;
	}
}

void ForwardSlow()
{
	//ResetData();
	programInput.data.xAxis = STEADY + SLOW;
}

void ForwardFast()
{
	//ResetData();
	programInput.data.xAxis = STEADY + FAST;
}

void RotateRightSlow()
{
	//ResetData();
	programInput.data.zAxis = STEADY + SLOW;;
}

void RotateLeftSlow()
{
	//ResetData();
	programInput.data.xAxis = STEADY - SLOW;;
}

void BackwardsSlow()
{
	//ResetData();
	programInput.data.xAxis = STEADY - SLOW;
}

void AutoLand()
{
	//ResetData();
	programInput.data.aux1 = ALTITUDEHOLDTRUE;
	programInput.data.aux3 = AUTOLANDTRUE;
}

void Start()
{
	//ResetData();
	programInput.TimeSpanInMiliSec = 2000;
	programInput.data.aux1 = ALTITUDEHOLDFALSE;
	programInput.data.aux3 = AUTOLANDFALSE;
}

void KeepSteady()
{
	//ResetData();
	programInput.height = STANDARD_HEIGHT;
	programInput.data.aux1 = ALTITUDEHOLDTRUE; //Can't keep steady without
	//programInput.data.aux3 = AUTOLANDFALSE;
}

void GroundStart()
{

	

	//if(_controllerInput[AUX1] == ALTITUDEHOLDTRUE) //If autohold enable
	//	return; //Skip the rest

	//int sonarHeight = MeasureSonar(ALTITUDE_RANGE_FINDER_INDEX); //Bottom sonar

	//int average = calcAverage(sonarHeight);

	//if( (average + STEADYTOLERANCE > programInput.height) && (average - STEADYTOLERANCE < programInput.height))
	//{
	//	if(!stableHeight)
	//		printNewLine("Enabling Altitude Hold", STATUSMODE);

	//	_controllerInput[AUX1] = ALTITUDEHOLDTRUE;
	//	stableHeight = true;
	//}

	//else if(average + STEADYTOLERANCE < programInput.height) //Not high enough
	//{
	//	stableHeight = false;

	//	if(miliSecCounter > initTime) //Still not high enough after initTime
	//	{
	//		miliSecCounter = 0;
	//		spinSpeed += 10;
	//		printInLine("SpinSpeed = ", STATUSMODE);
	//		printInLine(spinSpeed, STATUSMODE);
	//		printInLine(" ; ", STATUSMODE);
	//		printInLine(average, STATUSMODE);
	//		println(STATUSMODE);
	//	}
	//}
	//else if(average  - STEADYTOLERANCE > programInput.height) //Too high 
	//{
	//	stableHeight = false;

	//	if(miliSecCounter > initTime) //Still too high after initTime
	//	{
	//		miliSecCounter = 0;
	//		spinSpeed -= 5;
	//		printInLine("SpinSpeed = ", STATUSMODE);
	//		printInLine(spinSpeed, STATUSMODE);
	//		printInLine(" ; ", STATUSMODE);
	//		printInLine(average, STATUSMODE);
	//		println(STATUSMODE);
	//	}
	//}
}
