#include "Programs.h"

#define SLOW 30
#define FAST 70
#define STEADY 1500
#define STANDARD_HEIGHT 50
#define STEADYTOLERANCE 2



struct ProgramData programData;
struct ProgramInput programInput;

void ResetProgramData()
{
	programData.xAxis = STEADY;
	programData.yAxis = STEADY;
	programData.zAxis = STEADY;
	programData.aux1 = ALTITUDEHOLDFALSE;
	programData.aux3 = AUTOLANDFALSE;

	programInput.height = STANDARD_HEIGHT;
	programInput.TimeSpanInMiliSec = 0;
	programInput.data = programData;
	programInput.ProgramID = 0;
}



void ForwardSlow()
{
	programInput.data.xAxis = STEADY + SLOW;
	programInput.data.yAxis = STEADY;
	programInput.data.zAxis = STEADY;
}

void ForwardFast()
{
	programInput.data.xAxis = STEADY + FAST;
	programInput.data.yAxis = STEADY;
	programInput.data.zAxis = STEADY;
}

void RotateRightSlow()
{
	//programInput.data.xAxis = STEADY;
	programInput.data.yAxis = STEADY;
	programInput.data.zAxis = STEADY + SLOW;
}

void RotateLeftSlow()
{
	//programData.xAxis = STEADY;
	programInput.data.yAxis = STEADY;
	programInput.data.zAxis = STEADY - SLOW;
}

void BackwardsSlow()
{
	programInput.data.xAxis = STEADY - SLOW;
	programInput.data.yAxis = STEADY;
	programInput.data.zAxis = STEADY;
}

void AutoLand()
{
	programInput.data.xAxis = STEADY;
	programInput.data.yAxis = STEADY;
	programInput.data.zAxis = STEADY;
	programInput.data.aux1 = ALTITUDEHOLDTRUE;
	programInput.data.aux3 = AUTOLANDTRUE;
}

void Start()
{
	programInput.TimeSpanInMiliSec = 2000;
	programInput.data.aux1 = ALTITUDEHOLDFALSE;
	programInput.data.aux3 = AUTOLANDFALSE;
}

void KeepSteady()
{
	programInput.height = STANDARD_HEIGHT;
	programInput.data.aux1 = ALTITUDEHOLDTRUE; //Can't keep steady without
	//programInput.data.aux3 = AUTOLANDFALSE;
}

void StopMidAir()
{
	programInput.data.xAxis = STEADY;
	programInput.data.yAxis = STEADY;
	programInput.data.zAxis = STEADY;
}


//void GroundStart()
//{
//	//if(_controllerInput[AUX1] == ALTITUDEHOLDTRUE) //If autohold enable
//	//	return; //Skip the rest
//
//	//int sonarHeight = MeasureSonar(ALTITUDE_RANGE_FINDER_INDEX); //Bottom sonar
//
//	//int average = calcAverage(sonarHeight);
//
//	//if( (average + STEADYTOLERANCE > programInput.height) && (average - STEADYTOLERANCE < programInput.height))
//	//{
//	//	if(!stableHeight)
//	//		printNewLine("Enabling Altitude Hold", STATUSMODE);
//
//	//	_controllerInput[AUX1] = ALTITUDEHOLDTRUE;
//	//	stableHeight = true;
//	//}
//
//	//else if(average + STEADYTOLERANCE < programInput.height) //Not high enough
//	//{
//	//	stableHeight = false;
//
//	//	if(miliSecCounter > initTime) //Still not high enough after initTime
//	//	{
//	//		miliSecCounter = 0;
//	//		spinSpeed += 10;
//	//		printInLine("SpinSpeed = ", STATUSMODE);
//	//		printInLine(spinSpeed, STATUSMODE);
//	//		printInLine(" ; ", STATUSMODE);
//	//		printInLine(average, STATUSMODE);
//	//		println(STATUSMODE);
//	//	}
//	//}
//	//else if(average  - STEADYTOLERANCE > programInput.height) //Too high 
//	//{
//	//	stableHeight = false;
//
//	//	if(miliSecCounter > initTime) //Still too high after initTime
//	//	{
//	//		miliSecCounter = 0;
//	//		spinSpeed -= 5;
//	//		printInLine("SpinSpeed = ", STATUSMODE);
//	//		printInLine(spinSpeed, STATUSMODE);
//	//		printInLine(" ; ", STATUSMODE);
//	//		printInLine(average, STATUSMODE);
//	//		println(STATUSMODE);
//	//	}
//	//}
//}
