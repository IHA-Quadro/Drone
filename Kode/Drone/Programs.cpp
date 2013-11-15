#include "Programs.h"

#define ROTATE_SLOW 70
#define MOVE_SLOW 60
#define STEADY 1500
#define STANDARD_HEIGHT 100
#define STEADYTOLERANCE 2
#define PROGRAM_TIMER 300 //miliseconds to complete task

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
	programInput.data.xAxis = STEADY;
	programInput.data.yAxis = STEADY + MOVE_SLOW;
	programInput.data.zAxis = STEADY;
	programInput.TimeSpanInMiliSec = PROGRAM_TIMER;
}

void ForwardFast()
{
	programInput.data.xAxis = STEADY;
	programInput.data.yAxis = STEADY + MOVE_SLOW;
	programInput.data.zAxis = STEADY;
	programInput.TimeSpanInMiliSec = PROGRAM_TIMER;
}

void RotateRightSlow()
{
	//programInput.data.xAxis = STEADY;
	programInput.data.yAxis = STEADY;
	programInput.data.zAxis = STEADY + ROTATE_SLOW;
	programInput.TimeSpanInMiliSec = PROGRAM_TIMER;
}

void RotateLeftSlow()
{
	//programData.xAxis = STEADY;
	programInput.data.yAxis = STEADY;
	programInput.data.zAxis = STEADY - ROTATE_SLOW;
	programInput.TimeSpanInMiliSec = PROGRAM_TIMER;
}

void BackwardsSlow()
{
	programInput.data.xAxis = STEADY;
	programInput.data.yAxis = STEADY - MOVE_SLOW;
	programInput.data.zAxis = STEADY;
	programInput.TimeSpanInMiliSec = PROGRAM_TIMER;
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
	//programInput.height = STANDARD_HEIGHT;
	programInput.data.aux1 = ALTITUDEHOLDTRUE; //Can't keep steady without
	//programInput.data.aux3 = AUTOLANDFALSE;
}

void StopMidAir()
{
	programInput.data.xAxis = STEADY;
	programInput.data.yAxis = STEADY;
	programInput.data.zAxis = STEADY;
}