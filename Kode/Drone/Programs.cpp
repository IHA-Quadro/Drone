#include "Programs.h"

#define SLOW 30
#define FAST 70
#define STEADY 1500
#define STANDARD_HEIGHT 50

struct ProgramData programData;

static void ResetData()
{
	programInput.data.xAxis = STEADY;
	programInput.data.yAxis = STEADY;
	programInput.data.zAxis = STEADY;
	programInput.data.aux1 = ALTITUDEHOLDTRUE;
	programInput.data.aux3 = AUTOLANDFALSE;

	programInput.height = STANDARD_HEIGHT;
	programInput.TimeSpanInMiliSec = 0;
}


ProgramInput ForwardSlow()
{
	ResetData();
	programInput.data.xAxis = STEADY + SLOW;
}

ProgramInput ForwardFast()
{
	ResetData();
	programInput.data.xAxis = STEADY + FAST;
}

ProgramInput RotateRightSlow()
{
	ResetData();
	programInput.data.zAxis = STEADY + SLOW;;
}

ProgramInput RotateLeftSlow()
{
	ResetData();
	programInput.data.xAxis = STEADY - SLOW;;
}

ProgramInput BackwardsSlow()
{
	ResetData();
	programInput.data.xAxis = STEADY - SLOW;
}

ProgramInput AutoLand()
{
	ResetData();
	programInput.data.aux1 = ALTITUDEHOLDTRUE;
	programInput.data.aux3 = AUTOLANDTRUE;
}

ProgramInput Start()
{
	ResetData();
	programInput.TimeSpanInMiliSec = 2000;
	programInput.data.aux1 = ALTITUDEHOLDFALSE;
	programInput.data.aux3 = AUTOLANDFALSE;
}

ProgramInput KeepSteady()
{
	ResetData();
	programInput.height = STANDARD_HEIGHT;
	programInput.data.aux1 = ALTITUDEHOLDTRUE;
	programInput.data.aux3 = AUTOLANDFALSE;
}
