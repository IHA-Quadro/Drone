//All programs to run defined in one place along with the program struct

#ifndef _PROGRAMS_
#define _PROGRAMS_

#include "GlobalDefined.h"

struct ProgramInput 
{
	int ProgramID;
	ProgramData data;
	int TimeSpanInMiliSec;
	int height;
	int AdditionalData;
};

struct ProgramData 
{
	int xAxis;
	int yAxis;
	int zAxis;
	int aux1;
	int aux3;
};

extern struct ProgramData programData;
extern struct ProgramInput programInput;

ProgramInput ForwardSlow();
ProgramInput ForwardFast();
ProgramInput RotateRightSlow();
ProgramInput RotateLeftSlow();
ProgramInput BackwardsSlow();
ProgramInput AutoLand();
ProgramInput Start();
ProgramInput KeepSteady();



#endif