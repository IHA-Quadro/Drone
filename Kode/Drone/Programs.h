//All programs to run defined in one place along with the program struct

#ifndef _PROGRAMS_
#define _PROGRAMS_

#define PROGRAM_START 1
#define PROGRAM_AUTOLAND 2

#include "GlobalDefined.h"

struct ProgramData 
{
	int xAxis;
	int yAxis;
	int zAxis;
	int aux1;
	int aux3;
};

struct ProgramInput 
{
	int ProgramID;
	ProgramData data;
	int TimeSpanInMiliSec;
	int height;
	int AdditionalData;
};

extern struct ProgramData programData;
extern struct ProgramInput programInput;

void SelectProgram(int programID);
void ForwardSlow();
void ForwardFast();
void RotateRightSlow();
void RotateLeftSlow();
void BackwardsSlow();
void AutoLand();
void Start();
void KeepSteady();

#endif