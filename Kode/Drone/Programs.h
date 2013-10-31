//All programs to run defined in one place along with the program struct

#ifndef _PROGRAMS_
#define _PROGRAMS_

struct ProgramInput 
{
	ProgramData data;
	int TimeSpanInMiliSec;
};

struct ProgramData
{
	int xAxis;
	int yAxis;
	int zAxis;
	int throttle;
	int mode;
	int aux1;
	int aux2;
	int aux3;
};

ProgramInput ForwardSlow();
ProgramInput ForwardFast();
ProgramInput RotateRightSlow();
ProgramInput RotateLeftSlow();
ProgramInput BackwardsSlow();
ProgramInput AutoLand();
ProgramInput Start();
ProgramInput Keep();



#endif