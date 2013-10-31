//This file makes the decision of which programs will run.
//It will prioritize safety over anything else but will take SerialCom input as 
//higher priority than autoflight programs

#ifndef _DECISION_
#define _DECISION_

#include "Programs.h"

static ProgramInput programToRun;

ProgramInput GetActualProgram();
void LoadAnalyse();
void SerialComRequest(ProgramInput input);


#endif
