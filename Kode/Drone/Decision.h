//This file makes the decision of which programs will run.
//It will prioritize safety over anything else but will take SerialCom input as 
//higher priority than autoflight programs


#ifndef _DECISION_
#define _DECISION_

#include "InputAnalysis.h"
#include "Programs.h"
#include "PrintDrone.h"

extern bool StartTakeOff;

void ResetDecisions();
ProgramInput GetActualProgram();
void DecidedProgram();
void SerialComRequest(ProgramInput input);
static void SonarCheck();
static void FMSignal();

#endif
