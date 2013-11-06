//This file makes the decision of which programs will run.
//It will prioritize safety over anything else but will take SerialCom input as 
//higher priority than autoflight programs


#ifndef _DECISION_
#define _DECISION_

#include "InputAnalysis.h"
#include "Programs.h"
#include "PrintDrone.h"

void ResetDecisions();
ProgramInput GetActualProgram();
void DecidedProgram();
void SerialComRequest(ProgramInput input);
void SonarCheck();
void FMSignal();

#endif
