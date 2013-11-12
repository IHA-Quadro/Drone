//This file makes the decision of which programs will run.
//It will prioritize safety over anything else but will take SerialCom input as 
//higher priority than autoflight programs


#ifndef _DECISION_
#define _DECISION_

#include "InputAnalysis.h"
#include "ControlFaker.h"
#include "Programs.h"
#include "PrintDrone.h"

extern bool StartTakeOff;

void ResetDecisions();
void ResetMessages();
ProgramInput GetActualProgram();
void DecideProgram();
void SerialComRequest(ProgramInput input);
void GroundTakeOff();
void GroundStart();
static void SonarCheck();
static void FMSignal();
static void SelectProgram(int programID);

#endif
