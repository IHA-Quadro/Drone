#ifndef _RECEIVECOMMANDTESTDATA_h
#define _RECEIVECOMMANDTESTDATA_h

#include <Arduino.h>
#include "ControlFaker.h"
#include "MaxSonarRangeFinder.h"
#include "Programs.h"

#define lower 1450
#define upper 1550
#define steady 1500
#define STEADYTOLERANCE 2



extern int miliSecCounter;
extern bool miliSecCounterActive;
extern int throttleSpeed;

void ResetReceiveCommandTestData();
void TestAxis(byte axis);
void RotateDrone(int zaxis);
void MoveDrone(int xaxis, int yaxis);
static void MiliSecOverflow(int timeSpanInMiliSec);
void KeepRunningProgram();
void PrintSonarData(byte sonarID);
//void ThrottleDrone(int throttle);

void AccelerateSpeed(int accelSpeed, int initTime);
void RunProgram(ProgramInput input);
void KeepRunningProgram();

#endif

