#ifndef INPUTANALYSIS
#define INPUTANALYSIS

#include "MaxSonarRangeFinder.h"
#include "PrintDrone.h"
#include "Programs.h"
#include "RadioCommunication.h"

void ResetInputAnalysis();
void AnalyseSonarInput();
void AnalyseRadioInput();

int GetRadioProgram();
int GetSerialProgram();

bool GetLeftWarning();
bool GetRightWarning();
bool GetFrontWarning();

#endif