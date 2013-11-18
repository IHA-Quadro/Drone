#ifndef _PRINTDRONE_H_
#define _PRINTDRONE_H_

#include <Arduino.h>
#include "GlobalDefined.h"


enum printModes {
	STATUSMODE = 0,
	GYROMODE,
	WIREMODE,
	DEBUGMODE // Leave last
};

extern bool PrintConfig[DEBUGMODE];

void SetupPrintDrone();
void printData(float f, printModes mode);
void println(printModes mode);
void printNewLine(String s, printModes mode);
void printInLine(String s, printModes mode);

#endif