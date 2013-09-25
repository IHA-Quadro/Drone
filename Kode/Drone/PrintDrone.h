#ifndef _PRINTDRONE_H_
#define _PRINTDRONE_H_

#include <Arduino.h>
#include "GlobalDefined.h"

enum printModes {
	STATUSMODE = 0,
	GYROMODE,
	WIREMODE,
	MOTORMODE,
	DEBUGMODE // Leave last
};

extern bool PrintConfig[DEBUGMODE];
extern bool previousState[DEBUGMODE];

void SetupPrintDrone();
void SilienceMode(printModes mode, bool active);
void SilenceSerial();
void ActivatePreviousSerial();
void printInLine(int f, printModes mode);
void println(printModes mode);
void printNewLine(String s, printModes mode);
void printInLine(String s, printModes mode);

#endif