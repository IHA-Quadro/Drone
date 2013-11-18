#ifndef _PRINTDRONE_H_
#define _PRINTDRONE_H_

#include <Arduino.h>
#include "GlobalDefined.h"

#define DEBUGMODE 0
#define STATUSMODE 1
#define GYROMODE 2

void SetupPrintDrone();

bool PrintConfig[3];

void printDebug(const char* s);
void printStatus(const char* s);
void printGyro();
void printGyroAxis(byte axis);


#endif