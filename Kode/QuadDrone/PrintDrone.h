#ifndef _PRINTDRONE_H_
#define _PRINTDRONE_H_

#include <Arduino.h>
#include "GlobalDefined.h"

#define DEBUGMODE 0
#define STATUSMODE 1
#define GYROMODE 2

extern bool PrintConfig[3];

void SetupPrintDrone();
void printDebug(String s);
void printStatus(String s);
void printData(float f);
void println();
void printText(String s);
void printGyroAxis(byte axis);
void printGyro(String s);


#endif