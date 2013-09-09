// PrintDrone.h

#ifndef _PRINTDRONE_h
#define _PRINTDRONE_h

#include "Arduino.h"
#include "GlobalDefined.h"

#define DEBUGMODE 0
#define STATUSMODE 1
#define GYROMODE 2

class PrintDrone
{
	public:
		PrintDrone();

		void printDebug(const String &s);
		void printStatus(const String &s);
		void printGyro();
		void printGyroAxis(byte axis);

		bool PrintConfig[3];
};

extern PrintDrone PRINTDRONE;

#endif