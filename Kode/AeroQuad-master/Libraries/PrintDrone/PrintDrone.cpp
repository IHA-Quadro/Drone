// Extension of Print.h
// Calls to Print.h for output
// 

#include "PrintDrone.h"

PrintDrone::PrintDrone()
{
	for(int i = DEBUGMODE; i < GYROMODE; i++)
	{
		PrintConfig[i] = true;
	}
}

void PrintDrone::printDebug(const String &s)
{
	if(PrintConfig[DEBUGMODE])
	{
		Serial.println(s);
	}
}

void PrintDrone::printGyro()
{

}

void PrintDrone::printStatus(const String &s)
{
	if(PrintConfig[STATUSMODE])
	{
		Serial.println(s);
	}
}

void printGyroAxis(byte axis)
{
	if(PrintConfig[GYROMODE])
	{
		if(axis == XAXIS)
			Serial.println("x-axis is moving - hold still!");
		else if(axis == YAXIS)
			Serial.println("y-axis is moving - hold still!");
		else if(axis == ZAXIS)
			Serial.println("z-axis is moving - hold still!");
	}
}

PrintDrone PRINTDRONE;