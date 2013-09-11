#include "PrintDrone.h"

void setupPrintDrone()
{
	PrintConfig[DEBUGMODE] = true;
	PrintConfig[STATUSMODE] = false;
	PrintConfig[GYROMODE] = true;
}

void printDebug(const String &s)
{
	if(PrintConfig[DEBUGMODE])
	{
		Serial.println(s);
	}
}

void printGyro()
{
	Serial.print("Gyro heading: ");
	//Serial.println(gyroHeading);
}

void printStatus(const String &s)
{
	if(PrintConfig[STATUSMODE])
	{
		Serial.println(s);
	}
}

void PrintGyroAxis(byte axis)
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
