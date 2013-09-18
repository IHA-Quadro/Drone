#include "PrintDrone.h"

bool PrintConfig[3];

void SetupPrintDrone()
{
	PrintConfig[DEBUGMODE] = true;
	PrintConfig[STATUSMODE] = true;
	PrintConfig[GYROMODE] = true;
}

void printDebug(const char* s)
{
	if(PrintConfig[DEBUGMODE])
	{
		Serial.println(s);
	}
}

void printGyro()
{
	Serial.print("Gyro heading: ");
	//Serial1.println(gyroHeading);
}

void printStatus(const char* s)
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