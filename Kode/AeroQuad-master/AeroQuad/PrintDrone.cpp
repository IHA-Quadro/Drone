#include "PrintDrone.h"

void setupPrintDrone()
{
	PrintConfig[DEBUGMODE] = true;
	PrintConfig[STATUSMODE] = false;
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
	Serial1.print("Gyro heading: ");
	//Serial1.println(gyroHeading);
}

void printStatus(const char* s)
{
	if(PrintConfig[STATUSMODE])
	{
		Serial2.println(s);
	}
}

void PrintGyroAxis(byte axis)
{
	if(PrintConfig[GYROMODE])
	{
		if(axis == XAXIS)
			Serial3.println("x-axis is moving - hold still!");
		else if(axis == YAXIS)
			Serial.println("y-axis is moving - hold still!");
		else if(axis == ZAXIS)
			Serial.println("z-axis is moving - hold still!");
	}
}