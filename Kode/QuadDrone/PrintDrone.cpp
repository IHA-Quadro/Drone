#include "PrintDrone.h"

bool PrintConfig[3];

void SetupPrintDrone()
{
	PrintConfig[DEBUGMODE] = true;
	PrintConfig[STATUSMODE] = true;
	PrintConfig[GYROMODE] = true;
}

void printDebug(String s)
{
	if(PrintConfig[DEBUGMODE])
	{
		Serial.println(s);
	}
}

void printData(float f)
{
	if(PrintConfig[DEBUGMODE])
	{
		Serial.print(f);
	}
}

void printStatus(String s)
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

void printGyro(String s)
{
		if(PrintConfig[DEBUGMODE])
	{
		Serial.println(s);
	}
}

void println()
{
	if(PrintConfig[GYROMODE])
	{
		Serial.println();
	}
}

void printText(String s)
{
	if(PrintConfig[GYROMODE])
	{
		Serial.print(s);
	}
}