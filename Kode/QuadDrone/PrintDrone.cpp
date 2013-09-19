#include "PrintDrone.h"

bool PrintConfig[DEBUGMODE];

void SetupPrintDrone()
{
	for( char i = 0; i < DEBUGMODE+1 ; i++)
	{
		PrintConfig[i] = true;
	}
	PrintConfig[WIREMODE] = false;
}

void printData(float f, printModes mode)
{
	if(PrintConfig[mode])
	{
		Serial.print(f);
	}
}

void printNewLine(String s, printModes mode)
{
	if(PrintConfig[mode])
	{
		Serial.println(s);
	}
}

void println(printModes mode)
{
	if(PrintConfig[mode])
	{
		Serial.println();
	}
}

void printInLine(String s, printModes mode)
{
	if(PrintConfig[mode])
	{
		Serial.print(s);
	}
}