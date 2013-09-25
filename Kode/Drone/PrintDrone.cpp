#include "PrintDrone.h"

bool PrintConfig[DEBUGMODE];
bool previousState[DEBUGMODE];

void SetupPrintDrone()
{
	for( int i = 0; i < DEBUGMODE+1; i++)
	{
		PrintConfig[i] = true;
		previousState[i] = true;
	}
	PrintConfig[WIREMODE] = false;
}

void SilenceSerial()
{
	for(int i = 0; i < DEBUGMODE+1; i++)
	{
		previousState[i] = PrintConfig[i];
		PrintConfig[i] = false;
	}
}

void ActivatePreviousSerial()
{
	for(int i = 0; i < DEBUGMODE+1; i++)
	{
		PrintConfig[i] = previousState[i];
	}
}

void SilienceMode(printModes mode, bool active)
{
	PrintConfig[mode] = active;
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

void printInLine(int f, printModes mode)
{
	if(PrintConfig[mode])
	{
		Serial.print(f);
	}
}

void printInLine(String s, printModes mode)
{
	if(PrintConfig[mode])
	{
		Serial.print(s);
	}
}