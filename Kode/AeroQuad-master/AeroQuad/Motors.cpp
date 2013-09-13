#include "Motors.h"

void InitializeMotors()
{	
	byte i;
	for(i = 0; i < 8 ; i++)
	{
		motorCommand[i] = 0;
	}

	numberOfMotors = FOUR_Motors;
}

void pulseMotors(byte nbPulse) 
{
	byte i;
	for (i = 0; i < nbPulse; i++) 
	{
		commandAllMotors(MINCOMMAND + 100);
		delay(250);
		commandAllMotors(MINCOMMAND);
		delay(250);
	}
}