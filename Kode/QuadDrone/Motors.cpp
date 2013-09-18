#include "Motors.h"

int motorCommand[4] ={0,0,0,0};

NB_Motors numberOfMotors;

void InitializeMotors()
{	
	for(int i = 0; i < 8 ; i++)
	{
		motorCommand[i] = 0;
	}

	numberOfMotors = FOUR_Motors;
}

void pulseMotors(byte nbPulse) 
{
	for (byte i = 0; i < nbPulse; i++) 
	{
		commandAllMotors(MINCOMMAND + 100);
		delay(250);
		commandAllMotors(MINCOMMAND);
		delay(250);
	}
}