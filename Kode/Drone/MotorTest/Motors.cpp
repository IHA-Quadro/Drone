#include "Motors.h"

NB_Motors numberOfMotors = FOUR_Motors;
int motorCommand[8] = {0,0,0,0,0,0,0,0};

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
