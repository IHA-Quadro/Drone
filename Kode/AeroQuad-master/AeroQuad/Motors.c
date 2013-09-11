#include "Motors.h"

enum NB_Motors{
	FOUR_Motors = 4,
	SIX_Motors = 6,
	EIGHT_Motors = 8
};

NB_Motors numberOfMotors = FOUR_Motors;
int motorCommand[8] = {0,0,0,0,0,0,0,0};  // LASTMOTOR not know here, so, default at 8 @todo : Kenny, find a better way



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