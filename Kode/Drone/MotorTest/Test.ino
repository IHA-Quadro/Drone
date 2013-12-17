#include "Motors_PWM_Timer.h"

void setup()
{

	initializeMotors(FOUR_Motors);

	motorCommand[0] = 1100;
	motorCommand[1] = 1100;
	motorCommand[2] = 1100;
	motorCommand[3] = 1100;
}

void loop()
{

	motorCommand[0] = 1100;
	motorCommand[1] = 1100;
	motorCommand[2] = 1100;
	motorCommand[3] = 1100;

	writeMotors();
	delay(200);

}
