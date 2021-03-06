#ifndef _AEROQUAD_MOTORS_H_
#define _AEROQUAD_MOTORS_H_

#include <Arduino.h>
#include "GlobalDefined.h"

#define MOTOR1 0
#define MOTOR2 1
#define MOTOR3 2
#define MOTOR4 3
#define MOTOR5 4
#define MOTOR6 5
#define MOTOR7 6
#define MOTOR8 7
#define MINCOMMAND 1000
#define MAXCOMMAND 2000


enum NB_Motors{
	FOUR_Motors = 4,
	SIX_Motors = 6,
	EIGHT_Motors = 8
};

extern NB_Motors numberOfMotors;
extern int motorCommand[4];

void InitializeMotors();
void initializeMotors(NB_Motors numbers);
void writeMotors();
void commandAllMotors(int command);
void pulseMotors(byte nbPulse);

#endif