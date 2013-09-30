#include "MotorControl.h"

bool _killMotor = true;

void KillMotor(bool status)
{
	_killMotor = status;
}

bool IsMotorKilled()
{
	return _killMotor;
}