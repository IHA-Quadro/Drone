#include "MotorControl.h"

bool _killMotor = false;

void setMotorStatus(bool status)
{
	_killMotor = status;
}

bool getMotorStatus()
{
	return _killMotor;
}