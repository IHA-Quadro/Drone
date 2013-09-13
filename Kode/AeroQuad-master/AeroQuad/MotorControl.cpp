#include "MotorControl.h"

bool _killMotor;

void setMotorStatus(bool status)
{
	_killMotor = status;
}

bool getMotorStatus()
{
	return _killMotor;
}