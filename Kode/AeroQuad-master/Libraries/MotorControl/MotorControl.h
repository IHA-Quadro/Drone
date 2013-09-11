#ifndef _MOTORCONTROL_h
#define _MOTORCONTROL_h

#include "Arduino.h"
#include <stdbool.h>

void setMotorStatus(bool status);
bool getMotorStatus();

#endif