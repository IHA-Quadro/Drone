#include "FlightControlQuadPlus.h"

void applyMotorCommand() 
{
  motorCommand[FRONT] = throttle - motorAxisCommandPitch - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR] =  throttle + motorAxisCommandPitch - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[RIGHT] = throttle - motorAxisCommandRoll  + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[LEFT] =  throttle + motorAxisCommandRoll  + (YAW_DIRECTION * motorAxisCommandYaw);
}