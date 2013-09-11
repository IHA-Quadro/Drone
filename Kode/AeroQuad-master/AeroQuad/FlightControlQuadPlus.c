#include "FlightControlQuadPlus.h"


int motorMaxCommand[4] = {0,0,0,0};
int motorMinCommand[4] = {0,0,0,0};
int motorConfiguratorCommand[4] = {0,0,0,0};

void applyMotorCommand() {
  motorCommand[FRONT] = throttle - motorAxisCommandPitch - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR] =  throttle + motorAxisCommandPitch - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[RIGHT] = throttle - motorAxisCommandRoll  + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[LEFT] =  throttle + motorAxisCommandRoll  + (YAW_DIRECTION * motorAxisCommandYaw);
}