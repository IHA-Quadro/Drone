#ifndef _AQ_PROCESS_FLIGHT_CONTROL_VARIABLE_H_
#define _AQ_PROCESS_FLIGHT_CONTROL_VARIABLE_H_

#ifdef CHANGE_YAW_DIRECTION
  #define YAW_DIRECTION -1
#else
  #define YAW_DIRECTION 1
#endif

int motorAxisCommandRoll = 0;
int motorAxisCommandPitch = 0;
int motorAxisCommandYaw = 0;

#endif  