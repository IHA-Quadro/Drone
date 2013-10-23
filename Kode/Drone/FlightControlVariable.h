

#ifndef _AQ_PROCESS_FLIGHT_CONTROL_VARIABLE_H_
#define _AQ_PROCESS_FLIGHT_CONTROL_VARIABLE_H_

#ifdef CHANGE_YAW_DIRECTION
  #define YAW_DIRECTION -1
#else
  #define YAW_DIRECTION 1
#endif

extern int motorAxisCommandRoll;
extern int motorAxisCommandPitch;
extern int motorAxisCommandYaw;
extern int taskCounter;

#endif  // #define _AQ_PROCESS_FLIGHT_CONTROL_VARIABLE_H_

