#ifndef _AQ_PROCESS_FLIGHT_CONTROL_PLUS_MODE_H_
#define _AQ_PROCESS_FLIGHT_CONTROL_PLUS_MODE_H_

/*  
                 CW
            
           0....Front....0  
           ......***......    
      CCW  ......***......  CCW
           ......***......    
           0....Back.....0  
      
                 CW
*/     

#include "AeroQuad.h"
#include "FlightControlVariable.h"
#include "Motors.h"

#ifdef OLD_MOTOR_NUMBERING  
  #define FRONT MOTOR1
  #define REAR  MOTOR2
  #define RIGHT MOTOR3
  #define LEFT  MOTOR4
#else
  #define FRONT MOTOR1
  #define RIGHT MOTOR2
  #define REAR  MOTOR3
  #define LEFT  MOTOR4
#endif
#define LASTMOTOR (MOTOR4+1)

int motorMaxCommand[4] = {0,0,0,0};
int motorMinCommand[4] = {0,0,0,0};
int motorConfiguratorCommand[4] = {0,0,0,0};

void applyMotorCommand();

#endif // #define _AQ_PROCESS_FLIGHT_CONTROL_PLUS_MODE_H_
