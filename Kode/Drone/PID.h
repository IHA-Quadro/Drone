#ifndef _AQ_PID_H_
#define _AQ_PID_H_
#include "AeroQuad.h"

enum {
  RATE_XAXIS_PID_IDX = 0,
  RATE_YAXIS_PID_IDX,
  ZAXIS_PID_IDX,
  ATTITUDE_XAXIS_PID_IDX,
  ATTITUDE_YAXIS_PID_IDX,
  HEADING_HOLD_PID_IDX,
  ATTITUDE_GYRO_XAXIS_PID_IDX,
  ATTITUDE_GYRO_YAXIS_PID_IDX,

  #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
    BARO_ALTITUDE_HOLD_PID_IDX,
    ZDAMPENING_PID_IDX,
  #endif
  
#if defined AltitudeHoldRangeFinder
    SONAR_ALTITUDE_HOLD_PID_IDX,
  #endif

#if defined UseGPSNavigator
    GPSPITCH_PID_IDX,
    GPSROLL_PID_IDX,
    GPSYAW_PID_IDX,
  #endif    

  LAST_PID_IDX  // keep this definition at the end of this enum
};

//// PID Variables
struct PIDdata {
  float P, I, D;
  float lastError;
  // AKA experiments with PID
  float previousPIDTime;
  float integratedError;
  float windupGuard; // Thinking about having individual wind up guards for each PID
};

extern struct PIDdata PID[LAST_PID_IDX];

// This struct above declares the variable PID[] to hold each of the PID values for various functions
// The following constants are declared in AeroQuad.h
// ROLL = 0, PITCH = 1, YAW = 2 (used for Arcobatic Mode, gyros only)
// ROLLLEVEL = 3, PITCHLEVEL = 4, LEVELGYROROLL = 6, LEVELGYROPITCH = 7 (used for Stable Mode, accels + gyros)
// HEADING = 5 (used for heading hold)
// ALTITUDE = 8 (used for altitude hold)
// ZDAMPENING = 9 (used in altitude hold to dampen vertical accelerations)
extern float windupGuard; // Read in from EEPROM
//// Modified from http://www.arduino.cc/playground/Main/BarebonesPIDForEspresso

float updatePID(float targetPosition, float currentPosition, struct PIDdata *PIDparameters);

void zeroIntegralError() __attribute__ ((noinline));
void zeroIntegralError();

#endif // _AQ_PID_H_


