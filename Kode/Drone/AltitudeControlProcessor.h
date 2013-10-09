// FlightControl.pde is responsible for combining sensor measurements and
// transmitter commands into motor commands for the defined flight configuration (X, +, etc.)

#ifndef _AQ_ALTITUDE_CONTROL_PROCESSOR_H_
#define _AQ_ALTITUDE_CONTROL_PROCESSOR_H_

#include "UserConfiguration.h"
#include "AeroQuad.h"
#include "BarometricSensor.h"
#include "FlightCommandProcessor.h"
#include "PID.h"
#include "RangeFinder.h"

#if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder

#define INVALID_THROTTLE_CORRECTION -1000
#define ALTITUDE_BUMP_SPEED 0.01

extern bool altitudeHoldStateActive, isOnRangerRangeValid;
/**
 * processAltitudeHold
 * 
 * This function is responsible to process the throttle correction 
 * to keep the current altitude if selected by the user 
 */
void processAltitudeHold();

#endif

#endif // _AQ_ALTITUDE_CONTROL_PROCESSOR_H_
