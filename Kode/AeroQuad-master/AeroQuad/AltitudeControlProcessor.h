#ifndef _AQ_ALTITUDE_CONTROL_PROCESSOR_H_
#define _AQ_ALTITUDE_CONTROL_PROCESSOR_H_

#if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder

#define INVALID_THROTTLE_CORRECTION -1000
#define ALTITUDE_BUMP_SPEED 0.01

#include "PID.h"
#include "AeroQuad.h"

/**
* processAltitudeHold
* 
* This function is responsible to process the throttle correction 
* to keep the current altitude if selected by the user 
*/
void processAltitudeHold();

#endif

#endif // _AQ_ALTITUDE_CONTROL_PROCESSOR_H_
