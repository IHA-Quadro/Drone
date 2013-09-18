// FlightCommandProcessor is responsible for decoding transmitter stick combinations
// for setting up AeroQuad modes such as motor arming and disarming
#ifndef _AQ_FLIGHT_COMMAND_READER_H_
#define _AQ_FLIGHT_COMMAND_READER_H_

#include "AeroQuad.h"
#include "BarometricSensor.h"
#include "FlightControlQuadPlus.h"
#include "GlobalDefined.h"
#include "PID.h"
#include "PrintDrone.h"
#include "RangeFinder.h"
#include "Receiver.h"
#include "UserConfiguration.h"

#if defined (AltitudeHoldBaro) || defined (AltitudeHoldRangeFinder)
boolean isPositionHoldEnabledByUser();
#endif

#if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
void processAltitudeHoldStateFromReceiverCommand();
#endif


#if defined (AutoLanding)
void processAutoLandingStateFromReceiverCommand();
#endif

#if defined (UseGPSNavigator)
void processGpsNavigationStateFromReceiverCommand() 
#endif

void processZeroThrottleFunctionFromReceiverCommand();

/**
* readPilotCommands
* 
* This function is responsible to read receiver
* and process command from the users
*/
void readPilotCommands();

#endif 

