#ifndef _AQ_FLIGHT_COMMAND_READER_
#define _AQ_FLIGHT_COMMAND_READER_

#include "BarometricSensor.h"
#include "FlightControlQuadPlus.h"
#include "PID.h"
#include "RangeFinder.h"
#include "Receiver.h"
#include "UserConfiguration.h"

//#if defined (AltitudeHoldBaro) || defined (AltitudeHoldRangeFinder)
bool isPositionHoldEnabledByUser();  
void processAltitudeHoldStateFromReceiverCommand();
//#endif

//#if defined (AutoLanding)
void processAutoLandingStateFromReceiverCommand();
//#endif

extern bool Holdposition, panic;

#if defined (UseGPSNavigator)
void processGpsNavigationStateFromReceiverCommand() {
	// Init home command
	if (motorArmed == OFF && 
		receiverCommand[THROTTLE] < MINCHECK && receiverCommand[ZAXIS] < MINCHECK &&
		receiverCommand[YAXIS] > MAXCHECK && receiverCommand[XAXIS] > MAXCHECK &&
		haveAGpsLock()) {

			homePosition.latitude = currentPosition.latitude;
			homePosition.longitude = currentPosition.longitude;
			homePosition.altitude = DEFAULT_HOME_ALTITUDE;
	}


	if (receiverCommand[AUX2] < 1750) {  // Enter in execute mission state, if none, go back home, override the position hold
		if (!isGpsNavigationInitialized) {
			gpsRollAxisCorrection = 0;
			gpsPitchAxisCorrection = 0;
			gpsYawAxisCorrection = 0;
			isGpsNavigationInitialized = true;
		}

		positionHoldState = OFF;         // disable the position hold while navigating
		isPositionHoldInitialized = false;

		navigationState = ON;
	}
	else if (receiverCommand[AUX1] < 1250) {  // Enter in position hold state
		if (!isPositionHoldInitialized) {
			gpsRollAxisCorrection = 0;
			gpsPitchAxisCorrection = 0;
			gpsYawAxisCorrection = 0;

			positionHoldPointToReach.latitude = currentPosition.latitude;
			positionHoldPointToReach.longitude = currentPosition.longitude;
			positionHoldPointToReach.altitude = getBaroAltitude();
			isPositionHoldInitialized = true;
		}

		isGpsNavigationInitialized = false;  // disable navigation
		navigationState = OFF;

		positionHoldState = ON;
	}
	else {
		// Navigation and position hold are disabled
		positionHoldState = OFF;
		isPositionHoldInitialized = false;

		navigationState = OFF;
		isGpsNavigationInitialized = false;

		gpsRollAxisCorrection = 0;
		gpsPitchAxisCorrection = 0;
		gpsYawAxisCorrection = 0;
	}
}
#endif

void processZeroThrottleFunctionFromReceiverCommand();

/**
* readPilotCommands
* 
* This function is responsible to read receiver
* and process command from the users
*/
void readPilotCommands();

#endif // _AQ_FLIGHT_COMMAND_READER_

