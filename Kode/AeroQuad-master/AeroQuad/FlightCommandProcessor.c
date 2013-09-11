#include "FlightCommandProcessor.h"

#if defined (AltitudeHoldBaro) || defined (AltitudeHoldRangeFinder)
boolean isPositionHoldEnabledByUser() {
#if defined (UseGPSNavigator)
	if ((receiverCommand[AUX1] < 1750) || (receiverCommand[AUX2] < 1750)) {
		return true;
	}
	return false;
#else
	if (receiverCommand[AUX1] < 1750) {
		return true;
	}
	return false;
#endif
}
#endif

#if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
void processAltitudeHoldStateFromReceiverCommand() {
	if (isPositionHoldEnabledByUser()) {
		if (altitudeHoldState != ALTPANIC ) {  // check for special condition with manditory override of Altitude hold
			if (!isAltitudeHoldInitialized) {
#if defined AltitudeHoldBaro
				baroAltitudeToHoldTarget = getBaroAltitude();
				PID[BARO_ALTITUDE_HOLD_PID_IDX].integratedError = 0;
				PID[BARO_ALTITUDE_HOLD_PID_IDX].lastError = baroAltitudeToHoldTarget;
#endif
#if defined AltitudeHoldRangeFinder
				sonarAltitudeToHoldTarget = rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX];
				PID[SONAR_ALTITUDE_HOLD_PID_IDX].integratedError = 0;
				PID[SONAR_ALTITUDE_HOLD_PID_IDX].lastError = sonarAltitudeToHoldTarget;
#endif
				altitudeHoldThrottle = receiverCommand[THROTTLE];
				isAltitudeHoldInitialized = true;
			}
			altitudeHoldState = ON;
		}
	} 
	else {
		isAltitudeHoldInitialized = false;
		altitudeHoldState = OFF;
	}
}
#endif


#if defined (AutoLanding)
void processAutoLandingStateFromReceiverCommand() {
	if (receiverCommand[AUX3] < 1750) {
		if (altitudeHoldState != ALTPANIC ) {  // check for special condition with manditory override of Altitude hold
			if (isAutoLandingInitialized) {
				autoLandingState = BARO_AUTO_DESCENT_STATE;
#if defined AltitudeHoldBaro
				baroAltitudeToHoldTarget = getBaroAltitude();
				PID[BARO_ALTITUDE_HOLD_PID_IDX].integratedError = 0;
				PID[BARO_ALTITUDE_HOLD_PID_IDX].lastError = baroAltitudeToHoldTarget;
#endif
#if defined AltitudeHoldRangeFinder
				sonarAltitudeToHoldTarget = rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX];
				PID[SONAR_ALTITUDE_HOLD_PID_IDX].integratedError = 0;
				PID[SONAR_ALTITUDE_HOLD_PID_IDX].lastError = sonarAltitudeToHoldTarget;
#endif
				altitudeHoldThrottle = receiverCommand[THROTTLE];
				isAutoLandingInitialized = true;
			}
			altitudeHoldState = ON;
		}
	}
	else {
		autoLandingState = OFF;
		autoLandingThrottleCorrection = 0;
		isAutoLandingInitialized = false;
#if defined (UseGPSNavigator)
		if ((receiverCommand[AUX1] > 1750) && (receiverCommand[AUX2] > 1750)) {
			altitudeHoldState = OFF;
			isAltitudeHoldInitialized = false;
		}
#else
		if (receiverCommand[AUX1] > 1750) {
			altitudeHoldState = OFF;
			isAltitudeHoldInitialized = false;
		}
#endif
	}
}
#endif


void processZeroThrottleFunctionFromReceiverCommand() 
{
	// Disarm motors (left stick lower left corner)
	if (receiverCommand[ZAXIS] < MINCHECK && motorArmed == ON) 
	{
		PrintHelper.printDebug("Disarming motors");
		commandAllMotors(MINCOMMAND);
		motorArmed = OFF;
		inFlight = false;

#ifdef OSD
		notifyOSD(OSD_CENTER|OSD_WARN, "MOTORS UNARMED");
#endif

#if defined BattMonitorAutoDescent
		batteryMonitorAlarmCounter = 0;
		batteryMonitorStartThrottle = 0;
		batteyMonitorThrottleCorrection = 0.0;
#endif
	}    

	// Zero Gyro and Accel sensors (left stick lower left, right stick lower right corner)
	if ((receiverCommand[ZAXIS] < MINCHECK) && (receiverCommand[XAXIS] > MAXCHECK) && (receiverCommand[YAXIS] < MINCHECK)) 
	{
		PrintHelper.printDebug("Calibrate gyro");
		calibrateGyro(); 

		PrintHelper.printDebug("Calibrate Acceleration Bias");
		computeAccelBias();

		PrintHelper.printDebug("Store values from sensors");
		storeSensorsZeroToEEPROM();

		PrintHelper.printDebug("Calibrate Kinematics");
		calibrateKinematics();

		PrintHelper.printDebug("Calibrate Integral Errors");
		zeroIntegralError();

		PrintHelper.printDebug("Pulse motors 3 times");
		pulseMotors(3);
	}  

	// Arm motors (left stick lower right corner)
	if (receiverCommand[ZAXIS] > MAXCHECK && motorArmed == OFF && safetyCheck == ON) 
	{
#ifdef OSD_SYSTEM_MENU
		if (menuOwnsSticks) {
			return;
		}
#endif

		for (byte motor = 0; motor < LASTMOTOR; motor++) 
		{
			motorCommand[motor] = MINTHROTTLE;
		}
		PrintHelper.printDebug("Motors armed");
		motorArmed = ON;

#ifdef OSD
		notifyOSD(OSD_CENTER|OSD_WARN, "!MOTORS ARMED!");
#endif  

		zeroIntegralError();
	}

	// Prevents accidental arming of motor output if no transmitter command received
	if (receiverCommand[ZAXIS] > MINCHECK) 
	{
		PrintHelper.printDebug("Safetycheck");
		safetyCheck = ON; 
	}
}

void readPilotCommands() {

	readReceiver(); 

	if (receiverCommand[THROTTLE] < MINCHECK) 
		processZeroThrottleFunctionFromReceiverCommand();


	if (!inFlight) 
	{
		if (motorArmed == ON && receiverCommand[THROTTLE] > minArmedThrottle) 
			inFlight = true;
	}

	// Check Mode switch for Acro or Stable
	if (receiverCommand[MODE] > 1500) 
		flightMode = ATTITUDE_FLIGHT_MODE;

	else 
		flightMode = RATE_FLIGHT_MODE;

	if (previousFlightMode != flightMode) 
	{
		zeroIntegralError();
		previousFlightMode = flightMode;
	}


#if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
	processAltitudeHoldStateFromReceiverCommand();
#endif

#if defined (AutoLanding)
	processAutoLandingStateFromReceiverCommand();
#endif

#if defined (UseGPSNavigator)
	processGpsNavigationStateFromReceiverCommand();
#endif
}