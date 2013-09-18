#include "FlightCommandProcessor.h"

boolean isPositionHoldEnabledByUser() 
{
	if (receiverCommand[AUX1] < 1750) 
		return true;

	return false;
}

void processAltitudeHoldStateFromReceiverCommand() 
{
	if (isPositionHoldEnabledByUser()) 
	{
		if (altitudeHoldState != ALTPANIC ) 
		{  // check for special condition with manditory override of Altitude hold
			if (!isAltitudeHoldInitialized) 
			{
				baroAltitudeToHoldTarget = getBaroAltitude();
				PID[BARO_ALTITUDE_HOLD_PID_IDX].integratedError = 0;
				PID[BARO_ALTITUDE_HOLD_PID_IDX].lastError = baroAltitudeToHoldTarget;

				sonarAltitudeToHoldTarget = rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX];
				PID[SONAR_ALTITUDE_HOLD_PID_IDX].integratedError = 0;
				PID[SONAR_ALTITUDE_HOLD_PID_IDX].lastError = sonarAltitudeToHoldTarget;

				altitudeHoldThrottle = receiverCommand[THROTTLE];
				isAltitudeHoldInitialized = true;
			}
			altitudeHoldState = ON;
		}
	} 
	else 
	{
		isAltitudeHoldInitialized = false;
		altitudeHoldState = OFF;
	}
}


void processAutoLandingStateFromReceiverCommand() 
{
	if (receiverCommand[AUX3] < 1750) 
	{
		if (altitudeHoldState != ALTPANIC ) 
		{  // check for special condition with manditory override of Altitude hold
			if (isAutoLandingInitialized) 
			{
				autoLandingState = BARO_AUTO_DESCENT_STATE;
				baroAltitudeToHoldTarget = getBaroAltitude();
				PID[BARO_ALTITUDE_HOLD_PID_IDX].integratedError = 0;
				PID[BARO_ALTITUDE_HOLD_PID_IDX].lastError = baroAltitudeToHoldTarget;

				sonarAltitudeToHoldTarget = rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX];
				PID[SONAR_ALTITUDE_HOLD_PID_IDX].integratedError = 0;
				PID[SONAR_ALTITUDE_HOLD_PID_IDX].lastError = sonarAltitudeToHoldTarget;

				altitudeHoldThrottle = receiverCommand[THROTTLE];
				isAutoLandingInitialized = true;
			}
			altitudeHoldState = ON;
		}
	}
	else 
	{
		autoLandingState = OFF;
		autoLandingThrottleCorrection = 0;
		isAutoLandingInitialized = false;

		if (receiverCommand[AUX1] > 1750)
		{
			altitudeHoldState = OFF;
			isAltitudeHoldInitialized = false;
		}
	}
}


void processZeroThrottleFunctionFromReceiverCommand() 
{
	// Disarm motors (left stick lower left corner)
	if (receiverCommand[ZAXIS] < MINCHECK && motorArmed == ON) 
	{
		printDebug("Disarming motors");
		commandAllMotors(MINCOMMAND);
		motorArmed = OFF;
		inFlight = false;
	}    

	// Zero Gyro and Accel sensors (left stick lower left, right stick lower right corner)
	if ((receiverCommand[ZAXIS] < MINCHECK) && (receiverCommand[XAXIS] > MAXCHECK) && (receiverCommand[YAXIS] < MINCHECK)) 
	{
		printDebug("Calibrate gyro");
		calibrateGyro(); 

		printDebug("Calibrate Acceleration Bias");
		computeAccelBias();

		printDebug("Store values from sensors");
		storeSensorsZeroToEEPROM();

		printDebug("Calibrate Kinematics");
		calibrateKinematics();

		printDebug("Calibrate Integral Errors");
		zeroIntegralError();

		printDebug("Pulse motors 3 times");
		pulseMotors(3);
	}  

	// Arm motors (left stick lower right corner)
	if (receiverCommand[ZAXIS] > MAXCHECK && motorArmed == OFF && safetyCheck == ON) 
	{
		byte motor;
		for (motor = 0; motor < LASTMOTOR; motor++) 
		{
			motorCommand[motor] = MINTHROTTLE;
		}
		printDebug("Motors armed");
		motorArmed = ON;

		zeroIntegralError();
	}

	// Prevents accidental arming of motor output if no transmitter command received
	if (receiverCommand[ZAXIS] > MINCHECK) 
	{
		printDebug("Safetycheck");
		safetyCheck = ON; 
	}
}

void readPilotCommands() 
{
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

	processAltitudeHoldStateFromReceiverCommand();

	processAutoLandingStateFromReceiverCommand();
}