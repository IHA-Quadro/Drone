#include "FlightCommandProcessor.h"

bool isPositionHoldEnabledByUser() 
{
	if (receiverCommand[AUX1] < 1750) 
		return true;

	return false;
}

void processAltitudeHoldStateFromReceiverCommand() 
{
	if (isPositionHoldEnabledByUser()) 
	{
		if (altitudeHoldState != ALTPANIC ) // check for special condition with manditory override of Altitude hold
		{  
			if (!isAltitudeHoldInitialized) 
			{
				baroAltitudeToHoldTarget = getBaroAltitude();
				PID[BARO_ALTITUDE_HOLD_PID_IDX].integratedError = 0;
				PID[BARO_ALTITUDE_HOLD_PID_IDX].lastError = baroAltitudeToHoldTarget;

				sonarAltitudeToHoldTarget = ((float)programInput.height-8)/100; //Height to hold

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
	if (receiverCommand[AUX3] == AUTOLANDTRUE)
	{
		//Write max 10 times a sec and stop when motor is not armed
		if(taskCounter%5 == 0 && motorArmed != OFF)
			printNewLine("Trying to land", STATUSMODE);

		if (altitudeHoldState != ALTPANIC ) 
		{  // check for special condition with manditory override of Altitude hold
			if (!isAutoLandingInitialized) 
			{
				autoLandingState = BARO_AUTO_DESCENT_STATE;
				//autoLandingState = SONAR_AUTO_DESCENT_STATE;
				baroAltitudeToHoldTarget = getBaroAltitude();
				PID[BARO_ALTITUDE_HOLD_PID_IDX].integratedError = 0;
				PID[BARO_ALTITUDE_HOLD_PID_IDX].lastError = baroAltitudeToHoldTarget;

				sonarAltitudeToHoldTarget = rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX];

				PID[SONAR_ALTITUDE_HOLD_PID_IDX].integratedError = 0;
				PID[SONAR_ALTITUDE_HOLD_PID_IDX].lastError = sonarAltitudeToHoldTarget;
				altitudeHoldThrottle = receiverCommand[THROTTLE];

				printNewLine(altitudeHoldThrottle, ALTITUDEMODE); //Speed

				isAutoLandingInitialized = true;
			}
			altitudeHoldState = ON;
		}
		else
			printNewLine("PANIC!", ALTITUDEMODE);
	}
	else 
	{
		autoLandingState = OFF;
		autoLandingThrottleCorrection = 0;
		isAutoLandingInitialized = false;

		if (receiverCommand[AUX1] == ALTITUDEHOLDFALSE) 
		{
			altitudeHoldState = OFF;
			isAltitudeHoldInitialized = false;
		}
	}
}

void processZeroThrottleFunctionFromReceiverCommand() {
	// Disarm motors (left stick lower left corner)
	if (receiverCommand[ZAXIS] < MINCHECK && motorArmed == ON) 
	{
		printNewLine("Disarming motors", STATUSMODE);
		commandAllMotors(MINCOMMAND);
		motorArmed = OFF;
		inFlight = false;
	}    
	// Zero Gyro and Accel sensors (left stick lower left, right stick lower right corner)
	if ((receiverCommand[ZAXIS] < MINCHECK) && (receiverCommand[XAXIS] > MAXCHECK) && (receiverCommand[YAXIS] < MINCHECK)) 
	{
		printNewLine("Calibrateing gyro", STATUSMODE);
		calibrateGyro();

		printNewLine("Computing Accelerometer", STATUSMODE);
		computeAccelBias();

		printNewLine("Storing Sensor value in EEPROM", STATUSMODE);
		storeSensorsZeroToEEPROM();

		printNewLine("Calibrateing kinematics", STATUSMODE);
		calibrateKinematics();
		zeroIntegralError();

		printNewLine("Pulsing motors 3 times", STATUSMODE);
		pulseMotors(3);
	}   

	// Arm motors (left stick lower right corner)
	if (receiverCommand[ZAXIS] > MAXCHECK && motorArmed == OFF && safetyCheck == ON) 
	{
		printNewLine("Arming motors", STATUSMODE);

		for (byte motor = 0; motor < LASTMOTOR; motor++) 
		{
			motorCommand[motor] = MINTHROTTLE;
		}
		motorArmed = ON;

		zeroIntegralError();

	}
	// Prevents accidental arming of motor output if no transmitter command received
	if (receiverCommand[ZAXIS] > MINCHECK) 
	{
		printNewLine("Safetycheck", DEBUGMODE); //TODO: kaldes konstant. Se default programInput
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
