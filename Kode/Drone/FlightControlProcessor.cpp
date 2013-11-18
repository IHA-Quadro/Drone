#include "FlightControlProcessor.h"

void calculateFlightError()
{
	if (flightMode == ATTITUDE_FLIGHT_MODE) 
	{
		float rollAttitudeCmd  = updatePID((receiverCommand[XAXIS] - receiverZero[XAXIS]) * ATTITUDE_SCALING, kinematicsAngle[XAXIS], &PID[ATTITUDE_XAXIS_PID_IDX]);
		float pitchAttitudeCmd = updatePID((receiverCommand[YAXIS] - receiverZero[YAXIS]) * ATTITUDE_SCALING, -kinematicsAngle[YAXIS], &PID[ATTITUDE_YAXIS_PID_IDX]);
		motorAxisCommandRoll   = updatePID(rollAttitudeCmd, gyroRate[XAXIS], &PID[ATTITUDE_GYRO_XAXIS_PID_IDX]);
		motorAxisCommandPitch  = updatePID(pitchAttitudeCmd, -gyroRate[YAXIS], &PID[ATTITUDE_GYRO_YAXIS_PID_IDX]);
	}
	else 
	{
		motorAxisCommandRoll = updatePID(getReceiverSIData(XAXIS), gyroRate[XAXIS]*rotationSpeedFactor, &PID[RATE_XAXIS_PID_IDX]);
		motorAxisCommandPitch = updatePID(getReceiverSIData(YAXIS), -gyroRate[YAXIS]*rotationSpeedFactor, &PID[RATE_YAXIS_PID_IDX]);
	}
}

void processCalibrateESC()
{
	switch (calibrateESC) { // used for calibrating ESC's
	case 1:
		for (byte motor = 0; motor < LASTMOTOR; motor++)
			motorCommand[motor] = MAXCOMMAND;
		break;
	case 3:
		for (byte motor = 0; motor < LASTMOTOR; motor++)
			motorCommand[motor] = constrain(testCommand, 1000, 1200);
		break;
	case 5:
		for (byte motor = 0; motor < LASTMOTOR; motor++)
			motorCommand[motor] = constrain(motorConfiguratorCommand[motor], 1000, 1200);
		safetyCheck = ON;
		break;
	default:
		for (byte motor = 0; motor < LASTMOTOR; motor++)
			motorCommand[motor] = MINCOMMAND;
	}
	// Send calibration commands to motors
	writeMotors(); // Defined in Motors.h
}

void processAutoLandingAltitudeCorrection() 
{
	if (autoLandingState != OFF) //Should it land?
	{   
		if (autoLandingState == BARO_AUTO_DESCENT_STATE) //Descent by Baro?
		{
			baroAltitudeToHoldTarget -= BARO_AUTO_LANDING_DESCENT_SPEED;

			if (isOnRangerRange(rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX]))  //Will decent by baro anyway
				autoLandingState = SONAR_AUTO_DESCENT_STATE;			
		}
		else if (autoLandingState == SONAR_AUTO_DESCENT_STATE) //Descent by Sonar (yes, plz)
		{
			baroAltitudeToHoldTarget -= BARO_AUTO_LANDING_DESCENT_SPEED;
			sonarAltitudeToHoldTarget -= SONAR_AUTO_LANDING_DESCENT_SPEED;

			if (rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX] < 0.5)
				autoLandingState = MOTOR_AUTO_DESCENT_STATE;
		}
		else 
		{
			autoLandingThrottleCorrection -= 1;
			baroAltitudeToHoldTarget -= BARO_AUTO_LANDING_DESCENT_SPEED;
			sonarAltitudeToHoldTarget -= SONAR_AUTO_LANDING_DESCENT_SPEED; // decent with 0.25 pr sec

			if (((throttle + autoLandingThrottleCorrection) < 1000) || (rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX] < 0.20)) 
			{
				if(motorArmed != OFF)
					printNewLine("Killing motors by autodecent program", STATUSMODE);

				commandAllMotors(MINCOMMAND);
				motorArmed = OFF;
			}
		}
	}
}

void processThrottleCorrection() 
{
	int throttleAdjust = 0;

//#if defined UseGPSNavigator
//	if (navigationState == ON || positionHoldState == ON) {
//		throttleAdjust = throttle / (cos (kinematicsAngle[XAXIS]*0.55) * cos (kinematicsAngle[YAXIS]*0.55));
//		throttleAdjust = constrain ((throttleAdjust - throttle), 0, 50); //compensate max  +/- 25 deg XAXIS or YAXIS or  +/- 18 ( 18(XAXIS) + 18(YAXIS))
//	}
//#endif
//#if defined (AutoLanding)
	throttleAdjust += autoLandingThrottleCorrection;
//#endif

	throttle = constrain((throttle + throttleAdjust),MINCOMMAND,MAXCOMMAND-150);  // limmit throttle to leave some space for motor correction in max throttle manuever
}

void processHardManuevers() 
{
	if ((receiverCommand[XAXIS] < MINCHECK) ||
		(receiverCommand[XAXIS] > MAXCHECK) ||
		(receiverCommand[YAXIS] < MINCHECK) ||
		(receiverCommand[YAXIS] > MAXCHECK)) {  

			for (int motor = 0; motor < LASTMOTOR; motor++) {
				motorMinCommand[motor] = minArmedThrottle;
				motorMaxCommand[motor] = MAXCOMMAND;
			}
	}
}

void processMinMaxCommand()
{
	for (byte motor = 0; motor < LASTMOTOR; motor++)
	{
		motorMinCommand[motor] = minArmedThrottle;
		motorMaxCommand[motor] = MAXCOMMAND;
	}

	int maxMotor = motorCommand[0];

	for (byte motor=1; motor < LASTMOTOR; motor++) {
		if (motorCommand[motor] > maxMotor) {
			maxMotor = motorCommand[motor];
		}
	}

	for (byte motor = 0; motor < LASTMOTOR; motor++) {
		if (maxMotor > MAXCOMMAND) {
			motorCommand[motor] =  motorCommand[motor] - (maxMotor - MAXCOMMAND);
		}
	}
}

void processFlightControl() 
{
	calculateFlightError();
	processHeading();

	if (frameCounter % THROTTLE_ADJUST_TASK_SPEED == 0) 
	{  // 50hz task
		taskCounter++;
		if(taskCounter >= 50)
			taskCounter = 0;

		processAltitudeHold();

		processAutoLandingAltitudeCorrection();

		processThrottleCorrection();
	}

	if (motorArmed && safetyCheck) 
		applyMotorCommand();

	processMinMaxCommand();

	// If throttle in minimum position, don't apply yaw
	if (receiverCommand[THROTTLE] < MINCHECK) 
	{
		for (byte motor = 0; motor < LASTMOTOR; motor++) 
		{
			motorMinCommand[motor] = minArmedThrottle;

			if (inFlight && flightMode == RATE_FLIGHT_MODE) 
				motorMaxCommand[motor] = MAXCOMMAND;

			else 
				motorMaxCommand[motor] = minArmedThrottle;
		}
	}

	// Apply limits to motor commands
	for (byte motor = 0; motor < LASTMOTOR; motor++) {
		motorCommand[motor] = constrain(motorCommand[motor], motorMinCommand[motor], motorMaxCommand[motor]);
	}

	// ESC Calibration
	if (motorArmed == OFF) 
		processCalibrateESC();

	if (motorArmed == ON && safetyCheck == ON)
		writeMotors();
}
