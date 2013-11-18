#include "FlightControlProcessor.h"

void calculateFlightError()
{
#if defined (UseGPSNavigator)
	if (navigationState == ON || positionHoldState == ON) {
		float rollAttitudeCmd  = updatePID((receiverCommand[XAXIS] - receiverZero[XAXIS] + gpsRollAxisCorrection) * ATTITUDE_SCALING, kinematicsAngle[XAXIS], &PID[ATTITUDE_XAXIS_PID_IDX]);
		float pitchAttitudeCmd = updatePID((receiverCommand[YAXIS] - receiverZero[YAXIS] + gpsPitchAxisCorrection) * ATTITUDE_SCALING, -kinematicsAngle[YAXIS], &PID[ATTITUDE_YAXIS_PID_IDX]);
		motorAxisCommandRoll   = updatePID(rollAttitudeCmd, gyroRate[XAXIS], &PID[ATTITUDE_GYRO_XAXIS_PID_IDX]);
		motorAxisCommandPitch  = updatePID(pitchAttitudeCmd, -gyroRate[YAXIS], &PID[ATTITUDE_GYRO_YAXIS_PID_IDX]);
	}
	else
#endif
		if (flightMode == ATTITUDE_FLIGHT_MODE) {
			float rollAttitudeCmd  = updatePID((receiverCommand[XAXIS] - receiverZero[XAXIS]) * ATTITUDE_SCALING, kinematicsAngle[XAXIS], &PID[ATTITUDE_XAXIS_PID_IDX]);
			float pitchAttitudeCmd = updatePID((receiverCommand[YAXIS] - receiverZero[YAXIS]) * ATTITUDE_SCALING, -kinematicsAngle[YAXIS], &PID[ATTITUDE_YAXIS_PID_IDX]);
			motorAxisCommandRoll   = updatePID(rollAttitudeCmd, gyroRate[XAXIS], &PID[ATTITUDE_GYRO_XAXIS_PID_IDX]);
			motorAxisCommandPitch  = updatePID(pitchAttitudeCmd, -gyroRate[YAXIS], &PID[ATTITUDE_GYRO_YAXIS_PID_IDX]);
		}
		else {
			motorAxisCommandRoll = updatePID(getReceiverSIData(XAXIS), gyroRate[XAXIS]*rotationSpeedFactor, &PID[RATE_XAXIS_PID_IDX]);
			motorAxisCommandPitch = updatePID(getReceiverSIData(YAXIS), -gyroRate[YAXIS]*rotationSpeedFactor, &PID[RATE_YAXIS_PID_IDX]);
		}
}

void processCalibrateESC()
{
	byte motor;
	switch (calibrateESC) { // used for calibrating ESC's
	case 1:
		for (motor = 0; motor < LASTMOTOR; motor++)
			motorCommand[motor] = MAXCOMMAND;
		break;
	case 3:
		for (motor = 0; motor < LASTMOTOR; motor++)
			motorCommand[motor] = constrain(testCommand, 1000, 1200);
		break;
	case 5:
		for (motor = 0; motor < LASTMOTOR; motor++)
			motorCommand[motor] = constrain(motorConfiguratorCommand[motor], 1000, 1200);
		safetyCheck = ON;
		break;
	default:
		for (motor = 0; motor < LASTMOTOR; motor++)
			motorCommand[motor] = MINCOMMAND;
	}
	// Send calibration commands to motors
	writeMotors(); // Defined in Motors.h
}

#if defined AutoLanding
void processAutoLandingAltitudeCorrection() {
	if (autoLandingState != OFF) {   

		if (autoLandingState == BARO_AUTO_DESCENT_STATE) {
			baroAltitudeToHoldTarget -= BARO_AUTO_LANDING_DESCENT_SPEED;
			if (isOnRangerRange(rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX])) { 
				autoLandingState = SONAR_AUTO_DESCENT_STATE;
			}
		}
		else if (autoLandingState == SONAR_AUTO_DESCENT_STATE) {
			baroAltitudeToHoldTarget -= BARO_AUTO_LANDING_DESCENT_SPEED;
			sonarAltitudeToHoldTarget -= SONAR_AUTO_LANDING_DESCENT_SPEED;
			if (rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX] < 0.5) {
				autoLandingState = MOTOR_AUTO_DESCENT_STATE;
			}
		}
		else {
			autoLandingThrottleCorrection -= 1;
			baroAltitudeToHoldTarget -= BARO_AUTO_LANDING_DESCENT_SPEED;
			sonarAltitudeToHoldTarget -= SONAR_AUTO_LANDING_DESCENT_SPEED;

			if (((throttle + autoLandingThrottleCorrection) < 1000) || (rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX] < 0.20)) {
				commandAllMotors(MINCOMMAND);
				motorArmed = OFF;
			}
		}
	}
}
#endif

void processThrottleCorrection() {
 
  int throttleAdjust = 0;
  #if defined UseGPSNavigator
    if (navigationState == ON || positionHoldState == ON) {
      throttleAdjust = throttle / (cos (kinematicsAngle[XAXIS]*0.55) * cos (kinematicsAngle[YAXIS]*0.55));
      throttleAdjust = constrain ((throttleAdjust - throttle), 0, 50); //compensate max  +/- 25 deg XAXIS or YAXIS or  +/- 18 ( 18(XAXIS) + 18(YAXIS))
    }
  #endif
  #if defined BattMonitorAutoDescent
    throttleAdjust += batteyMonitorThrottleCorrection;
  #endif
  #if defined (AutoLanding)
    #if defined BattMonitorAutoDescent
      if (batteyMonitorThrottleCorrection != 0) { // don't auto land in the same time that the battery monitor do auto descent, or Override the auto descent to land, TBD
        throttleAdjust += autoLandingThrottleCorrection;
      }
    #else
      throttleAdjust += autoLandingThrottleCorrection;
    #endif
  #endif
  
  throttle = constrain((throttle + throttleAdjust),MINCOMMAND,MAXCOMMAND-150);  // limmit throttle to leave some space for motor correction in max throttle manuever
}

void processHardManuevers() 
{
	int motor;

  if ((receiverCommand[XAXIS] < MINCHECK) ||
      (receiverCommand[XAXIS] > MAXCHECK) ||
      (receiverCommand[YAXIS] < MINCHECK) ||
      (receiverCommand[YAXIS] > MAXCHECK)) {  
        
    for (motor = 0; motor < LASTMOTOR; motor++) 
		{
      motorMinCommand[motor] = minArmedThrottle;
      motorMaxCommand[motor] = MAXCOMMAND;
    }
  }
}

void processMinMaxCommand()
{
	byte motor;
  for (motor = 0; motor < LASTMOTOR; motor++)
  {
    motorMinCommand[motor] = minArmedThrottle;
    motorMaxCommand[motor] = MAXCOMMAND;
  }

  int maxMotor = motorCommand[0];
  
  for (motor=1; motor < LASTMOTOR; motor++) {
    if (motorCommand[motor] > maxMotor) {
      maxMotor = motorCommand[motor];
    }
  }
    
  for (motor = 0; motor < LASTMOTOR; motor++) {
    if (maxMotor > MAXCOMMAND) {
      motorCommand[motor] =  motorCommand[motor] - (maxMotor - MAXCOMMAND);
    }
  }
}

void processFlightControl() {
  
  // ********************** Calculate Flight Error ***************************
  calculateFlightError();
  
  // ********************** Update Yaw ***************************************
  processHeading();
  
  if (frameCounter % THROTTLE_ADJUST_TASK_SPEED == 0) {  // 50hz task
    
    // ********************** Process position hold or navigation **************************
    #if defined (UseGPS)
      #if defined (UseGPSNavigator)
        processGpsNavigation();
      #endif  
    #endif
    
    // ********************** Process Altitude hold **************************
    #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
      processAltitudeHold();
    #else
      throttle = receiverCommand[THROTTLE];
    #endif
    
    // ********************** Process Battery monitor hold **************************
    #if defined BattMonitor && defined BattMonitorAutoDescent
      processBatteryMonitorThrottleAdjustment();
    #endif

    // ********************** Process Auto-Descent  **************************
    #if defined AutoLanding
      processAutoLandingAltitudeCorrection();
    #endif
    
    // ********************** Process throttle correction ********************
    processThrottleCorrection();
  }

  // ********************** Calculate Motor Commands *************************
  if (motorArmed && safetyCheck) {
    applyMotorCommand(); //TODO: Check denne kommando
  } 

  // *********************** process min max motor command *******************
  processMinMaxCommand();

  // If throttle in minimum position, don't apply yaw
  if (receiverCommand[THROTTLE] < MINCHECK) 
	{
		byte motor;
    for (motor = 0; motor < LASTMOTOR; motor++) 
		{
      motorMinCommand[motor] = minArmedThrottle;

      if (inFlight && flightMode == RATE_FLIGHT_MODE) 
        motorMaxCommand[motor] = MAXCOMMAND;
      
			else 
        motorMaxCommand[motor] = minArmedThrottle;
    }
  }

  byte motor;
  // Apply limits to motor commands
  for (motor = 0; motor < LASTMOTOR; motor++) {
    motorCommand[motor] = constrain(motorCommand[motor], motorMinCommand[motor], motorMaxCommand[motor]);
  }

  // ESC Calibration
  if (motorArmed == OFF) {
    processCalibrateESC();
  }
  
  // *********************** Command Motors **********************
  if (motorArmed == ON && safetyCheck == ON) {
    writeMotors();
  }
}