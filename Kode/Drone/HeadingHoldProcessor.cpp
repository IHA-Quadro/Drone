#include "HeadingHoldProcessor.h"

float setHeading          = 0;
unsigned long headingTime = micros();

/**
* processHeading
*
* This function will calculate the craft heading correction depending 
* of the users command. Heading correction is process with the gyro
* or a magnetometer
*/
void processHeading()
{
	if (headingHoldConfig == ON) {

#if defined(HeadingMagHold)
		heading = degrees(trueNorthHeading);
#else
		heading = degrees(gyroHeading);
#endif

		// Always center relative heading around absolute heading chosen during yaw command
		// This assumes that an incorrect yaw can't be forced on the AeroQuad >180 or <-180 degrees
		// This is done so that AeroQuad does not accidentally hit transition between 0 and 360 or -180 and 180
		// AKA - THERE IS A BUG HERE - if relative heading is greater than 180 degrees, the PID will swing from negative to positive
		// Doubt that will happen as it would have to be uncommanded.
		relativeHeading = heading - setHeading;
		if (heading <= (setHeading - 180)) {
			relativeHeading += 360;
		}
		if (heading >= (setHeading + 180)) {
			relativeHeading -= 360;
		}

		// Apply heading hold only when throttle high enough to start flight
		if (receiverCommand[THROTTLE] > MINCHECK ) { 

			//#if defined (UseGPSNavigator)
			//			if (( (receiverCommand[ZAXIS] + gpsYawAxisCorrection) > (MIDCOMMAND + 25)) || 
			//				( (receiverCommand[ZAXIS] + gpsYawAxisCorrection) < (MIDCOMMAND - 25))) {
			//#else
			if ((receiverCommand[ZAXIS] > (MIDCOMMAND + 25)) || (receiverCommand[ZAXIS] < (MIDCOMMAND - 25))) 
			{
				// If commanding yaw, turn off heading hold and store latest heading
				setHeading = heading;
				headingHold = 0;
				PID[HEADING_HOLD_PID_IDX].integratedError = 0;
				headingHoldState = OFF;
				headingTime = currentTime;
			}
			else 
			{
				if (relativeHeading < 0.25 && relativeHeading > -0.25)
				{
					headingHold = 0;
					PID[HEADING_HOLD_PID_IDX].integratedError = 0;
				}
				else if (headingHoldState == OFF) { // quick fix to soften heading hold on new heading
					if ((currentTime - headingTime) > 500000) 
					{
						headingHoldState = ON;
						headingTime = currentTime;
						setHeading = heading;
						headingHold = 0;
					}
				}
				else 
				{
					// No new yaw input, calculate current heading vs. desired heading heading hold
					// Relative heading is always centered around zero
					headingHold = updatePID(0, relativeHeading, &PID[HEADING_HOLD_PID_IDX]);
					headingTime = currentTime; // quick fix to soften heading hold, wait 100ms before applying heading hold
				}
			}
		}
		else {
			// minimum throttle not reached, use off settings
			setHeading = heading;
			headingHold = 0;
			PID[HEADING_HOLD_PID_IDX].integratedError = 0;
		}
	}
	// NEW SI Version
	//#if defined (UseGPSNavigator) 
	//	float receiverSiData = (receiverCommand[ZAXIS] - receiverZero[ZAXIS] + gpsYawAxisCorrection) * (2.5 * PWM2RAD);
	//#else
	float receiverSiData = (receiverCommand[ZAXIS] - receiverZero[ZAXIS]) * (2.5 * PWM2RAD);
	//#endif

	const float commandedYaw = constrain(receiverSiData + radians(headingHold), -PI, PI);
	motorAxisCommandYaw = updatePID(commandedYaw, gyroRate[ZAXIS], &PID[ZAXIS_PID_IDX]);
}