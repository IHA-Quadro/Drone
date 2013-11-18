#include "AeroQuad.h"
#include "GlobalDefined.h"

void InitializeAeroQuad()
{
	G_Dt = 0.002; 
	throttle = 1000;
	motorArmed = OFF;
	safetyCheck = OFF;
	maxLimit = OFF;
	minLimit = OFF;
	for(int i = 0 ;i < 3; i++)
	{
		filteredAccel[i] = 0;
	}
	inFlight = false; // true when motor are armed and that the user pass one time the min throttle
	rotationSpeedFactor = 1.0;


#if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder

	altitudeHoldState = OFF;  // ON, OFF or ALTPANIC
	altitudeHoldBump = 90;
	altitudeHoldPanicStickMovement = 250;
	minThrottleAdjust = -50;
	maxThrottleAdjust = 50;
	altitudeHoldThrottle = 1000;
	isAltitudeHoldInitialized = false;

	   velocityCompFilter1 = 1.0 / (1.0 + 0.3);
   velocityCompFilter2 = 1 - velocityCompFilter1;

   runtimaZBiasInitialized = false;  
   zVelocity = 0.0;
   estimatedZVelocity = 0.0;
   runtimeZBias = 0.0; 
   zDampeningThrottleCorrection = 0.0;

  #if defined AltitudeHoldBaro
     baroAltitudeToHoldTarget = 0.0;
  #endif  
  #if defined AltitudeHoldRangeFinder
     sonarAltitudeToHoldTarget = 0.0;
  #endif

#endif
}

// called when eeprom is initialized
void initializePlatformSpecificAccelCalibration() 
{
	// Kenny default value, a real accel calibration is strongly recommended
	accelScaleFactor[XAXIS] = 0.0365570020;
	accelScaleFactor[YAXIS] = 0.0363000011;
	accelScaleFactor[ZAXIS] = -0.0384629964;
#ifdef HeadingMagHold
	magBias[XAXIS]  = 1.500000;
	magBias[YAXIS]  = 205.500000;
	magBias[ZAXIS]  = -33.000000;
#endif
}
