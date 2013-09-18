#include "AeroQuad.h"

byte calibrateESC;
int testCommand;
byte previousFlightMode;
byte flightMode;
unsigned long frameCounter; // main loop executive frame counter
int minArmedThrottle; // initial value configured by user

float G_Dt; 
int throttle;
byte motorArmed;
byte safetyCheck;
byte maxLimit;
byte minLimit;
float filteredAccel[3];
boolean inFlight; // true when motor are armed and that the user pass one time the min throttle
float rotationSpeedFactor;

// main loop time variable
unsigned long previousTime;
unsigned long currentTime;
unsigned long deltaTime;

// sub loop time variable
unsigned long oneHZpreviousTime;
unsigned long tenHZpreviousTime;
unsigned long lowPriorityTenHZpreviousTime;
unsigned long lowPriorityTenHZpreviousTime2;
unsigned long fiftyHZpreviousTime;
unsigned long hundredHZpreviousTime;


// Analog Reference Value
// This value provided from Configurator
// Use a DMM to measure the voltage between AREF and GND
// Enter the measured voltage below to define your value for aref
float aref; // Read in from EEPROM
//TODO: This value is never initialized

byte  headingHoldConfig;
float headingHold; // calculated adjustment for quad to go to heading (PID output)
float heading; // measured heading from yaw gyro (process variable)
float relativeHeading; // current heading the quad is set to (set point)
byte  headingHoldState;

/**
* Altitude control global declaration
*/
// special state that allows immediate turn off of Altitude hold if large throttle changesa are made at the TX
byte altitudeHoldState;  // ON, OFF or ALTPANIC
int altitudeHoldBump;
int altitudeHoldPanicStickMovement;
int minThrottleAdjust;
int maxThrottleAdjust;
int altitudeHoldThrottle;
boolean isAltitudeHoldInitialized;

float velocityCompFilter1;
float velocityCompFilter2;

boolean runtimaZBiasInitialized;  
float zVelocity;
float estimatedZVelocity;
float runtimeZBias; 
float zDampeningThrottleCorrection;

float baroAltitudeToHoldTarget;
float sonarAltitudeToHoldTarget;

byte autoLandingState;
boolean isAutoLandingInitialized;
int autoLandingThrottleCorrection;

/**
* Debug utility global declaration
* Debug code should never be part of a release sofware
* @see Kenny*/

// #define DEBUG
byte fastTransfer; // Used for troubleshooting


void InitializeAeroQuad()
{
	calibrateESC = 0;
	testCommand = 1000;
	previousFlightMode = ATTITUDE_FLIGHT_MODE;

	flightMode = RATE_FLIGHT_MODE;
	frameCounter = 0;

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

	previousTime = 0;
	currentTime = 0;
	deltaTime = 0;
	oneHZpreviousTime = 0;
	tenHZpreviousTime = 0;
	lowPriorityTenHZpreviousTime = 0;
	lowPriorityTenHZpreviousTime2 = 0;
	fiftyHZpreviousTime = 0;
	hundredHZpreviousTime = 0;

	headingHoldConfig = 0;
	headingHold = 0;
	heading = 0;
	relativeHeading = 0;
	headingHoldState = OFF;


	altitudeHoldState = OFF;  // ON, OFF or ALTPANIC
	altitudeHoldBump = 90;
	altitudeHoldPanicStickMovement = 250;
	minThrottleAdjust = -50;
	maxThrottleAdjust = 50;
	altitudeHoldThrottle = 1000;
	isAltitudeHoldInitialized = false;

	autoLandingState = OFF;
	isAutoLandingInitialized = false;
	autoLandingThrottleCorrection = 0;

	velocityCompFilter1 = 1.0 / (1.0 + 0.3);
	velocityCompFilter2 = 1 - velocityCompFilter1;

	runtimaZBiasInitialized = false;  
	zVelocity = 0.0;
	estimatedZVelocity = 0.0;
	runtimeZBias = 0.0; 
	zDampeningThrottleCorrection = 0.0;

	baroAltitudeToHoldTarget = 0.0;
	sonarAltitudeToHoldTarget = 0.0;

	//Debug
	fastTransfer = OFF;
}

// Called when eeprom is initialized
void initializePlatformSpecificAccelCalibration() 
{
	accelScaleFactor[XAXIS] = 0.0365570020;
	accelScaleFactor[YAXIS] = 0.0363000011;
	accelScaleFactor[ZAXIS] = -0.0384629964;
}