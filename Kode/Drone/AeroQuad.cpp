#include "AeroQuad.h"

byte calibrateESC = 0;
int testCommand = 1000;

byte previousFlightMode = ATTITUDE_FLIGHT_MODE;
byte flightMode = RATE_FLIGHT_MODE;

unsigned long frameCounter = 0; // main loop executive frame counter
int minArmedThrottle; // initial value configured by user

float G_Dt = 0.002; 
int throttle = 1000;
byte motorArmed = OFF;
byte safetyCheck = OFF;
byte maxLimit = OFF;
byte minLimit = OFF;
float filteredAccel[3] = {0.0,0.0,0.0};
boolean inFlight = false; // true when motor are armed and that the user pass one time the min throttle
float rotationSpeedFactor = 1.0;

// main loop time variable
unsigned long previousTime = 0;
unsigned long currentTime = 0;
unsigned long deltaTime = 0;
// sub loop time variable
unsigned long oneHZpreviousTime = 0;
unsigned long tenHZpreviousTime = 0;
unsigned long lowPriorityTenHZpreviousTime = 0;
unsigned long lowPriorityTenHZpreviousTime2 = 0;
unsigned long fiftyHZpreviousTime = 0;
unsigned long hundredHZpreviousTime = 0;

float aref;

byte  headingHoldConfig   = 0;
float headingHold         = 0; 
float heading             = 0; 
float relativeHeading     = 0; 
byte  headingHoldState    = OFF;

#if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
byte altitudeHoldState = OFF;  // ON, OFF or ALTPANIC
int altitudeHoldBump = 90;
int altitudeHoldPanicStickMovement = 250;
int minThrottleAdjust = -50;
int maxThrottleAdjust = 50;
int altitudeHoldThrottle = 1000;
boolean isAltitudeHoldInitialized = false;


float velocityCompFilter1 = 1.0 / (1.0 + 0.3);
float velocityCompFilter2 = 1 - velocityCompFilter1;

boolean runtimaZBiasInitialized = false;  
float zVelocity = 0.0;
float estimatedZVelocity = 0.0;
float runtimeZBias = 0.0; 
float zDampeningThrottleCorrection = 0.0;

#if defined AltitudeHoldBaro
float baroAltitudeToHoldTarget = 0.0;
#endif
#if defined AltitudeHoldRangeFinder
float sonarAltitudeToHoldTarget = 0.0;
#endif
#endif

#if defined (AutoLanding)
byte autoLandingState = OFF;
boolean isAutoLandingInitialized = false;
int autoLandingThrottleCorrection = 0;
#endif

//#define DEBUG
byte fastTransfer = OFF; // Used for troubleshooting