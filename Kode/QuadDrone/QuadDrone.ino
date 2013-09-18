//#include <Arduino.h>
//#include <Wire.h> //TODO: Backfire?
//#include <iomxx0_1.h>

#include <EEPROM.h>
#include <Wire.h>

#include "Accelerometer.h"
#include "AeroQuad.h"
#include "ControlFaker.h"
#include "FlightControlProcessor.h"
#include "FlightCommandProcessor.h"
#include "InoHelper.h"
#include "Kinematics.h"
#include "Kinematics_ARG.h"
#include "Motors.h"
#include "PrintDrone.h"
#include "Receiver.h"
#include "Receiver_MEGA.h"
#include "SerialCom.h"
#include "UserConfiguration.h" // Edit this file first before uploading to the AeroQuad

//#include "Device_I2C.h"
//#include "Gyroscope.h"
//#include "Motors_PWM_Timer.h"
//#include "BarometricSensor.h"
//#include "MaxSonarRangeFinder.h" 
//#include "FlightControlQuadPlus.h"
//#include "AltitudeControlProcessor.h"
//#include "DataStorage.h"

#define MOTOR_PWM_Timer
#define RECEIVER_MEGA
#define BMP085
#define ITG3200_ADDRESS_ALTERNATE
#define XLMAXSONAR 

//#include "AQMath.h"
//#include "ControlFaker.h"
#include "FourtOrderFilter.h"
//#include "GlobalDefined.h"
//#include "PID.h"
//#include "ReceiveCommandTestData.h"

void initializePlatformSpecificAccelCalibration();

/**
* Put AeroQuadMega_v21 specific initialization need here
*/
void initPlatform() {

	pinMode(LED_Red, OUTPUT);
	digitalWrite(LED_Red, LOW);
	pinMode(LED_Yellow, OUTPUT);
	digitalWrite(LED_Yellow, LOW);

	// pins set to INPUT for camera stabilization so won't interfere with new camera class
	pinMode(33, INPUT); // disable SERVO 1, jumper D12 for roll
	pinMode(34, INPUT); // disable SERVO 2, jumper D11 for pitch
	pinMode(35, INPUT); // disable SERVO 3, jumper D13 for yaw
	pinMode(43, OUTPUT); // LED 1
	pinMode(44, OUTPUT); // LED 2
	pinMode(45, OUTPUT); // LED 3
	pinMode(46, OUTPUT); // LED 4
	digitalWrite(43, HIGH); // LED 1 on
	digitalWrite(44, HIGH); // LED 2 on
	digitalWrite(45, HIGH); // LED 3 on
	digitalWrite(46, HIGH); // LED 4 on

	Wire.begin();
	TWBR = 12;
}

/**
* Measure critical sensors
*/
void measureCriticalSensors() {
	measureGyroSum();
	measureAccelSum();
}



/*******************************************************************
* Main setup function, called one time at bootup
* initialize all system and sub system of the
* Aeroquad
******************************************************************/
void setup() 
{
	SetupPrintDrone();
	InitializeAeroQuad();

	SERIAL_BEGIN(BAUD);
	pinMode(LED_Green, OUTPUT);
	digitalWrite(LED_Green, LOW);

	printDebug("Starting setup of drone");
	printDebug("Initializing base values for drone");
	
	SetupControlFaker();
	
	readEEPROM(); // defined in DataStorage.h
	boolean firstTimeBoot = false;
	if (readFloat(SOFTWARE_VERSION_ADR) != SOFTWARE_VERSION) 
	{ // If we detect the wrong soft version, we init all parameters
		initializeEEPROM();
		writeEEPROM();
		firstTimeBoot = true;
	}

	initPlatform();

	//Motors
	printDebug("Initializing Motors");
	InitializeMotors();
	initializeMotors(FOUR_Motors);

	printDebug("Initializing Receiver");
	initializeReceiverValues();
	initializeReceiver(LASTCHANNEL);

	printDebug("Initializing EEPROM");
	initReceiverFromEEPROM();

	// Initialize sensors
	// If sensors have a common initialization routine
	// insert it into the gyro class because it executes first
	printDebug("Initializing Gyro");
	initializeGyro(); // defined in Gyro.h
	while (!calibrateGyro()); // this make sure the craft is still befor to continue init process

	printDebug("Initializing Accelometer");
	initializeAccel(); // defined in Accel.h
	if (firstTimeBoot) 
	{
		printDebug("First time boot");
		computeAccelBias();
		writeEEPROM();
	}
	setupFourthOrder();
	initSensorsZeroFromEEPROM();

	// Integral Limit for attitude mode
	// This overrides default set in readEEPROM()
	// Set for 1/2 max attitude command (+/-0.75 radians)
	// Rate integral not used for now
	PID[ATTITUDE_XAXIS_PID_IDX].windupGuard = 0.375;
	PID[ATTITUDE_YAXIS_PID_IDX].windupGuard = 0.375;

	// Flight angle estimation
	printDebug("Initializing kinematics");
	initializeKinematics();

	// Optional Sensors
	printDebug("Settingup altitude barometer");
	initializeBaro();
	vehicleState |= ALTITUDEHOLD_ENABLED;

	//Rangefinder
	printDebug("Initializing Rangefinder");
	RangeFinderAssign();


#if defined(BinaryWrite) || defined(BinaryWritePID)
#ifdef OpenlogBinaryWrite
	binaryPort = &Serial1;
	binaryPort->begin(115200);
	delay(1000);
#else
	binaryPort = &Serial;
#endif
#endif

	previousTime = micros();
	digitalWrite(LED_Green, HIGH);
	safetyCheck = 0;
	
}


/*******************************************************************
* 100Hz task
******************************************************************/
void process100HzTask() {
	int axis;
	G_Dt = (currentTime - hundredHZpreviousTime) / 1000000.0;
	hundredHZpreviousTime = currentTime;

	//printGyro();
	evaluateGyroRate();
	evaluateMetersPerSec();


	for (axis = XAXIS; axis <= ZAXIS; axis++) 
	{
		filteredAccel[axis] = computeFourthOrder(meterPerSecSec[axis], &fourthOrder[axis]);
	}

	calculateKinematics(gyroRate[XAXIS], gyroRate[YAXIS], gyroRate[ZAXIS], filteredAccel[XAXIS], filteredAccel[YAXIS], filteredAccel[ZAXIS], G_Dt);

#if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder

	Process100HzAssign();
#endif    

#if defined(AltitudeHoldBaro)
	measureBaroSum(); 

	if (frameCounter % THROTTLE_ADJUST_TASK_SPEED == 0)   //  50 Hz tasks
		evaluateBaroAltitude();
#endif

	processFlightControl();


#if defined(BinaryWrite)
	// write out fastTelemetry to Configurator or openLog
	if (fastTransfer == ON) 
		fastTelemetry();
#endif      

#ifdef SlowTelemetry
	updateSlowTelemetry100Hz();
#endif

#if defined(UseGPS)
	updateGps();
#endif      

#if defined(CameraControl)
	moveCamera(kinematicsAngle[YAXIS],kinematicsAngle[XAXIS],kinematicsAngle[ZAXIS]);
#if defined CameraTXControl
	processCameraTXControl();
#endif
#endif       

}

/*******************************************************************
* 50Hz task
******************************************************************/
void process50HzTask() 
{
	G_Dt = (currentTime - fiftyHZpreviousTime) / 1000000.0;
	fiftyHZpreviousTime = currentTime;

	// Reads external pilot commands and performs functions based on stick configuration
	readPilotCommands(); 

#if defined(UseAnalogRSSIReader) || defined(UseEzUHFRSSIReader) || defined(UseSBUSRSSIReader)
	readRSSI();
#endif

#ifdef AltitudeHoldRangeFinder
	updateRangeFinders();
#endif

#if defined(UseGPS)
	if (haveAGpsLock() && !isHomeBaseInitialized()) {
		initHomeBase();
	}
#endif      
}

/*******************************************************************
* 10Hz task
******************************************************************/
void process10HzTask1() {

#if defined(HeadingMagHold)

	G_Dt = (currentTime - tenHZpreviousTime) / 1000000.0;
	tenHZpreviousTime = currentTime;

	measureMagnetometer(kinematicsAngle[XAXIS], kinematicsAngle[YAXIS]);

	calculateHeading();

#endif
}

/*******************************************************************
* low priority 10Hz task 2
******************************************************************/
void process10HzTask2() 
{
	G_Dt = (currentTime - lowPriorityTenHZpreviousTime) / 1000000.0;
	lowPriorityTenHZpreviousTime = currentTime;

#if defined(BattMonitor)
	measureBatteryVoltage(G_Dt*1000.0);
#endif

	// Listen for configuration commands and reports telemetry
	readSerialCommand();
	sendSerialTelemetry();
}

/*******************************************************************
* low priority 10Hz task 3
******************************************************************/
void process10HzTask3() 
{
	PrintStatus();

	G_Dt = (currentTime - lowPriorityTenHZpreviousTime2) / 1000000.0;
	lowPriorityTenHZpreviousTime2 = currentTime;

#ifdef OSD_SYSTEM_MENU
	updateOSDMenu();
#endif

#ifdef MAX7456_OSD
	updateOSD();
#endif

#if defined(UseGPS) || defined(BattMonitor)
	processLedStatus();
#endif

#ifdef SlowTelemetry
	updateSlowTelemetry10Hz();
#endif
}

/*******************************************************************
* 1Hz task 
******************************************************************/
void process1HzTask() {
	PrintStatus();

#ifdef MavLink
	G_Dt = (currentTime - oneHZpreviousTime) / 1000000.0;
	oneHZpreviousTime = currentTime;

	sendSerialHeartbeat();   
#endif
}

/*******************************************************************
* Main loop funtions
******************************************************************/
void loop () 
{
	currentTime = micros();
	deltaTime = currentTime - previousTime;

	measureCriticalSensors();

	// ================================================================
	// 100Hz task loop
	// ================================================================
	if (deltaTime >= 10000) 
	{
		frameCounter++;

		process100HzTask();

		// ================================================================
		// 50Hz task loop
		// ================================================================
		if (frameCounter % TASK_50HZ == 0) {  //  50 Hz tasks
			process50HzTask();
		}

		// ================================================================
		// 10Hz task loop
		// ================================================================
		if (frameCounter % TASK_10HZ == 0) {  //   10 Hz tasks
			process10HzTask1();
		}
		else if ((currentTime - lowPriorityTenHZpreviousTime) > 100000) {
			process10HzTask2();
		}
		else if ((currentTime - lowPriorityTenHZpreviousTime2) > 100000) {
			process10HzTask3();
		}

		// ================================================================
		// 1Hz task loop
		// ================================================================
		if (frameCounter % TASK_1HZ == 0) {  //   1 Hz tasks
			process1HzTask();
			printDebug("Looping");
		}

		previousTime = currentTime;
	}

	if (frameCounter >= 100) 
		frameCounter = 0;
}