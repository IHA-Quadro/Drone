#include <Arduino.h>
#include <EEPROM.h>

#include "Accelerometer.h"
#include "AeroQuad.h"
#include "ControlFaker.h"
#include "FlightControlProcessor.h"
#include "FlightCommandProcessor.h"
#include "FourtOrderFilter.h"
#include "Gyroscope.h"
#include "InoHelper.h"
#include "Kinematics.h"
#include "Kinematics_ARG.h"
#include "Motors.h"
#include "PrintDrone.h"
#include "Receiver.h"
#include "Receiver_MEGA.h"
#include "SerialCom.h"
#include "UserConfiguration.h" // Edit this file first before uploading to the AeroQuad

//#include "AltitudeControlProcessor.h"
//#include "AQMath.h"
//#include "BarometricSensor.h"
//#include "ControlFaker.h"
//#include "DataStorage.h"
//#include "Device_I2C.h"
//#include "FlightControlQuadPlus.h"
//#include "GlobalDefined.h"
//#include "MaxSonarRangeFinder.h" 
//#include "Motors_PWM_Timer.h"
//#include "PID.h"
//#include "ReceiveCommandTestData.h"

#define MOTOR_PWM_Timer
#define RECEIVER_MEGA
#define BMP085
#define ITG3200_ADDRESS_ALTERNATE
#define XLMAXSONAR 

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
	//measureGyro();
	measureGyroSum();
	measureAccelSum();
}

void PrintGyroData()
{
	//printText("Gyro data: ", GYROMODE);
	//printData(gyroHeading, GYROMODE);
	//printText(", ", GYROMODE);
	//printData(gyroScaleFactor, GYROMODE);
	//printText(", ", GYROMODE);
	//printData(gyroLastMesuredTime, GYROMODE);
	//printText(", ", GYROMODE);
	//printData(gyroSampleCount, GYROMODE);
	//println(GYROMODE);

	//printText("GyroRate: ", GYROMODE);
	//printData(gyroRate[XAXIS], GYROMODE);
	//printText(", ", GYROMODE);
	//printData(gyroRate[YAXIS], GYROMODE);
	//printText(", ", GYROMODE);
	//printData(gyroRate[ZAXIS], GYROMODE);
	//println(GYROMODE);

	//printText("GyroZero: ", GYROMODE);
	//printData(gyroZero[XAXIS], GYROMODE);
	//printText(", ", GYROMODE);
	//printData(gyroZero[YAXIS], GYROMODE);
	//printText(", ", GYROMODE);
	//printData(gyroZero[ZAXIS], GYROMODE);
	//println(GYROMODE);

	//printText("GyroSamples: ", GYROMODE);
	//printData(gyroSample[XAXIS], GYROMODE);
	//printText(", ", GYROMODE);
	//printData(gyroSample[YAXIS], GYROMODE);
	//printText(", ", GYROMODE);
	//printData(gyroSample[ZAXIS], GYROMODE);
	//println(GYROMODE);

	printInLine("ReadShortI2C(): ", GYROMODE);
	printData(readShortI2C(), GYROMODE);
	println(GYROMODE);

	printInLine("ReadShortI2C(ITG3200_ADDRESS): ", GYROMODE);
	printData(readShortI2C(ITG3200_ADDRESS), GYROMODE);
	println(GYROMODE);

	//printText("GyroScaleFactor: ", GYROMODE);
	//printData(gyroScaleFactor, GYROMODE);
	//println(GYROMODE);
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

	printNewLine("Starting setup of drone", DEBUGMODE);
	printNewLine("Initializing base values for drone", DEBUGMODE);

	SetupControlFaker();

	readEEPROM(); // defined in DataStorage.h
	bool firstTimeBoot = false;
	if (readFloat(SOFTWARE_VERSION_ADR) != SOFTWARE_VERSION) 
	{ // If we detect the wrong soft version, we init all parameters
		initializeEEPROM();
		writeEEPROM();
		firstTimeBoot = true;
	}

	initPlatform();

	//Motors
	printNewLine("Initializing Motors", DEBUGMODE);
	InitializeMotors();
	initializeMotors(FOUR_Motors);

	printNewLine("Initializing Receiver", DEBUGMODE);
	initializeReceiverValues();
	initializeReceiver(LASTCHANNEL);

	printNewLine("Initializing EEPROM", DEBUGMODE);
	initReceiverFromEEPROM();

	// Initialize sensors
	// If sensors have a common initialization routine
	// insert it into the gyro class because it executes first
	printNewLine("Initializing Gyro", DEBUGMODE);
	initializeGyro();

	//printGyro("GyroScaleFactor");
	//printData(gyroScaleFactor);
	//printText(" - ");
	//printData(radians(1.0 / 14.375));
	//println();

	while (!calibrateGyro()); // this make sure the craft is still befor to continue init process

	printNewLine("Initializing Accelometer", DEBUGMODE);
	initializeAccel(); // defined in Accel.h
	if (firstTimeBoot) 
	{
		printNewLine("First time boot", DEBUGMODE);
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
	printNewLine("Initializing kinematics", DEBUGMODE);
	initializeKinematics();

	// Optional Sensors
	printNewLine("Settingup altitude barometer", DEBUGMODE);
	initializeBaro();
	vehicleState |= ALTITUDEHOLD_ENABLED;

	//Rangefinder
	printNewLine("Initializing Rangefinder", DEBUGMODE);
	RangeFinderAssign();

	//
	//#if defined(BinaryWrite) || defined(BinaryWritePID)
	//#ifdef OpenlogBinaryWrite
	//	binaryPort = &Serial1;
	//	binaryPort->begin(115200);
	//	delay(1000);
	//#else
	//	binaryPort = &Serial;
	//#endif
	//#endif

	previousTime = micros();
	digitalWrite(LED_Green, HIGH);
	safetyCheck = 0;
}

/*******************************************************************
* 100Hz task
******************************************************************/
void process100HzTask() {

	G_Dt = (currentTime - hundredHZpreviousTime) / 1000000.0;
	hundredHZpreviousTime = currentTime;

	evaluateGyroRate();
	evaluateMetersPerSec();

	for(byte axis = XAXIS; axis <= ZAXIS; axis++) 
	{
		filteredAccel[axis] = computeFourthOrder(meterPerSecSec[axis], &fourthOrder[axis]);
	}

	calculateKinematics(gyroRate[XAXIS], gyroRate[YAXIS], gyroRate[ZAXIS], filteredAccel[XAXIS], filteredAccel[YAXIS], filteredAccel[ZAXIS], G_Dt);

	Process100HzAssign();

	measureBaroSum(); 

	if (frameCounter % THROTTLE_ADJUST_TASK_SPEED == 0)   //  50 Hz tasks
		evaluateBaroAltitude();

	processFlightControl();   
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

	updateRangeFinders();
}

/*******************************************************************
* 10Hz task
******************************************************************/
void process10HzTask1() {

}

/*******************************************************************
* low priority 10Hz task 2
******************************************************************/
void process10HzTask2() 
{
	G_Dt = (currentTime - lowPriorityTenHZpreviousTime) / 1000000.0;
	lowPriorityTenHZpreviousTime = currentTime;

	// Listen for configuration commands and reports telemetry
	readSerialCommand();
	sendSerialTelemetry();
}

/*******************************************************************
* low priority 10Hz task 3
******************************************************************/
void process10HzTask3() 
{
	PrintGyroData();

	G_Dt = (currentTime - lowPriorityTenHZpreviousTime2) / 1000000.0;
	lowPriorityTenHZpreviousTime2 = currentTime;
}

/*******************************************************************
* 1Hz task 
******************************************************************/
void process1HzTask() {
	printNewLine();
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
		}
		previousTime = currentTime;
	}

	if (frameCounter >= 100) 
		frameCounter = 0;
}