/****************************************************************************
Before flight, select the different user options for your AeroQuad by
editing UserConfiguration.h.

If you need additional assistance go to http://www.aeroquad.com/forum.php
or talk to us live on IRC #aeroquad
*****************************************************************************/

//Arduino includes
#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <utility/twi.h> //Contains 'gI2CCheckBusyAfterStop' and 'I2C_HOW_MANY_BUSY_CHECKS_AFTER_STOP'

//Local includes
#include "Accelerometer.h"
#include "Accelerometer_ADXL345_9DOF.h"
#include "AeroQuad.h"
#include "BarometricSensor.h"
#include "ControlFaker.h"
#include "DataStorage.h"
#include "Device_I2C.h"
#include "FourtOrderFilter.h"
#include "Gyroscope.h"
#include "Gyroscope_ITG3200Common.h"
#include "InoHelper.h"
#include "Kinematics_ARG.h"
#include "MaxSonarRangeFinder.h"
#include "MotorControl.h"
#include "Motors.h"
#include "PID.h"
#include "PrintDrone.h"
#include "Receiver_MEGA.h"
#include "ReceiveCommandTestData.h"
#include "SensorsStatus.h"
#include "SerialCom.h"
#include "UserConfiguration.h"

#define LED_Green 13
#define LED_Red 4
#define LED_Yellow 31

// Gyroscope declaration
#define ITG3200_ADDRESS_ALTERNATE
#define RECEIVER_MEGA
#define MOTOR_PWM_Timer
#define BMP085
#define XLMAXSONAR 

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
void measureCriticalSensors() 
{
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
	SERIAL_BEGIN(BAUD);
	pinMode(LED_Green, OUTPUT);
	digitalWrite(LED_Green, LOW);

	SetupPrintDrone(); //Serial output

	printNewLine("Initialing drone", STATUSMODE);
	readEEPROM(); // defined in DataStorage.h
	bool firstTimeBoot = false;

	if (readFloat(SOFTWARE_VERSION_ADR) != SOFTWARE_VERSION) 
	{ // If we detect the wrong soft version, we init all parameters
		printNewLine("Recalibrating the system - ERROR", STATUSMODE);
		initializeEEPROM();
		writeEEPROM();
		firstTimeBoot = true;
	}

	initPlatform();
	initializeMotors(FOUR_Motors);

	printNewLine("Initializing EEPROM", STATUSMODE);
	initReceiverFromEEPROM();

	printNewLine("Initialing ControlFaker", STATUSMODE);
	SetupControlFaker();

	printNewLine("Initializing Receiver", STATUSMODE);
	initializeReceiver(LASTCHANNEL);

	// Initialize sensors
	// If sensors have a common initialization routine
	// insert it into the gyro class because it executes first
	printNewLine("Initializing Gyro", STATUSMODE);
	initializeGyro(); // defined in Gyro.h
	while (!calibrateGyro()); // this make sure the craft is still befor to continue init process

	printNewLine("Initializing Accelerometer", STATUSMODE);
	initializeAccel(); // defined in Accel.h

	if (firstTimeBoot) 
	{
		computeAccelBias();
		writeEEPROM();
	}

	printNewLine("Initializing FourthOrder", STATUSMODE);
	setupFourthOrder();
	initSensorsZeroFromEEPROM();

	// Integral Limit for attitude mode
	// This overrides default set in readEEPROM()
	// Set for 1/2 max attitude command (+/-0.75 radians)
	// Rate integral not used for now
	PID[ATTITUDE_XAXIS_PID_IDX].windupGuard = 0.375;
	PID[ATTITUDE_YAXIS_PID_IDX].windupGuard = 0.375;

	// Flight angle estimation
	printNewLine("Initializing Kinematics", STATUSMODE);
	initializeKinematics();

	// Optional Sensors
	printNewLine("Initializing Barometer", STATUSMODE);
	initializeBaro();
	vehicleState |= ALTITUDEHOLD_ENABLED;

	printNewLine("Initializing RangeFinder", STATUSMODE);
	inititalizeRangeFinders();

	vehicleState |= RANGE_ENABLED;
	PID[SONAR_ALTITUDE_HOLD_PID_IDX].P = PID[BARO_ALTITUDE_HOLD_PID_IDX].P*2;
	PID[SONAR_ALTITUDE_HOLD_PID_IDX].I = PID[BARO_ALTITUDE_HOLD_PID_IDX].I;
	PID[SONAR_ALTITUDE_HOLD_PID_IDX].D = PID[BARO_ALTITUDE_HOLD_PID_IDX].D;
	PID[SONAR_ALTITUDE_HOLD_PID_IDX].windupGuard = PID[BARO_ALTITUDE_HOLD_PID_IDX].windupGuard;

	//printNewLine("Setup ReceiveCommandTestData", STATUSMODE);
	//ResetReceiveCommandTestData();

	previousTime = micros();
	digitalWrite(LED_Green, HIGH);
	safetyCheck = 0;

	//KillMotor(true);
}

/*******************************************************************
* Main loop funtions
******************************************************************/
void loop () 
{
	currentTime = micros();
	deltaTime = currentTime - previousTime;

	measureCriticalSensors();

	// 100Hz task loop
	if (deltaTime >= 10000) 
	{
		frameCounter++;

		//if(miliSecCounterActive)
		//	miliSecCounter += 10; //ReceiveCommandTestData.h

		process100HzTask();

		// 50Hz task loop
		if (frameCounter % TASK_50HZ == 0) 
			process50HzTask();

		// 10Hz task loop
		if (frameCounter % TASK_10HZ == 0) 
			process10HzTask1();//Not used

		else if ((currentTime - lowPriorityTenHZpreviousTime) > 100000) 
			process10HzTask2();

		else if ((currentTime - lowPriorityTenHZpreviousTime2) > 100000) 
			process10HzTask3();
		

		//2Hz Task
		if(frameCounter % TASK_2HZ == 0)
			process2HzTask();

		// 1Hz task loop
		if (frameCounter % TASK_1HZ == 0) 
			process1HzTask(); //Print reports

		previousTime = currentTime;
	}

	if (frameCounter >= 100) 
		frameCounter = 0;
}
