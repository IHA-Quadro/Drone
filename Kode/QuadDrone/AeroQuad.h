#ifndef AeroQuad_H
#define AeroQuad_H

#include <Arduino.h>
#include <math.h>
#include <pins_arduino.h>
#include <stdlib.h>

#include "Accelerometer.h"
#include "AQMath.h"
#include "GlobalDefined.h"

extern byte calibrateESC;
extern int testCommand;
//////////////////////////////////////////////////////

/**
* Flight control global declaration
*/
#define RATE_FLIGHT_MODE 0
#define ATTITUDE_FLIGHT_MODE 1
#define TASK_100HZ 1
#define TASK_50HZ 2
#define TASK_10HZ 10
#define TASK_1HZ 100
#define THROTTLE_ADJUST_TASK_SPEED TASK_50HZ

#define OFF 0

// Flight Software Version
#define SOFTWARE_VERSION 3.2
#define BAUD 115200

/**
* Serial communication global declaration
*/
#define SERIAL_PRINT      SERIAL_PORT.print
#define SERIAL_PRINTLN    SERIAL_PORT.println
#define SERIAL_AVAILABLE  SERIAL_PORT.available
#define SERIAL_READ       SERIAL_PORT.read
#define SERIAL_FLUSH      SERIAL_PORT.flush
#define SERIAL_BEGIN      SERIAL_PORT.begin

extern byte previousFlightMode;

extern byte flightMode;
extern unsigned long frameCounter; // main loop executive frame counter
extern int minArmedThrottle; // initial value configured by user

extern float G_Dt; 
extern int throttle;
extern byte motorArmed;
extern byte safetyCheck;
extern byte maxLimit;
extern byte minLimit;
extern float filteredAccel[3];
extern boolean inFlight; // true when motor are armed and that the user pass one time the min throttle
extern float rotationSpeedFactor;

// main loop time variable
extern unsigned long previousTime;
extern unsigned long currentTime;
extern unsigned long deltaTime;

// sub loop time variable
extern unsigned long oneHZpreviousTime;
extern unsigned long tenHZpreviousTime;
extern unsigned long lowPriorityTenHZpreviousTime;
extern unsigned long lowPriorityTenHZpreviousTime2;
extern unsigned long fiftyHZpreviousTime;
extern unsigned long hundredHZpreviousTime;


// Analog Reference Value
// This value provided from Configurator
// Use a DMM to measure the voltage between AREF and GND
// Enter the measured voltage below to define your value for aref
extern float aref; // Read in from EEPROM
//TODO: This value is never initialized

/**
* Heading and heading hold global declaration section
*/

extern byte  headingHoldConfig;
extern float headingHold; // calculated adjustment for quad to go to heading (PID output)
extern float heading; // measured heading from yaw gyro (process variable)
extern float relativeHeading; // current heading the quad is set to (set point)
extern byte  headingHoldState;
void  processHeading();

//HardwareSerial *binaryPort;

void InitializeAeroQuad();

void readSerialCommand();
void sendSerialTelemetry();
void printInt(int data);
float readFloatSerial();
long readIntegerSerial();
void sendBinaryFloat(float);
void sendBinaryuslong(unsigned long);
void fastTelemetry();
void comma();
//void reportVehicleState();

/**
* Altitude control global declaration
*/
// special state that allows immediate turn off of Altitude hold if large throttle changesa are made at the TX
extern byte altitudeHoldState;  // ON, OFF or ALTPANIC
extern int altitudeHoldBump;
extern int altitudeHoldPanicStickMovement;
extern int minThrottleAdjust;
extern int maxThrottleAdjust;
extern int altitudeHoldThrottle;
extern boolean isAltitudeHoldInitialized;


extern float velocityCompFilter1;
extern float velocityCompFilter2;

extern boolean runtimaZBiasInitialized;  
extern float zVelocity;
extern float estimatedZVelocity;
extern float runtimeZBias; 
extern float zDampeningThrottleCorrection;

extern float baroAltitudeToHoldTarget;
extern float sonarAltitudeToHoldTarget;

/**
* Auto landing feature variables
*/
#define BARO_AUTO_DESCENT_STATE 2
#define SONAR_AUTO_DESCENT_STATE 3
#define MOTOR_AUTO_DESCENT_STATE 4

extern byte autoLandingState;
extern boolean isAutoLandingInitialized;
extern int autoLandingThrottleCorrection;

/**
* EEPROM global section
*/

typedef struct {
	float p;
	float i;
	float d;
} t_NVR_PID;

typedef struct {
	float slope;
	float offset;
	float smooth_factor;
} t_NVR_Receiver;


typedef struct {    
	t_NVR_PID ROLL_PID_GAIN_ADR;
	t_NVR_PID LEVELROLL_PID_GAIN_ADR;
	t_NVR_PID YAW_PID_GAIN_ADR;
	t_NVR_PID PITCH_PID_GAIN_ADR;
	t_NVR_PID LEVELPITCH_PID_GAIN_ADR;
	t_NVR_PID HEADING_PID_GAIN_ADR;
	t_NVR_PID LEVEL_GYRO_ROLL_PID_GAIN_ADR;
	t_NVR_PID LEVEL_GYRO_PITCH_PID_GAIN_ADR;
	t_NVR_PID ALTITUDE_PID_GAIN_ADR;
	t_NVR_PID ZDAMP_PID_GAIN_ADR;
	t_NVR_PID GPSROLL_PID_GAIN_ADR;
	t_NVR_PID GPSPITCH_PID_GAIN_ADR;
	t_NVR_PID GPSYAW_PID_GAIN_ADR;
	t_NVR_Receiver RECEIVER_DATA[10]; //MAX_NB_CHANNEL

	float SOFTWARE_VERSION_ADR;
	float WINDUPGUARD_ADR;
	float XMITFACTOR_ADR;
	float MINARMEDTHROTTLE_ADR;
	float AREF_ADR;
	float FLIGHTMODE_ADR;
	float HEADINGHOLD_ADR;
	float ACCEL_1G_ADR;
	float ALTITUDE_MAX_THROTTLE_ADR;
	float ALTITUDE_MIN_THROTTLE_ADR;
	float ALTITUDE_SMOOTH_ADR;
	float ALTITUDE_WINDUP_ADR;
	float ALTITUDE_BUMP_ADR;
	float ALTITUDE_PANIC_ADR;

	// Gyro calibration
	float ROTATION_SPEED_FACTOR_ARD;

	// Accel Calibration
	float XAXIS_ACCEL_BIAS_ADR;
	float XAXIS_ACCEL_SCALE_FACTOR_ADR;
	float YAXIS_ACCEL_BIAS_ADR;
	float YAXIS_ACCEL_SCALE_FACTOR_ADR;
	float ZAXIS_ACCEL_BIAS_ADR;
	float ZAXIS_ACCEL_SCALE_FACTOR_ADR;

	// Mag Calibration
	float XAXIS_MAG_BIAS_ADR;
	float YAXIS_MAG_BIAS_ADR;
	float ZAXIS_MAG_BIAS_ADR;

	// Battery Monitor
	float BATT_ALARM_VOLTAGE_ADR;
	float BATT_THROTTLE_TARGET_ADR;
	float BATT_DOWN_TIME_ADR;

	// Range Finder
	float RANGE_FINDER_MAX_ADR;
	float RANGE_FINDER_MIN_ADR;

	// Camera Control
	float CAMERAMODE_ADR;
	float MCAMERAPITCH_ADR;
	float MCAMERAROLL_ADR;    
	float MCAMERAYAW_ADR;
	float SERVOCENTERPITCH_ADR;
	float SERVOCENTERROLL_ADR;
	float SERVOCENTERYAW_ADR;
	float SERVOMINPITCH_ADR;
	float SERVOMINROLL_ADR;
	float SERVOMINYAW_ADR;
	float SERVOMAXPITCH_ADR;
	float SERVOMAXROLL_ADR;
	float SERVOMAXYAW_ADR;
	float SERVOTXCHANNELS_ADR;

	// GPS mission storing
	float GPS_MISSION_NB_POINT_ADR;
} t_NVR_Data;

void readEEPROM(); 
void initSensorsZeroFromEEPROM();
void storeSensorsZeroToEEPROM();
void initReceiverFromEEPROM();

void initializePlatformSpecificAccelCalibration() ;

float nvrReadFloat(int address); // defined in DataStorage.h
void nvrWriteFloat(float value, int address); // defined in DataStorage.h
long nvrReadLong(int address); // defined in DataStorage.h
void nvrWriteLong(long value, int address); // defined in DataStorage.h
void nvrReadPID(unsigned char IDPid, unsigned int IDEeprom);
void nvrWritePID(unsigned char IDPid, unsigned int IDEeprom);

#define GET_NVR_OFFSET(param) ((int)&(((t_NVR_Data*) 0)->param))
#define readFloat(addr) nvrReadFloat(GET_NVR_OFFSET(addr))
#define writeFloat(value, addr) nvrWriteFloat(value, GET_NVR_OFFSET(addr))
#define readLong(addr) nvrReadLong(GET_NVR_OFFSET(addr))
#define writeLong(value, addr) nvrWriteLong(value, GET_NVR_OFFSET(addr))
#define readPID(IDPid, addr) nvrReadPID(IDPid, GET_NVR_OFFSET(addr))
#define writePID(IDPid, addr) nvrWritePID(IDPid, GET_NVR_OFFSET(addr))

/**
* Debug utility global declaration
* Debug code should never be part of a release sofware
* @see Kenny
*/

#define DEBUG
extern byte fastTransfer; // Used for troubleshooting

#endif
