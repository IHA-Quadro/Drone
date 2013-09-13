/****************************************************************************
Before flight, select the different user options for your AeroQuad by
editing UserConfiguration.h.

If you need additional assistance go to http://www.aeroquad.com/forum.php
or talk to us live on IRC #aeroquad
*****************************************************************************/

#include <EEPROM.h>
#include <Wire.h>

#include "AeroQuad.h"
#include "AQMath.h"
#include "ControlFaker.h"
#include "FourtOrderFilter.h"
#include "GlobalDefined.h"
#include "InoHelper.h"
#include "MaxSonarRangeFinder.h"
#include "PID.h"
#include "PrintDrone.h"
#include "ReceiveCommandTestData.h"
#include "UserConfiguration.h" // Edit this file first before uploading to the AeroQuad

void initializePlatformSpecificAccelCalibration();

#ifdef BattMonitor
#include <BatteryMonitorTypes.h>
#endif

//
// Define Security Checks
//

#if defined(UseGPSNMEA) || defined(UseGPSUBLOX) || defined(UseGPSMTK) || defined(UseGPS406)
#define UseGPS
#endif 

#if defined(UseGPSNavigator) && !defined(AltitudeHoldBaro)
#error "GpsNavigation NEED AltitudeHoldBaro defined"
#endif

#if defined(AutoLanding) && (!defined(AltitudeHoldBaro) || !defined(AltitudeHoldRangeFinder))
#error "AutoLanding NEED AltitudeHoldBaro and AltitudeHoldRangeFinder defined"
#endif

#if defined(ReceiverSBUS) && defined(SlowTelemetry)
#error "Receiver SWBUS and SlowTelemetry are in conflict for Seria2, they can't be used together"
#endif

#if defined (CameraTXControl) && !defined (CameraControl)
#error "CameraTXControl need to have CameraControl defined"
#endif 


//********************************************************
//********************************************************
//********* PLATFORM SPECIFIC SECTION ********************
//********************************************************
//********************************************************

#ifdef AeroQuadMega_v21

#include "Device_I2C.h"

// Gyroscope declaration
#define ITG3200_ADDRESS_ALTERNATE
#include "Gyroscope.h"

// Accelerometer declaration
#include "Accelerometer.h"

// Receiver Declaration
#define RECEIVER_MEGA

// Motor declaration
#define MOTOR_PWM_Timer

// heading mag hold declaration
#ifdef HeadingMagHold
#include <Compass.h>
#define SPARKFUN_9DOF_5883L
#endif

// Altitude declaration
#ifdef AltitudeHoldBaro
#define BMP085
#endif
#ifdef AltitudeHoldRangeFinder
#define XLMAXSONAR 
#endif


// Battery Monitor declaration
#ifdef BattMonitor
#ifdef POWERED_BY_VIN
#define BattDefaultConfig DEFINE_BATTERY(0, 0, 15.0, 0, BM_NOPIN, 0, 0) // v2 shield powered via VIN (no diode)
#else
#define BattDefaultConfig DEFINE_BATTERY(0, 0, 15.0, 0.82, BM_NOPIN, 0, 0) // v2 shield powered via power jack
#endif
#else
#undef BattMonitorAutoDescent
#undef POWERED_BY_VIN        
#endif

#ifdef OSD
#define MAX7456_OSD
#endif  

#ifndef UseGPS
#undef UseGPSNavigator
#endif


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
#endif


//********************************************************
//********************************************************
//********* HARDWARE GENERALIZATION SECTION **************
//********************************************************
//********************************************************

#ifdef AeroQuadSTM32
#include "AeroQuad_STM32.h"
#endif



//********************************************************
//****************** KINEMATICS DECLARATION **************
//********************************************************
#include "Kinematics.h"
#if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
// CHR6DM have it's own kinematics, so, initialize in it's scope
#else
#include "Kinematics_ARG.h"
#endif

//********************************************************
//******************** RECEIVER DECLARATION **************
//********************************************************
#if defined(ReceiverHWPPM)
#include <Receiver_HWPPM.h>
#elif defined(ReceiverPPM)
#include <Receiver_PPM.h>
#elif defined(AeroQuad_Mini) && (defined(hexPlusConfig) || defined(hexXConfig) || defined(hexY6Config))
#include <Receiver_PPM.h>
#elif defined(RemotePCReceiver)
#include <Receiver_RemotePC.h>
#elif defined(ReceiverSBUS)
#include <Receiver_SBUS.h>
#elif defined(RECEIVER_328P)
#include <Receiver_328p.h>
#elif defined(RECEIVER_MEGA)
#include "Receiver_MEGA.h"
#elif defined(RECEIVER_APM)
#include <Receiver_APM.h>
#elif defined(RECEIVER_STM32PPM)
#include <Receiver_STM32PPM.h>  
#elif defined(RECEIVER_STM32)
#include <Receiver_STM32.h>  
#endif

#if defined(UseAnalogRSSIReader) 
#include <AnalogRSSIReader.h>
#elif defined(UseEzUHFRSSIReader)
#include <EzUHFRSSIReader.h>
#elif defined(UseSBUSRSSIReader)
#include <SBUSRSSIReader.h>
#endif



//********************************************************
//********************** MOTORS DECLARATION **************
//********************************************************
#if defined(triConfig)
#if defined (MOTOR_STM32)
#define MOTORS_STM32_TRI
#include <Motors_STM32.h>    
#else
#include <Motors_Tri.h>
#endif
#elif defined(MOTOR_PWM)
#include <Motors_PWM.h>
#elif defined(MOTOR_PWM_Timer)
#include "Motors_PWM_Timer.h"
#elif defined(MOTOR_APM)
#include <Motors_APM.h>
#elif defined(MOTOR_I2C)
#include <Motors_I2C.h>
#elif defined(MOTOR_STM32)
#include <Motors_STM32.h>    
#endif

//********************************************************
//******* HEADING HOLD MAGNETOMETER DECLARATION **********
//********************************************************
#if defined(HMC5843)
#include <HeadingFusionProcessorMARG.h>
#include <Magnetometer_HMC5843.h>
#elif defined(SPARKFUN_9DOF_5883L) || defined(SPARKFUN_5883L_BOB) || defined(HMC5883L)
#include <HeadingFusionProcessorMARG.h>
#include <Magnetometer_HMC5883L.h>
#elif defined(COMPASS_CHR6DM)
#endif

//********************************************************
//******* ALTITUDE HOLD BAROMETER DECLARATION ************
//********************************************************
#if defined(BMP085)
#include "BarometricSensor.h"
#elif defined(MS5611)
#include <BarometricSensor_MS5611.h>
#endif
#if defined(XLMAXSONAR)
#include <MaxSonarRangeFinder.h> //TODO:Denne skal findes
#endif 
//********************************************************
//*************** BATTERY MONITOR DECLARATION ************
//********************************************************
#ifdef BattMonitor
#include <BatteryMonitor.h>
#ifndef BattCustomConfig
#define BattCustomConfig BattDefaultConfig
#endif
struct BatteryData batteryData[] = {BattCustomConfig};
#endif
//********************************************************
//************** CAMERA CONTROL DECLARATION **************
//********************************************************
// used only on mega for now
#if defined(CameraControl_STM32)
#include <CameraStabilizer_STM32.h>
#elif defined(CameraControl)
#include <CameraStabilizer_Aeroquad.h>
#endif

#if defined (CameraTXControl)
#include <CameraStabilizer_TXControl.h>
#endif

//********************************************************
//******** FLIGHT CONFIGURATION DECLARATION **************
//********************************************************
#if defined(quadXConfig)
#include "FlightControlQuadX.h"
#elif defined(quadPlusConfig)
#include "FlightControlQuadPlus.h"
#elif defined(hexPlusConfig)
#include "FlightControlHexPlus.h"
#elif defined(hexXConfig)
#include "FlightControlHexX.h"
#elif defined(triConfig)
#include "FlightControlTri.h"
#elif defined(quadY4Config)
#include "FlightControlQuadY4.h"
#elif defined(hexY6Config)
#include "FlightControlHexY6.h"
#elif defined(octoX8Config)
#include "FlightControlOctoX8.h"
#elif defined(octoXConfig)
#include "FlightControlOctoX.h"
#elif defined(octoPlusConfig)
#include "FlightControlOctoPlus.h"
#endif

//********************************************************
//****************** GPS DECLARATION *********************
//********************************************************
#if defined(UseGPS)

#if !defined(HeadingMagHold)
#error We need the magnetometer to use the GPS
#endif 

#include "GpsAdapter.h"
#include "GpsNavigator.h"
#endif

//********************************************************
//****************** OSD DEVICE DECLARATION ************** Onboard Screen Display
//********************************************************
#ifdef MAX7456_OSD     // only OSD supported for now is the MAX7456
#include <Device_SPI.h>
#include "OSDDisplayController.h"
#include "MAX7456.h"
#endif

#if defined(SERIAL_LCD)
#include "SerialLCD.h"
#endif

#ifdef OSD_SYSTEM_MENU
#if !defined(MAX7456_OSD) && !defined(SERIAL_LCD)
#error "Menu cannot be used without OSD or LCD"
#endif
#include "OSDMenu.h"
#endif


 

#ifdef SlowTelemetry
#include <AQ_RSCode.h>
#endif

#ifdef SoftModem
#include <AQ_SoftModem.h>
#endif


// Include this last as it contains objects from above declarations
#include "AltitudeControlProcessor.h"
#include "FlightControlProcessor.h"
#include "FlightCommandProcessor.h"
#include "HeadingHoldProcessor.h"
#include "DataStorage.h"

#if defined(UseGPS) || defined(BattMonitor)
#include "LedStatusProcessor.h"
#endif  

#if defined(MavLink)
#include "MavLink.h"
#else
#include "SerialCom.h"
#include "GlobalDefined.h"
#endif



/*******************************************************************
* Main setup function, called one time at bootup
* initialize all system and sub system of the
* Aeroquad
******************************************************************/
void setup() {
	printDebug("Starting setup of drone");
	printDebug("Initializing HeadingHold");
	InitializeHeadingHoldProcessor();

	SERIAL_BEGIN(BAUD);
	pinMode(LED_Green, OUTPUT);
	digitalWrite(LED_Green, LOW);

	initCommunication();

	readEEPROM(); // defined in DataStorage.h
	boolean firstTimeBoot = false;
	if (readFloat(SOFTWARE_VERSION_ADR) != SOFTWARE_VERSION) { // If we detect the wrong soft version, we init all parameters
		initializeEEPROM();
		writeEEPROM();
		firstTimeBoot = true;
	}

	initPlatform();

#if defined(quadXConfig) || defined(quadPlusConfig) || defined(quadY4Config) || defined(triConfig)
	initializeMotors(FOUR_Motors);
#elif defined(hexPlusConfig) || defined(hexXConfig) || defined(hexY6Config)
	initializeMotors(SIX_Motors);
#elif defined(octoX8Config) || defined(octoXConfig) || defined(octoPlusConfig)
	initializeMotors(EIGHT_Motors);
#endif

	printDebug("Initializing Receiver");
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

#ifdef HeadingMagHold
	vehicleState |= HEADINGHOLD_ENABLED;
	initializeMagnetometer();
	initializeHeadingFusion();
#endif

	// Optional Sensors
#ifdef AltitudeHoldBaro
	initializeBaro();
	vehicleState |= ALTITUDEHOLD_ENABLED;
#endif
#ifdef AltitudeHoldRangeFinder
	RangeFinderAssign();
#endif

#ifdef BattMonitor
	initializeBatteryMonitor(sizeof(batteryData) / sizeof(struct BatteryData), batteryMonitorAlarmVoltage);
	vehicleState |= BATTMONITOR_ENABLED;
#endif

#if defined(CameraControl)
	initializeCameraStabilization();
	vehicleState |= CAMERASTABLE_ENABLED;
#endif

#if defined(MAX7456_OSD)
	initializeSPI();
	initializeOSD();
#endif

#if defined(SERIAL_LCD)
	InitSerialLCD();
#endif

#if defined(BinaryWrite) || defined(BinaryWritePID)
#ifdef OpenlogBinaryWrite
	binaryPort = &Serial1;
	binaryPort->begin(115200);
	delay(1000);
#else
	binaryPort = &Serial;
#endif
#endif

#if defined(UseGPS)
	initializeGps();
#endif 

#ifdef SlowTelemetry
	initSlowTelemetry();
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
		}

		previousTime = currentTime;
	}

	if (frameCounter >= 100) 
		frameCounter = 0;
}