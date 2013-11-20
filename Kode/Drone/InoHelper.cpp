#include "InoHelper.h"

#include "Accelerometer.h"
#include "AltitudeControlProcessor.h"
#include "AeroQuad.h"
#include "Decision.h"
#include "FlightCommandProcessor.h"
#include "FlightControlProcessor.h"
#include "FourtOrderFilter.h"
#include "MaxSonarRangeFinder.h"
#include "UserConfiguration.h"


// called when eeprom is initialized
void initializePlatformSpecificAccelCalibration() 
{
	// Kenny default value, a real accel calibration is strongly recommended
	accelScaleFactor[XAXIS] = 0.0365570020;
	accelScaleFactor[YAXIS] = 0.0363000011;
	accelScaleFactor[ZAXIS] = -0.0384629964;
}

void process100HzTask() 
{
	G_Dt = (currentTime - hundredHZpreviousTime) / 1000000.0;
	hundredHZpreviousTime = currentTime;

	evaluateGyroRate();
	evaluateMetersPerSec();

	for (int axis = XAXIS; axis <= ZAXIS; axis++) 
	{
		filteredAccel[axis] = computeFourthOrder(meterPerSecSec[axis], &fourthOrder[axis]);
	}

	calculateKinematics(gyroRate[XAXIS], gyroRate[YAXIS], gyroRate[ZAXIS], filteredAccel[XAXIS], filteredAccel[YAXIS], filteredAccel[ZAXIS], G_Dt);

	//#if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
	zVelocity = (filteredAccel[ZAXIS] * (1 - accelOneG * invSqrt(isq(filteredAccel[XAXIS]) + isq(filteredAccel[YAXIS]) + isq(filteredAccel[ZAXIS])))) - runTimeAccelBias[ZAXIS] - runtimeZBias;

	if (!runtimaZBiasInitialized) 
	{
		runtimeZBias = (filteredAccel[ZAXIS] * (1 - accelOneG * invSqrt(isq(filteredAccel[XAXIS]) + isq(filteredAccel[YAXIS]) + isq(filteredAccel[ZAXIS])))) - runTimeAccelBias[ZAXIS];
		runtimaZBiasInitialized = true;
	}

	estimatedZVelocity += zVelocity;
	estimatedZVelocity = (velocityCompFilter1 * zVelocity) + (velocityCompFilter2 * estimatedZVelocity);
	//#endif    
	//#if defined(AltitudeHoldBaro)
	measureBaroSum(); 

	if (frameCounter % THROTTLE_ADJUST_TASK_SPEED == 0)   //  50 Hz tasks
		evaluateBaroAltitude();
	//#endif

	processFlightControl();

}

void process50HzTask() 
{
	G_Dt = (currentTime - fiftyHZpreviousTime) / 1000000.0;
	fiftyHZpreviousTime = currentTime;

	// Reads external pilot commands and performs functions based on stick configuration
	readPilotCommands(); 

	updateRangeFinders();  
}

void process10HzTask1() 
{
#if defined(HeadingMagHold)
	G_Dt = (currentTime - tenHZpreviousTime) / 1000000.0;
	tenHZpreviousTime = currentTime;

	measureMagnetometer(kinematicsAngle[XAXIS], kinematicsAngle[YAXIS]);

	calculateHeading();
#endif
}

//low priority 10Hz task 2
void process10HzTask2() 
{
	G_Dt = (currentTime - lowPriorityTenHZpreviousTime) / 1000000.0;
	lowPriorityTenHZpreviousTime = currentTime;

	// Listen for configuration commands and reports telemetry
	readSerialCommand();
	sendSerialTelemetry();
}

//low priority 10Hz task 3
void process10HzTask3() {
	G_Dt = (currentTime - lowPriorityTenHZpreviousTime2) / 1000000.0;
	lowPriorityTenHZpreviousTime2 = currentTime;
}

//2 Hz process
void process2HzTask()
{
	//PrintSonarReport();
	//PrintChosenProgram();
	//PrintControllerOutput();
	//PrintWarnings();
}

//1Hz process
void process1HzTask() {
	//#ifdef MavLink
	//	G_Dt = (currentTime - oneHZpreviousTime) / 1000000.0;
	//	oneHZpreviousTime = currentTime;
	//
	//	sendSerialHeartbeat();   
	//#endif

	PrintAltitudeReport();
}


static void PrintChosenProgram()
{
	printNewLine(GetRadioProgram(), RADIOMODE);
}

static void PrintSonarReport()
{
	int sonarHeight = (int)(RangerAverage[ALTITUDE_RANGE_FINDER_INDEX].average *100) + 8; //Bottom sonar
	printInLine(sonarHeight, SONARMODE);
	printInLine(" - ", SONARMODE);
	printInLine((_controllerInput[AUX1] == ALTITUDEHOLDFALSE ? "Free flight" : "Fixed flight"), SONARMODE);
	printInLine("<=>", SONARMODE);
	printNewLine(programInput.height, SONARMODE);

	printInLine( RangerAverage[LEFT_RANGE_FINDER_INDEX].average, SONARMODE);
	printInLine(" - ", SONARMODE);
	printInLine( RangerAverage[FRONT_RANGE_FINDER_INDEX].average, SONARMODE);
	printInLine(" - ", SONARMODE);
	printNewLine( RangerAverage[RIGHT_RANGE_FINDER_INDEX].average, SONARMODE);
}

static void PrintAltitudeReport()
{
	printInLine("AltitudeHoldState activate: ", ALTITUDEMODE);
	printNewLine((altitudeHoldState == true ? "true" : "false"), ALTITUDEMODE);

	if(receiverCommand[AUX3] < 1750)
	{
		printInLine("Sensor found: ", ALTITUDEMODE);
		printNewLine((isOnRangerRangeValid?"Yes":"No"), ALTITUDEMODE);
	}


	printInLine("Hold position: ", ALTITUDEMODE);
	printNewLine((Holdposition?"Yes":"No"), ALTITUDEMODE);

	if(receiverCommand[AUX1] < 1750)
	{
		printInLine("Panic mode: ", ALTITUDEMODE);
		printNewLine((panic?"Yes":"No"), ALTITUDEMODE);
	}
}

static void PrintDebugReport()
{
	printInLine("Armed: ", DEBUGMODE);
	printNewLine((motorArmed == ON ? "On" : "Off"), STATUSMODE);
}

static void PrintWarnings()
{
	bool print = (GetLeftWarning() || GetRightWarning() || GetFrontWarning());

	if(print)
	{
		printInLine("Warning: ", WARNINGMODE);
		printInLine(GetLeftWarning() ? ": Left" : "", WARNINGMODE);
		printInLine(GetFrontWarning() ? ": Center" : "", WARNINGMODE);
		printInLine(GetRightWarning() ? ": Right" : "", WARNINGMODE);

		printNewLine("", WARNINGMODE);
	}
}

static void PrintControllerOutput()
{
	printInLine("Parameters: ", MOTORMODE);
	printInLine(_controllerInput[XAXIS], MOTORMODE);
	printInLine(" - ", MOTORMODE);
	printInLine(_controllerInput[YAXIS], MOTORMODE);
	printInLine(" - ", MOTORMODE);
	printInLine(_controllerInput[ZAXIS], MOTORMODE);
	printInLine(" - ", MOTORMODE);
	printNewLine(_controllerInput[THROTTLE], MOTORMODE);
	printInLine(" - ", MOTORMODE);
	printNewLine((_controllerInput[AUX1] == ALTITUDEHOLDFALSE ? "True" : "False") , MOTORMODE);
}
