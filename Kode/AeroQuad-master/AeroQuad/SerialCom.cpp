#include "SerialCom.h"

char queryType = 'X';

void initCommunication() {
  // do nothing here for now
}

//***************************************************************************************************
//********************************** Serial Commands ************************************************
//***************************************************************************************************
bool validateCalibrateCommand(byte command)
{
  if (readFloatSerial() == 123.45) {// use a specific float value to validate full throttle call is being sent
    motorArmed = OFF;
    calibrateESC = command;
    return true;
  }
  else {
    calibrateESC = 0;
    testCommand = 1000;
    return false;
  }
}

void readSerialPID(unsigned char PIDid) {
  struct PIDdata* pid = &PID[PIDid];
  pid->P = readFloatSerial();
  pid->I = readFloatSerial();
  pid->D = readFloatSerial();
  pid->lastError = 0;
  pid->integratedError = 0;
}

void skipSerialValues(byte number) {
  for(byte i=0; i<number; i++) {
    readFloatSerial();
  }
}

void InvertUART()
{
	PrintConfig[STATUSMODE] = !PrintConfig[STATUSMODE];
	PrintConfig[DEBUGMODE] = !PrintConfig[DEBUGMODE];
}

void StopMotors()
{
	bool previousState = getMotorStatus();

	setMotorStatus(previousState);

	if(previousState == false)
	{
		printStatus("Motor stopped!");
		InvertUART();
	}
	else
	{
		ResetInputData();
		InvertUART();
		printStatus("Motor started!");
	}
}

void readSerialCommand() {
  // Check for serial message
  if (SERIAL_AVAILABLE()) {
    queryType = SERIAL_READ();
    switch (queryType) {
    case 'A': // Receive roll and pitch rate mode PID
      readSerialPID(RATE_XAXIS_PID_IDX);
      readSerialPID(RATE_YAXIS_PID_IDX);
      rotationSpeedFactor = readFloatSerial();
      break;

    case 'B': // Receive roll/pitch attitude mode PID
      readSerialPID(ATTITUDE_XAXIS_PID_IDX);
      readSerialPID(ATTITUDE_YAXIS_PID_IDX);
      readSerialPID(ATTITUDE_GYRO_XAXIS_PID_IDX);
      readSerialPID(ATTITUDE_GYRO_YAXIS_PID_IDX);
      windupGuard = readFloatSerial(); // defaults found in setup() of AeroQuad.pde
      break;

    case 'C': // Receive yaw PID
      readSerialPID(ZAXIS_PID_IDX);
      readSerialPID(HEADING_HOLD_PID_IDX);
      headingHoldConfig = readFloatSerial();
      break;

    case 'D': // Altitude hold PID
      #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
        readSerialPID(BARO_ALTITUDE_HOLD_PID_IDX);
        PID[BARO_ALTITUDE_HOLD_PID_IDX].windupGuard = readFloatSerial();
        altitudeHoldBump = readFloatSerial();
        altitudeHoldPanicStickMovement = readFloatSerial();
        minThrottleAdjust = readFloatSerial();
        maxThrottleAdjust = readFloatSerial();
        #if defined AltitudeHoldBaro
          baroSmoothFactor = readFloatSerial();
        #else
          readFloatSerial();
        #endif
        readSerialPID(ZDAMPENING_PID_IDX);
      #endif
      break;

    case 'E': // Receive sensor filtering values
      aref = readFloatSerial();
      minArmedThrottle = readFloatSerial();
      break;

    case 'F': // Receive transmitter smoothing values
      receiverXmitFactor = readFloatSerial();
      for(byte channel = XAXIS; channel<LASTCHANNEL; channel++) {
        receiverSmoothFactor[channel] = readFloatSerial();
      }
      break;

    case 'G': // Receive transmitter calibration values
      channelCal = (int)readFloatSerial();
      receiverSlope[channelCal] = readFloatSerial();
      break;

    case 'H': // Receive transmitter calibration values
      channelCal = (int)readFloatSerial();
      receiverOffset[channelCal] = readFloatSerial();
      break;

    case 'I': // Initialize EEPROM with default values
      initializeEEPROM(); // defined in DataStorage.h
      writeEEPROM();
      storeSensorsZeroToEEPROM();
      calibrateGyro();
      zeroIntegralError();
      #ifdef HeadingMagHold
        initializeMagnetometer();
      #endif
      #ifdef AltitudeHoldBaro
        initializeBaro();
      #endif
      break;

    case 'J': // calibrate gyros
      calibrateGyro();
      break;

    case 'K': // Write accel calibration values
      accelScaleFactor[XAXIS] = readFloatSerial();
      readFloatSerial();
      accelScaleFactor[YAXIS] = readFloatSerial();
      readFloatSerial();
      accelScaleFactor[ZAXIS] = readFloatSerial();
      readFloatSerial();
      computeAccelBias();    
      storeSensorsZeroToEEPROM();
      break;

    case 'L': // generate accel bias
      computeAccelBias();
      #if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
        calibrateKinematics();
        accelOneG = meterPerSecSec[ZAXIS];
      #endif
      storeSensorsZeroToEEPROM();
      break;

    case 'M': // calibrate magnetometer
      #ifdef HeadingMagHold
        magBias[XAXIS]  = readFloatSerial();
        magBias[YAXIS]  = readFloatSerial();
        magBias[ZAXIS]  = readFloatSerial();
        writeEEPROM();
      #else
        skipSerialValues(3);
      #endif
      break;

    case 'N': // battery monitor
      #ifdef BattMonitor
        batteryMonitorAlarmVoltage = readFloatSerial();
        batteryMonitorThrottleTarget = readFloatSerial();
        batteryMonitorGoingDownTime = readFloatSerial();
        setBatteryCellVoltageThreshold(batteryMonitorAlarmVoltage);
      #else
        skipSerialValues(3);
      #endif
      break;

    case 'O': // define waypoints
      #ifdef UseGPSNavigator
        missionNbPoint = readIntegerSerial();
        waypoint[missionNbPoint].latitude = readIntegerSerial();
        waypoint[missionNbPoint].longitude = readIntegerSerial();
        waypoint[missionNbPoint].altitude = readIntegerSerial();
      #else
        for(byte i = 0; i < 4; i++) {
          readFloatSerial();
        }
      #endif
      break;
    case 'P': //  read Camera values
      #ifdef CameraControl
        cameraMode = readFloatSerial();
        servoCenterPitch = readFloatSerial();
        servoCenterRoll = readFloatSerial();
        servoCenterYaw = readFloatSerial();
        mCameraPitch = readFloatSerial();
        mCameraRoll = readFloatSerial();
        mCameraYaw = readFloatSerial();
        servoMinPitch = readFloatSerial();
        servoMinRoll = readFloatSerial();
        servoMinYaw = readFloatSerial();
        servoMaxPitch = readFloatSerial();
        servoMaxRoll = readFloatSerial();
        servoMaxYaw = readFloatSerial();
        #ifdef CameraTXControl
          servoTXChannels = readFloatSerial();
        #endif
      #else
        #ifdef CameraTXControl
          skipSerialValues(14)
        #else
          skipSerialValues(13);
        #endif
      #endif
      break;

		case 'R': //Rotate
			RotateDrone( readFloatSerial());
			break;


		case 'S':
			StopMotors();
			break;

		case 'T':
			ThrottleDrone(readFloatSerial());
			break;

    case 'U': // Range Finder
      #if defined (AltitudeHoldRangeFinder)
        maxRangeFinderRange = readFloatSerial();
        minRangeFinderRange = readFloatSerial();
      #else
        skipSerialValues(2);
      #endif
      break;

    case 'V': // GPS
      #if defined (UseGPSNavigator)
        readSerialPID(GPSROLL_PID_IDX);
        readSerialPID(GPSPITCH_PID_IDX);
        readSerialPID(GPSYAW_PID_IDX);
        writeEEPROM();
      #else
        skipSerialValues(9);
      #endif
      break;

    case 'W': // Write all user configurable values to EEPROM
      writeEEPROM(); // defined in DataStorage.h
      zeroIntegralError();
      break;

    case 'X': // Stop sending messages
      break;

    case '1': // Calibrate ESCS's by setting Throttle high on all channels
      validateCalibrateCommand(1);
      break;

    case '2': // Calibrate ESC's by setting Throttle low on all channels
      validateCalibrateCommand(2);
      break;

    case '3': // Test ESC calibration
      if (validateCalibrateCommand(3)) {
        testCommand = readFloatSerial();
      }
      break;

    case '4': // Turn off ESC calibration
      if (validateCalibrateCommand(4)) {
        calibrateESC = 0;
        testCommand = 1000;
      }
      break;

    case '5': // Send individual motor commands (motor, command)
      if (validateCalibrateCommand(5)) {
        for (byte motor = 0; motor < LASTMOTOR; motor++) {
          motorConfiguratorCommand[motor] = (int)readFloatSerial();
        }
      }
      break;

    case 'Z': // fast telemetry transfer <--- get rid if this?
      if (readFloatSerial() == 1.0)
        fastTransfer = ON;
      else
        fastTransfer = OFF;
      break;
    }
  }
}

//***************************************************************************************************
//********************************* Serial Telemetry ************************************************
//***************************************************************************************************

void PrintValueComma(float val) {
  SERIAL_PRINT(val);
  comma();
}

void PrintValueComma(double val) {
  SERIAL_PRINT(val);
  comma();
}

void PrintValueComma(char val) {
  SERIAL_PRINT(val);
  comma();
}

void PrintValueComma(int val) {
  SERIAL_PRINT(val);
  comma();
}

void PrintValueComma(unsigned long val)
{
  SERIAL_PRINT(val);
  comma();
}

void PrintValueComma(byte val)
{
  SERIAL_PRINT(val);
  comma();
}

void PrintValueComma(long int val)
{
  SERIAL_PRINT(val);
  comma();
}

void PrintPID(unsigned char IDPid)
{
  PrintValueComma(PID[IDPid].P);
  PrintValueComma(PID[IDPid].I);
  PrintValueComma(PID[IDPid].D);
}

void PrintDummyValues(byte number) {
  for(byte i=0; i<number; i++) {
    PrintValueComma(0);
  }
}


float getHeading()
{
  #if defined(HeadingMagHold) || defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
    float heading = trueNorthHeading;
    if (heading < 0) { 
      heading += (2.0 * M_PI);
    }
    return heading;
  #else
    return(gyroHeading);
  #endif
}

void sendSerialTelemetry() {
  switch (queryType) {
  case '=': // Reserved debug command to view any variable from Serial Monitor
    break;

  case 'a': // Send roll and pitch rate mode PID values
    PrintPID(RATE_XAXIS_PID_IDX);
    PrintPID(RATE_YAXIS_PID_IDX);
    PrintValueComma(rotationSpeedFactor);
    SERIAL_PRINTLN();
    queryType = 'X';
    break;

  case 'b': // Send roll and pitch attitude mode PID values
    PrintPID(ATTITUDE_XAXIS_PID_IDX);
    PrintPID(ATTITUDE_YAXIS_PID_IDX);
    PrintPID(ATTITUDE_GYRO_XAXIS_PID_IDX);
    PrintPID(ATTITUDE_GYRO_YAXIS_PID_IDX);
    SERIAL_PRINTLN(windupGuard);
    queryType = 'X';
    break;

  case 'c': // Send yaw PID values
    PrintPID(ZAXIS_PID_IDX);
    PrintPID(HEADING_HOLD_PID_IDX);
    SERIAL_PRINTLN((int)headingHoldConfig);
    queryType = 'X';
    break;

  case 'd': // Altitude Hold
    #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
      PrintPID(BARO_ALTITUDE_HOLD_PID_IDX);
      PrintValueComma(PID[BARO_ALTITUDE_HOLD_PID_IDX].windupGuard);
      PrintValueComma(altitudeHoldBump);
      PrintValueComma(altitudeHoldPanicStickMovement);
      PrintValueComma(minThrottleAdjust);
      PrintValueComma(maxThrottleAdjust);
      #if defined AltitudeHoldBaro
        PrintValueComma(baroSmoothFactor);
      #else
        PrintValueComma(0);
      #endif
      PrintPID(ZDAMPENING_PID_IDX);
    #else
      PrintDummyValues(10);
    #endif
    SERIAL_PRINTLN();
    queryType = 'X';
    break;

  case 'e': // miscellaneous config values
    PrintValueComma(aref);
    SERIAL_PRINTLN(minArmedThrottle);
    queryType = 'X';
    break;

  case 'f': // Send transmitter smoothing values
    PrintValueComma(receiverXmitFactor);
    for (byte axis = XAXIS; axis < LASTCHANNEL; axis++) {
      PrintValueComma(receiverSmoothFactor[axis]);
    }
    PrintDummyValues(10 - LASTCHANNEL);
    SERIAL_PRINTLN();
    queryType = 'X';
    break;

  case 'g': // Send transmitter calibration data
    for (byte axis = XAXIS; axis < LASTCHANNEL; axis++) {
      Serial.print(receiverSlope[axis], 6);
      Serial.print(',');
    }
    SERIAL_PRINTLN();
    queryType = 'X';
    break;

  case 'h': // Send transmitter calibration data
    for (byte axis = XAXIS; axis < LASTCHANNEL; axis++) {
      Serial.print(receiverOffset[axis], 6);
      Serial.print(',');
    }
    SERIAL_PRINTLN();
    queryType = 'X';
    break;

  case 'i': // Send sensor data
    for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
      PrintValueComma(gyroRate[axis]);
    }
    for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
      PrintValueComma(filteredAccel[axis]);
    }
    for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
      #if defined(HeadingMagHold)
        PrintValueComma(getMagnetometerData(axis));
      #else
        PrintValueComma(0);
      #endif
    }
    SERIAL_PRINTLN();
    break;

  case 'j': // Send raw mag values
    #ifdef HeadingMagHold
      PrintValueComma(getMagnetometerRawData(XAXIS));
      PrintValueComma(getMagnetometerRawData(YAXIS));
      SERIAL_PRINTLN(getMagnetometerRawData(ZAXIS));
    #endif
    break;

  case 'k': // Send accelerometer cal values
    SERIAL_PRINT(accelScaleFactor[XAXIS], 6);
    comma();
    SERIAL_PRINT(runTimeAccelBias[XAXIS], 6);
    comma();
    SERIAL_PRINT(accelScaleFactor[YAXIS], 6);
    comma();
    SERIAL_PRINT(runTimeAccelBias[YAXIS], 6);
    comma();
    SERIAL_PRINT(accelScaleFactor[ZAXIS], 6);
    comma();
    SERIAL_PRINTLN(runTimeAccelBias[ZAXIS], 6);
    queryType = 'X';
    break;

  case 'l': // Send raw accel values
    measureAccelSum();
    PrintValueComma((int)(accelSample[XAXIS]/accelSampleCount));
    accelSample[XAXIS] = 0;
    PrintValueComma((int)(accelSample[YAXIS]/accelSampleCount));
    accelSample[YAXIS] = 0;
    SERIAL_PRINTLN ((int)(accelSample[ZAXIS]/accelSampleCount));
    accelSample[ZAXIS] = 0;
    accelSampleCount = 0;
    break;

  case 'm': // Send magnetometer cal values
    #ifdef HeadingMagHold
      SERIAL_PRINT(magBias[XAXIS], 6);
      comma();
      SERIAL_PRINT(magBias[YAXIS], 6);
      comma();
      SERIAL_PRINTLN(magBias[ZAXIS], 6);
    #endif
    queryType = 'X';
    break;

  case 'n': // battery monitor
    #ifdef BattMonitor
      PrintValueComma(batteryMonitorAlarmVoltage);
      PrintValueComma(batteryMonitorThrottleTarget);
      PrintValueComma(batteryMonitorGoingDownTime);
    #else
      PrintDummyValues(3);
    #endif
    SERIAL_PRINTLN();
    queryType = 'X';
    break;

  case 'o': // send waypoints
    #ifdef UseGPSNavigator
      for (byte index = 0; index < MAX_WAYPOINTS; index++) {
        PrintValueComma(index);
        PrintValueComma(waypoint[index].latitude);
        PrintValueComma(waypoint[index].longitude);
        PrintValueComma(waypoint[index].altitude);
      }
    #else
      PrintDummyValues(4);
    #endif
    SERIAL_PRINTLN();
    queryType = 'X';
    break;

  case 'p': // Send Camera values
    #ifdef CameraControl
      PrintValueComma(cameraMode);
      PrintValueComma(servoCenterPitch);
      PrintValueComma(servoCenterRoll);
      PrintValueComma(servoCenterYaw);
      PrintValueComma(mCameraPitch);
      PrintValueComma(mCameraRoll);
      PrintValueComma(mCameraYaw);
      PrintValueComma(servoMinPitch);
      PrintValueComma(servoMinRoll);
      PrintValueComma(servoMinYaw);
      PrintValueComma(servoMaxPitch);
      PrintValueComma(servoMaxRoll);
      PrintValueComma(servoMaxYaw);
      #ifdef CameraTXControl
        PrintValueComma(servoTXChannels);
      #endif
    #else
      #ifdef CameraTXControl
        PrintDummyValues(14);
      #else
        PrintDummyValues(13);
      #endif
    #endif
    SERIAL_PRINTLN();
    queryType = 'X';
    break;

  case 'q': // Send Vehicle State Value
    SERIAL_PRINTLN(vehicleState);
    queryType = 'X';
    break;

  case 'r': // Vehicle attitude
    PrintValueComma(kinematicsAngle[XAXIS]);
    PrintValueComma(kinematicsAngle[YAXIS]);
    SERIAL_PRINTLN(getHeading());
    break;

  case 's': // Send all flight data
    PrintValueComma(motorArmed);
    PrintValueComma(kinematicsAngle[XAXIS]);
    PrintValueComma(kinematicsAngle[YAXIS]);
    PrintValueComma(getHeading());
    #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
      #if defined AltitudeHoldBaro
        PrintValueComma(getBaroAltitude());
      #elif defined AltitudeHoldRangeFinder
        PrintValueComma(rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX] != INVALID_RANGE ? rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX] : 0.0);
      #endif
      PrintValueComma((int)altitudeHoldState);
    #else
      PrintValueComma(0);
      PrintValueComma(0);
    #endif

    for (byte channel = 0; channel < 8; channel++) { // Configurator expects 8 values
      PrintValueComma((channel < LASTCHANNEL) ? receiverCommand[channel] : 0);
    }

    for (byte motor = 0; motor < LASTMOTOR; motor++) {
      PrintValueComma(motorCommand[motor]);
    }
    PrintDummyValues(8 - LASTMOTOR); // max of 8 motor outputs supported

    #ifdef BattMonitor
      PrintValueComma((float)batteryData[0].voltage/100.0); // voltage internally stored at 10mV:s
    #else
      PrintValueComma(0);
    #endif
    PrintValueComma(flightMode);
    SERIAL_PRINTLN();
    break;

  case 't': // Send processed transmitter values
    for (byte axis = 0; axis < LASTCHANNEL; axis++) {
      PrintValueComma(receiverCommand[axis]);
    }
    SERIAL_PRINTLN();
    break;

  case 'u': // Send range finder values
    #if defined (AltitudeHoldRangeFinder)
      PrintValueComma(maxRangeFinderRange);
      SERIAL_PRINTLN(minRangeFinderRange);
    #else
      PrintValueComma(0);
      SERIAL_PRINTLN(0);
    #endif
    queryType = 'X';
    break;

  case 'v': // Send GPS PIDs
    #if defined (UseGPSNavigator)
      PrintPID(GPSROLL_PID_IDX);
      PrintPID(GPSPITCH_PID_IDX);
      PrintPID(GPSYAW_PID_IDX);
      queryType = 'X';
    #else
      PrintDummyValues(9);
    #endif
    SERIAL_PRINTLN();
    queryType = 'X';
    break;
  case 'y': // send GPS info
    #if defined (UseGPS)
      PrintValueComma(gpsData.state);
      PrintValueComma(gpsData.lat);
      PrintValueComma(gpsData.lon);
      PrintValueComma(gpsData.height);
      PrintValueComma(gpsData.course);
      PrintValueComma(gpsData.speed);
      PrintValueComma(gpsData.accuracy);
      PrintValueComma(gpsData.sats);
      PrintValueComma(gpsData.fixtime);
      PrintValueComma(gpsData.sentences);
      PrintValueComma(gpsData.idlecount);
    #else
      PrintDummyValues(11);
    #endif    
    SERIAL_PRINTLN();
    break;
 
  case 'z': // Send all Altitude data 
    #if defined (AltitudeHoldBaro) 
      PrintValueComma(getBaroAltitude()); 
    #else
      PrintValueComma(0);
    #endif 
    #if defined (AltitudeHoldRangeFinder) 
      SERIAL_PRINTLN(rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX]);
    #else
      SERIAL_PRINTLN(0); 
    #endif 
    break;
    
  case '$': // send BatteryMonitor voltage/current readings
    #if defined (BattMonitor)
      PrintValueComma((float)batteryData[0].voltage/100.0); // voltage internally stored at 10mV:s
      #if defined (BM_EXTENDED)
        PrintValueComma((float)batteryData[0].current/100.0);
        PrintValueComma((float)batteryData[0].usedCapacity/1000.0);
      #else
        PrintDummyValues(2);
      #endif
    #else
      PrintDummyValues(3);
    #endif
    SERIAL_PRINTLN();
    break;
    
  case '%': // send RSSI
    #if defined (UseAnalogRSSIReader) || defined (UseEzUHFRSSIReader) || defined (UseSBUSRSSIReader)
      SERIAL_PRINTLN(rssiRawValue);
    #else
      SERIAL_PRINTLN(0);
    #endif
    break;

  case 'x': // Stop sending messages
    break;

  case '!': // Send flight software version
    SERIAL_PRINTLN(SOFTWARE_VERSION, 1);
    queryType = 'X';
    break;

  case '#': // Send configuration
    reportVehicleState();
    queryType = 'X';
    break;

  case '6': // Report remote commands
    for (byte motor = 0; motor < LASTMOTOR; motor++) {
      PrintValueComma(motorCommand[motor]);
    }
    SERIAL_PRINTLN();
    queryType = 'X';
    break;

#if defined(OSD) && defined(OSD_LOADFONT)
  case '&': // fontload
    if (OFF == motorArmed) {
      max7456LoadFont();
    }
    queryType = 'X';
    break;
#endif

  }
}

void readValueSerial(char *data, byte size) {
  byte index = 0;
  byte timeout = 0;
  data[0] = '\0';

  do {
    if (SERIAL_AVAILABLE() == 0) {
      delay(1);
      timeout++;
    } else {
      data[index] = SERIAL_READ();
      timeout = 0;
      index++;
    }
  } while ((index == 0 || data[index-1] != ';') && (timeout < 10) && (index < size-1));

  data[index] = '\0';
}


// Used to read floating point values from the serial port
float readFloatSerial() {
  char data[15] = "";

  readValueSerial(data, sizeof(data));
  return atof(data);
}

// Used to read integer values from the serial port
long readIntegerSerial() {
  char data[16] = "";

  readValueSerial(data, sizeof(data));
  return atol(data);
}

void comma() {
  SERIAL_PRINT(',');
}
