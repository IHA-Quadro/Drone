// SerialCom.pde is responsible for the serial communication for commands and telemetry from the AeroQuad
// This comtains readSerialCommand() which listens for a serial command and it's arguments
// This also contains readSerialTelemetry() which listens for a telemetry request and responds with the requested data
// For more information on each command/telemetry look at: http://aeroquad.com/content.php?117

// Includes re-write / fixes from Aadamson and ala42, special thanks to those guys!
// http://aeroquad.com/showthread.php?1461-We-have-some-hidden-warnings&p=14618&viewfull=1#post14618

#ifndef _AQ_SERIAL_COMM_
#define _AQ_SERIAL_COMM_

#include "AeroQuad.h"
#include "BarometricSensor.h"
#include "ControlFaker.h"
#include "DataStorage.h"
#include "FlightControlQuadPlus.h"
#include "GlobalDefined.h"
#include "PID.h"
#include "ReceiveCommandTestData.h"
#include "SensorsStatus.h"

extern char queryType;

void initCommunication();

//***************************************************************************************************
//********************************** Serial Commands ************************************************
//***************************************************************************************************
bool validateCalibrateCommand(byte command);

void readSerialPID(unsigned char PIDid);

void skipSerialValues(byte number);

void readSerialCommand();

//***************************************************************************************************
//********************************* Serial Telemetry ************************************************
//***************************************************************************************************

void PrintValueComma(float val);
void PrintValueComma(double val);
void PrintValueComma(char val);
void PrintValueComma(int val);
void PrintValueComma(unsigned long val);
void PrintValueComma(byte val);
void PrintValueComma(long int val);
void PrintPID(unsigned char IDPid);
void PrintDummyValues(byte number);
float getHeading();
void sendSerialTelemetry();
void readValueSerial(char *data, byte size);
float readFloatSerial();
long readIntegerSerial();
void comma();
void printVehicleState(const char *sensorName, unsigned long state, const char *message);

#ifdef BinaryWrite
void printInt(int data) {
	byte msb, lsb;

	msb = data >> 8;
	lsb = data & 0xff;

	binaryPort->write(msb);
	binaryPort->write(lsb);
}

void sendBinaryFloat(float data) {
	union binaryFloatType {
		byte floatByte[4];
		float floatVal;
	} binaryFloat;

	binaryFloat.floatVal = data;
	binaryPort->write(binaryFloat.floatByte[3]);
	binaryPort->write(binaryFloat.floatByte[2]);
	binaryPort->write(binaryFloat.floatByte[1]);
	binaryPort->write(binaryFloat.floatByte[0]);
}

void sendBinaryuslong(unsigned long data) {
	union binaryuslongType {
		byte uslongByte[4];
		unsigned long uslongVal;
	} binaryuslong;

	binaryuslong.uslongVal = data;
	binaryPort->write(binaryuslong.uslongByte[3]);
	binaryPort->write(binaryuslong.uslongByte[2]);
	binaryPort->write(binaryuslong.uslongByte[1]);
	binaryPort->write(binaryuslong.uslongByte[0]);
}


void fastTelemetry()
{
	// **************************************************************
	// ***************** Fast Transfer Of Sensor Data ***************
	// **************************************************************
	// AeroQuad.h defines the output rate to be 10ms
	// Since writing to UART is done by hardware, unable to measure data rate directly
	// Through analysis:  115200 baud = 115200 bits/second = 14400 bytes/second
	// If float = 4 bytes, then 3600 floats/second
	// If 10 ms output rate, then 36 floats/10ms
	// Number of floats written using sendBinaryFloat is 15

	if (motorArmed == ON) {
#ifdef OpenlogBinaryWrite
		printInt(21845); // Start word of 0x5555
		sendBinaryuslong(currentTime);
		printInt((int)flightMode);
		for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
			sendBinaryFloat(gyroRate[axis]);
		}
		for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
			sendBinaryFloat(meterPerSecSec[axis]);
		}
		sendBinaryFloat(accelOneG);
#ifdef HeadingMagHold
		sendBinaryFloat(hdgX);
		sendBinaryFloat(hdgY);
		for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
#if defined(HeadingMagHold)
			sendBinaryFloat(getMagnetometerData(axis));
#endif
		}
#else
		sendBinaryFloat(0.0);
		sendBinaryFloat(0.0);
		sendBinaryFloat(0.0);
#endif
		for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
			sendBinaryFloat(kinematicsAngle[axis]);
		}
		printInt(32767); // Stop word of 0x7FFF
#else
		printInt(21845); // Start word of 0x5555
		for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
			sendBinaryFloat(gyroRate[axis]);
		}
		for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
			sendBinaryFloat(meterPerSecSec[axis]);
		}
		for (byte axis = XAXIS; axis <= ZAXIS; axis++)
#if defined(HeadingMagHold)
			sendBinaryFloat(getMagnetometerData(axis));
#else
			sendBinaryFloat(0);
#endif
		for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
			sendBinaryFloat(getGyroUnbias(axis));
		}
		for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
			sendBinaryFloat(kinematicsAngle[axis]);
		}
		printInt(32767); // Stop word of 0x7FFF
#endif
	}
}
#endif // BinaryWrite

void reportVehicleState();



#ifdef SlowTelemetry
struct __attribute__((packed)) telemetryPacket {
	unsigned short  id;
	long  latitude;
	long  longitude;
	short altitude;
	short course;
	short heading;
	byte  speed;
	byte  rssi;
	byte  voltage;
	byte  current;
	unsigned short capacity;
	unsigned short gpsinfo;
	byte  ecc[8];
};

union telemetryBuffer {
	struct telemetryPacket data;
	byte   bytes[32];
} telemetryBuffer;

#define TELEMETRY_MSGSIZE 24
#define TELEMETRY_MSGSIZE_ECC (TELEMETRY_MSGSIZE + 8)

byte slowTelemetryByte = 255;

void initSlowTelemetry() {
#ifdef SoftModem
	softmodemInit();
#else
	Serial2.begin(1200);
#endif
	slowTelemetryByte = 255;
}

/* 100Hz task, sends data out byte by byte */
void updateSlowTelemetry100Hz() {

	if (slowTelemetryByte < TELEMETRY_MSGSIZE_ECC ) {
#ifdef SoftModem
		if (softmodemFreeToSend()) {
			softmodemSendByte(telemetryBuffer.bytes[slowTelemetryByte]);
			slowTelemetryByte++;
		}
#else
		Serial2.write(telemetryBuffer.bytes[slowTelemetryByte]);
		slowTelemetryByte++;
#endif
	}
	else {
		slowTelemetryByte=255;
	}
}

void updateSlowTelemetry10Hz() {

	if (slowTelemetryByte==255) {
		telemetryBuffer.data.id        = 0x5141; // "AQ"
#ifdef UseGPS
		telemetryBuffer.data.latitude  = currentPosition.latitude;  // degrees/10000000
		telemetryBuffer.data.longitude = currentPosition.longitude; // degrees/10000000
		telemetryBuffer.data.course    = getCourse()/10; // degrees
		telemetryBuffer.data.speed     = getGpsSpeed()*36/1000;              // km/h
		telemetryBuffer.data.heading   = (short)(trueNorthHeading*RAD2DEG); // degrees
		telemetryBuffer.data.gpsinfo   = 0;
		telemetryBuffer.data.gpsinfo  |= (((unsigned short)((gpsData.sats<15)?gpsData.sats:15)) << 12);
#else
		telemetryBuffer.data.latitude  = 0;
		telemetryBuffer.data.longitude = 0;
		telemetryBuffer.data.course    = 0;
		telemetryBuffer.data.speed     = 0;
		telemetryBuffer.data.heading   = 0;
		telemetryBuffer.data.gpsinfo   = 0;
#endif

#ifdef AltitudeHoldBaro
		telemetryBuffer.data.altitude  = (short)(getBaroAltitude()*10.0); // 0.1m
#else
		telemetryBuffer.data.altitude  = 0;
#endif

#ifdef UseRSSIFaileSafe
#ifdef RSSI_RAWVAL
		telemetryBuffer.data.rssi      = rssiRawValue/10; // scale to 0-100
#else
		telemetryBuffer.data.rssi      = rssiRawValue;
#endif
#else
		telemetryBuffer.data.rssi      = 100;
#endif

#ifdef BattMonitor
		telemetryBuffer.data.voltage   = batteryData[0].voltage/10;  // to 0.1V
		telemetryBuffer.data.current   = batteryData[0].current/100; // to A
		telemetryBuffer.data.capacity  = batteryData[0].usedCapacity/1000; // mAh
#else
		telemetryBuffer.data.voltage   = 0;
		telemetryBuffer.data.current   = 0;
		telemetryBuffer.data.capacity  = 0;
#endif

		/* add ECC */
		encode_data(telemetryBuffer.bytes,24);

		/* trigger send */
		slowTelemetryByte=0;
	}
}
#endif // SlowTelemetry

#endif // _AQ_SERIAL_COMM_
