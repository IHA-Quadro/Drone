#include "Accelerometer.h"

float accelScaleFactor[3];
float runTimeAccelBias[3];
float accelOneG;
float meterPerSecSec[3];
long accelSample[3];
byte accelSampleCount;
  
void initializeAccel();
void measureAccel();
void measureAccelSum();
void evaluateMetersPerSec();
void computeAccelBias();

void ResetAccelData();


void ResetAccelData()
{
	for(byte axis = XAXIS; axis <= ZAXIS; axis++)
	{
		accelSample[axis] = 0;
		accelSampleCount = 0;
		meterPerSecSec[axis] = 0.0;
		runTimeAccelBias[axis] = 0;
	}
	accelOneG = 0.0;
}

void initializeAccel() 
{
	for(int i = 0; i < 3;i++)
	{
		accelScaleFactor[i]=0.0;
		runTimeAccelBias[i]=0.0;
		meterPerSecSec[i]=0.0;
		accelSample[i]=0;
	}
	accelSampleCount=0;
	accelOneG = 0;

  if (readWhoI2C(ACCEL_ADDRESS) ==  0xE5) { 		// page 14 of datasheet
    vehicleState |= ACCEL_DETECTED;
  }
	
  updateRegisterI2C(ACCEL_ADDRESS, 0x2D, 1<<3);     // set device to *measure*
  updateRegisterI2C(ACCEL_ADDRESS, 0x31, 0x09);     // set full range and +/- 4G
  updateRegisterI2C(ACCEL_ADDRESS, 0x2C, 8+2+1);    // 200hz sampling
  delay(10); 
}
  
void measureAccel() {

  sendByteI2C(ACCEL_ADDRESS, 0x32);
  Wire.requestFrom(ACCEL_ADDRESS, 6); 

  meterPerSecSec[YAXIS] = readReverseShortI2C() * accelScaleFactor[YAXIS] + runTimeAccelBias[YAXIS];
  meterPerSecSec[XAXIS] = readReverseShortI2C() * accelScaleFactor[XAXIS] + runTimeAccelBias[XAXIS];
  meterPerSecSec[ZAXIS] = readReverseShortI2C() * accelScaleFactor[ZAXIS] + runTimeAccelBias[ZAXIS];
}

void measureAccelSum() {

  sendByteI2C(ACCEL_ADDRESS, 0x32);
  Wire.requestFrom(ACCEL_ADDRESS, 6);
  
  accelSample[YAXIS] += readReverseShortI2C() ;
  accelSample[XAXIS] += readReverseShortI2C() ;
  accelSample[ZAXIS] += readReverseShortI2C() ;
  accelSampleCount++;
}

void evaluateMetersPerSec() {
	
  for (byte axis = XAXIS; axis <= ZAXIS; axis++) 
	{
    meterPerSecSec[axis] = (accelSample[axis] / accelSampleCount) * accelScaleFactor[axis] + runTimeAccelBias[axis];
	accelSample[axis] = 0;
  }
  accelSampleCount = 0;		
}

void computeAccelBias() {
  
  for (int samples = 0; samples < SAMPLECOUNT; samples++) {
    measureAccelSum();
    delayMicroseconds(2500);
  }

  for (byte axis = 0; axis < 3; axis++) {
    meterPerSecSec[axis] = (float(accelSample[axis])/SAMPLECOUNT) * accelScaleFactor[axis];
    accelSample[axis] = 0;
  }
  accelSampleCount = 0;

  runTimeAccelBias[XAXIS] = -meterPerSecSec[XAXIS];
  runTimeAccelBias[YAXIS] = -meterPerSecSec[YAXIS];
  runTimeAccelBias[ZAXIS] = -9.8065 - meterPerSecSec[ZAXIS];

  accelOneG = fabs(meterPerSecSec[ZAXIS] + runTimeAccelBias[ZAXIS]);
}
