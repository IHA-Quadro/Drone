#ifndef _AEROQUAD_ACCELEROMETER_H_
#define _AEROQUAD_ACCELEROMETER_H_

#include <Arduino.h>

#include "Device_I2C.h"
#include "GlobalDefined.h"
#include "SensorsStatus.h"

#define SAMPLECOUNT 400.0
#define ACCEL_ADDRESS 0x53

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
  
#endif