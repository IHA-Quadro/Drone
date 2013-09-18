#ifndef _AEROQUAD_ACCELEROMETER2_H_
#define _AEROQUAD_ACCELEROMETER2_H_

#include "Device_I2C.h"
#include "GlobalDefined.h"
#include "SensorsStatus.h"

#define SAMPLECOUNT 400.0
#define ACCEL_ADDRESS 0x53

extern float accelScaleFactor[3];
extern float runTimeAccelBias[3];
extern float accelOneG;
extern float meterPerSecSec[3];
extern long accelSample[3];
extern byte accelSampleCount;
  
extern void initializeAccel();
extern void measureAccel();
extern void measureAccelSum();
extern void evaluateMetersPerSec();
extern void computeAccelBias();

extern void ResetAccelData();
  
#endif