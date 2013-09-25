#ifndef _AEROQUAD_ACCELEROMETER_ADXL345_H_
#define _AEROQUAD_ACCELEROMETER_ADXL345_H_

#include "Accelerometer.h"
#include "Device_I2C.h"
#include "SensorsStatus.h"

#define ACCEL_ADDRESS 0x53

void initializeAccel();
void measureAccel();
void measureAccelSum();
void evaluateMetersPerSec();
void computeAccelBias();

#endif
