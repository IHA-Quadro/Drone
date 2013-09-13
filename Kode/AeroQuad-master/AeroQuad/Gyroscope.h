#ifndef _AEROQUAD_GYROSCOPE_H_
#define _AEROQUAD_GYROSCOPE_H_

#include <Arduino.h>
#include "AQMath.h"
#include "Device_I2C.h"
#include "GlobalDefined.h"
#include "SensorsStatus.h"

#define FINDZERO 49

#ifdef ITG3200_ADDRESS_ALTERNATE
#define ITG3200_ADDRESS					0x68
#else
#define ITG3200_ADDRESS					0x69
#endif

#define GYRO_CALIBRATION_TRESHOLD 4

#define ITG3200_IDENTITY                0x68
#define ITG3200_IDENTITY_MASK           0x7E
#define ITG3200_MEMORY_ADDRESS			0x1D
#define ITG3200_BUFFER_SIZE				6
#define ITG3200_RESET_ADDRESS			0x3E
#define ITG3200_RESET_VALUE				0x80
#define ITG3200_LOW_PASS_FILTER_ADDR	0x16
#define ITG3200_LOW_PASS_FILTER_VALUE	0x1D	// 10Hz low pass filter
#define ITG3200_OSCILLATOR_ADDR			0x3E
#define ITG3200_OSCILLATOR_VALUE		0x01	// use X gyro oscillator
#define ITG3200_SCALE_TO_RADIANS		823.626831 // 14.375 LSBs per °/sec, / Pi / 180
#define ITG3200_TEMPERATURE_ADDRESS     0x1B


float gyroRate[3] = {0.0,0.0,0.0};
int   gyroZero[3] = {0,0,0};
long  gyroSample[3] = {0,0,0};
float gyroScaleFactor = 0.0;
float gyroHeading = 0.0;
unsigned long gyroLastMesuredTime = 0;
byte gyroSampleCount = 0;

void measureGyroSum();
void evaluateGyroRate();
void initializeGyro();
void measureGyro();
boolean calibrateGyro();
void readGyroTemp();

void ResetGyroData();

float gyroTempBias[3] = {0.0,0.0,0.0};
void measureSpecificGyroADC(int *gyroADC);
void measureSpecificGyroSum();
void evaluateSpecificGyroRate(int *gyroADC);


#endif