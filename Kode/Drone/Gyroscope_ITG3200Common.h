#ifndef _AEROQUAD_GYROSCOPE_ITG3200_COMMON_H_
#define _AEROQUAD_GYROSCOPE_ITG3200_COMMON_H_

#include "Device_I2C.h"
#include "Gyroscope.h"
#include "SensorsStatus.h"

//#ifdef ITG3200_ADDRESS_ALTERNATE
  #define ITG3200_ADDRESS					0x68
//#else
//  #define ITG3200_ADDRESS					0x69
//#endif

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
#define ITG3200_SCALE_TO_RADIANS		823.626831 // 14.375 LSBs per �/sec, / Pi / 180
#define ITG3200_TEMPERATURE_ADDRESS     0x1B


extern float gyroTempBias[3];
void measureSpecificGyroADC(int *gyroADC);
void measureSpecificGyroSum();
void evaluateSpecificGyroRate(int *gyroADC);


void initializeGyro();
void measureGyro();
void measureGyroSum();
void evaluateGyroRate();


#endif
