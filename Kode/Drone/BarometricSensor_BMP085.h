#ifndef _AQ_BAROMETRIC_SENSOR_BMP085_
#define _AQ_BAROMETRIC_SENSOR_BMP085_

#include "BarometricSensor.h"
#include "Device_I2C.h"
#include "AQMath.h"
#include "SensorsStatus.h"

// This sets up the BMP085 from Sparkfun
// Code from http://wiring.org.co/learning/libraries/bmp085.html
// Also made bug fixes based on BMP085 library from Jordi Munoz and Jose Julio

#define BMP085_I2C_ADDRESS 0x77

#define TEMPERATURE 0
#define PRESSURE 1
#define OVER_SAMPLING_SETTING 1 // use to be 3

extern byte overSamplingSetting;
extern int ac1, ac2, ac3;
extern unsigned int ac4, ac5, ac6;
extern int b1, b2, mb, mc, md;
extern long pressure ;
extern long rawPressure, rawTemperature;
extern byte pressureCount;
extern float pressureFactor;
extern boolean isReadPressure;
extern float rawPressureSum;
extern byte rawPressureSumCount;
  

void requestRawPressure();
long readRawPressure(); 
void requestRawTemperature();
unsigned int readRawTemperature();
void initializeBaro();
void measureBaro();
void measureBaroSum();


#endif
