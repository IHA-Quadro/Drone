#ifndef _AQ_BAROMETRIC_SENSOR_
#define _AQ_BAROMETRIC_SENSOR_

#include <Arduino.h>

#include "AQMath.h"
#include "Device_I2C.h"
#include "GlobalDefined.h"
#include "SensorsStatus.h"
#include "Wire.h"

#define BMP085_I2C_ADDRESS 0x77

#define TEMPERATURE 0
#define PRESSURE 1
#define OVER_SAMPLING_SETTING 1 // use to be 3

extern float baroAltitude; 
extern float baroRawAltitude;
extern float baroGroundAltitude;
extern float baroSmoothFactor;

extern byte overSamplingSetting;
extern int ac1, ac2, ac3;
extern unsigned int ac4, ac5, ac6;
extern int b1, b2, mb, mc, md;
extern long pressure;
extern long rawPressure, rawTemperature;
extern byte pressureCount;
extern float pressureFactor;
extern boolean isReadPressure;
extern float rawPressureSum;
extern byte rawPressureSumCount;
  
// **********************************************************************
// The following function calls must be defined inside any new subclasses
// **********************************************************************
void initializeBaro(); 
void measureBaro();
void measureBaroSum();
void evaluateBaroAltitude();
  
void measureGroundBaro();
const float getBaroAltitude();

void requestRawPressure();
long readRawPressure();
void requestRawTemperature();  
unsigned int readRawTemperature();


// This sets up the BMP085 from Sparkfun
// Code from http://wiring.org.co/learning/libraries/bmp085.html
// Also made bug fixes based on BMP085 library from Jordi Munoz and Jose Julio

#endif