#ifndef _AQ_BAROMETRIC_SENSOR_
#define _AQ_BAROMETRIC_SENSOR_

#include <Arduino.h>
#include "AQMath.h"
#include "Device_I2C.h"
#include "GlobalDefined.h"
#include "SensorsStatus.h"

float baroAltitude      = 0.0; 
float baroRawAltitude   = 0.0;
float baroGroundAltitude = 0.0;
float baroSmoothFactor   = 0.02;
  
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

#define BMP085_I2C_ADDRESS 0x77

#define TEMPERATURE 0
#define PRESSURE 1
#define OVER_SAMPLING_SETTING 1 // use to be 3

byte overSamplingSetting = OVER_SAMPLING_SETTING;
int ac1 = 0, ac2 = 0, ac3 = 0;
unsigned int ac4 = 0, ac5 = 0, ac6 = 0;
int b1 = 0, b2 = 0, mb = 0, mc = 0, md = 0;
long pressure = 0;
long rawPressure = 0, rawTemperature = 0;
byte pressureCount = 0;
float pressureFactor = 1/5.255;
boolean isReadPressure = false;
float rawPressureSum = 0;
byte rawPressureSumCount = 0;
  


#endif