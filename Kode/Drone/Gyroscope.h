#ifndef _AEROQUAD_GYROSCOPE_H_
#define _AEROQUAD_GYROSCOPE_H_

#include "Arduino.h"
#include "GlobalDefined.h"

#define FINDZERO 49

extern float gyroRate[3];
extern int   gyroZero[3];
extern long  gyroSample[3];
extern float gyroScaleFactor;
extern float gyroHeading;
extern unsigned long gyroLastMesuredTime;
extern byte gyroSampleCount;

void measureGyroSum();
void evaluateGyroRate();
void initializeGyro();
void measureGyro();
boolean calibrateGyro();
void readGyroTemp();

#endif