#ifndef _AEROQUAD_GYROSCOPE_ITG3200_H_
#define _AEROQUAD_GYROSCOPE_ITG3200_H_

#include "AQMath.h"
#include "SensorsStatus.h"
#include "Gyroscope_ITG3200Common.h"

void measureSpecificGyroADC(int *gyroADC);
void measureSpecificGyroSum();
void evaluateSpecificGyroRate(int *gyroADC);
boolean calibrateGyro();



#endif
