#ifndef _AEROQUAD_GYROSCOPE_ITG3200_H_
#define _AEROQUAD_GYROSCOPE_ITG3200_H_


#include "SensorsStatus.h"
#include "Gyroscope_ITG3200Common.h"

void measureSpecificGyroADC(int *gyroADC) {

  gyroADC[XAXIS] = readShortI2C()  - gyroZero[XAXIS];
  gyroADC[YAXIS] = gyroZero[YAXIS] - readShortI2C();
  gyroADC[ZAXIS] = gyroZero[ZAXIS] - readShortI2C();
}

void measureSpecificGyroSum() {

  for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
    gyroSample[axis] += readShortI2C();
  }
}

void evaluateSpecificGyroRate(int *gyroADC) {

  gyroADC[XAXIS] = (gyroSample[XAXIS] / gyroSampleCount) - gyroZero[XAXIS];
  gyroADC[YAXIS] = gyroZero[YAXIS] - (gyroSample[YAXIS] / gyroSampleCount);
  gyroADC[ZAXIS] = gyroZero[ZAXIS] - (gyroSample[ZAXIS] / gyroSampleCount);
}

boolean calibrateGyro() {
  //Finds gyro drift.
  //Returns false if during calibration there was movement of board. 

  int findZero[FINDZERO];
  int diff = 0;

  for (byte axis = 0; axis < 3; axis++) {
    for (int i=0; i<FINDZERO; i++) {
      sendByteI2C(ITG3200_ADDRESS, (axis * 2) + ITG3200_MEMORY_ADDRESS);
      findZero[i] = readShortI2C(ITG3200_ADDRESS);
      delay(10);
    }

    int tmp = findMedianIntWithDiff(findZero, FINDZERO, &diff);
	if (diff <= GYRO_CALIBRATION_TRESHOLD) { // 4 = 0.27826087 degrees during 49*10ms measurements (490ms). 0.57deg/s difference between first and last.
	  gyroZero[axis] = tmp;
	} 
	else {
		return false; //Calibration failed.
	}
  }

  return true; //Calibration successfull.
}

#endif
