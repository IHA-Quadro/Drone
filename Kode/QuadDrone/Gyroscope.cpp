#include "Gyroscope.h"

float gyroRate[3] = {0.0,0.0,0.0};
int   gyroZero[3] = {0,0,0};
long  gyroSample[3] = {0,0,0};
float gyroTempBias[3] = {0.0,0.0,0.0};
float gyroScaleFactor = 0.0;
float gyroHeading = 0.0;
unsigned long gyroLastMesuredTime = 0;
byte gyroSampleCount = 0;

void initializeGyro() {
	if ((readWhoI2C(ITG3200_ADDRESS) & ITG3200_IDENTITY_MASK) == ITG3200_IDENTITY) {
		vehicleState |= GYRO_DETECTED;
	}

	Wire.begin(ITG3200_ADDRESS);
	gyroScaleFactor = radians(1.0 / 14.375);  //  ITG3200 14.375 LSBs per °/sec
	updateRegisterI2C(ITG3200_ADDRESS, ITG3200_RESET_ADDRESS, ITG3200_RESET_VALUE); // send a reset to the device
	updateRegisterI2C(ITG3200_ADDRESS, ITG3200_LOW_PASS_FILTER_ADDR, ITG3200_LOW_PASS_FILTER_VALUE); // 10Hz low pass filter
	updateRegisterI2C(ITG3200_ADDRESS, ITG3200_RESET_ADDRESS, ITG3200_OSCILLATOR_VALUE); // use internal oscillator 

	//Wire.begin(ITG3200_ADDRESS);
}

void measureGyro() {
	sendByteI2C(ITG3200_ADDRESS, ITG3200_MEMORY_ADDRESS);
	Wire.requestFrom(ITG3200_ADDRESS, ITG3200_BUFFER_SIZE);

	int gyroADC[3];
	measureSpecificGyroADC(gyroADC);

	for (byte axis = 0; axis <= ZAXIS; axis++) {
		gyroRate[axis] = gyroADC[axis] * gyroScaleFactor;
	}

	// Measure gyro heading
	long int currentTime = micros();
	if (gyroRate[ZAXIS] > radians(1.0) || gyroRate[ZAXIS] < radians(-1.0)) 
		gyroHeading += gyroRate[ZAXIS] * ((currentTime - gyroLastMesuredTime) / 1000000.0);

	gyroLastMesuredTime = currentTime;
}

void measureGyroSum() 
{
	sendByteI2C(ITG3200_ADDRESS, ITG3200_MEMORY_ADDRESS);
	Wire.requestFrom(ITG3200_ADDRESS, ITG3200_BUFFER_SIZE);

	measureSpecificGyroSum();

	gyroSampleCount++;
}

void evaluateGyroRate() {
	int gyroADC[3];
	evaluateSpecificGyroRate(gyroADC);
	gyroSample[XAXIS] = 0;
	gyroSample[YAXIS] = 0;
	gyroSample[ZAXIS] = 0;
	gyroSampleCount = 0;

	for (byte axis = 0; axis <= ZAXIS; axis++) 
	{
		gyroRate[axis] = gyroADC[axis] * gyroScaleFactor;
	}

	// Measure gyro heading
	long int currentTime = micros();
	if (gyroRate[ZAXIS] > radians(1.0) || gyroRate[ZAXIS] < radians(-1.0)) 
		gyroHeading += gyroRate[ZAXIS] * ((currentTime - gyroLastMesuredTime) / 1000000.0);
	
	gyroLastMesuredTime = currentTime;
}

void ResetGyroData()
{
	printNewLine("Reset Gyro data", DEBUGMODE);
	for(int axis = XAXIS; axis < ZAXIS+1; axis++)
	{
		//gyroRate[axis] = 0.0;
		gyroZero[axis] = 0;
	}
	//gyroHeading = 0.0;
	gyroLastMesuredTime = micros();
	gyroSampleCount = 0;
}

void measureSpecificGyroADC(int *gyroADC) {
	gyroADC[YAXIS] = readShortI2C()  - gyroZero[YAXIS];
	gyroADC[XAXIS] = readShortI2C()  - gyroZero[XAXIS];
	gyroADC[ZAXIS] = gyroZero[ZAXIS] - readShortI2C();
}

void measureSpecificGyroSum() {
	gyroSample[YAXIS] += readShortI2C();
	gyroSample[XAXIS] += readShortI2C();
	gyroSample[ZAXIS] += readShortI2C();
}

void evaluateSpecificGyroRate(int *gyroADC) 
{
	gyroADC[XAXIS] = (gyroSample[XAXIS] / gyroSampleCount) - gyroZero[XAXIS];
	gyroADC[YAXIS] = (gyroSample[YAXIS] / gyroSampleCount) - gyroZero[YAXIS];
	gyroADC[ZAXIS] = gyroZero[ZAXIS] - (gyroSample[ZAXIS] / gyroSampleCount);
}

bool calibrateGyro() {
	//Finds gyro drift.
	//Returns false if during calibration there was movement of board. 

	int findZero[FINDZERO];
	int diff = 0;

	for (byte axis = 0; axis < 3; axis++) 
	{
		for (int i=0; i<FINDZERO; i++)
		{
			sendByteI2C(ITG3200_ADDRESS, (axis * 2) + ITG3200_MEMORY_ADDRESS);
			findZero[i] = readShortI2C(ITG3200_ADDRESS);
			delay(10);
		}

		if (axis == XAXIS) 
		{
			int tmp = findMedianIntWithDiff(findZero, FINDZERO, &diff);
			if (diff <= GYRO_CALIBRATION_TRESHOLD) 
			{ // 4 = 0.27826087 degrees during 49*10ms measurements (490ms). 0.57deg/s difference between first and last.
				gyroZero[YAXIS] = tmp;
			} 
			else 
				return false; //Calibration failed.
		}
		else if (axis == YAXIS) 
		{
			int tmp = findMedianIntWithDiff(findZero, FINDZERO, &diff);
			if (diff <= GYRO_CALIBRATION_TRESHOLD) { // 4 = 0.27826087 degrees during 49*10ms measurements (490ms). 0.57deg/s difference between first and last.
				gyroZero[XAXIS] = tmp;
			} 
			else 
				return false; //Calibration failed.
		}
		else 
		{
			int tmp = findMedianIntWithDiff(findZero, FINDZERO, &diff);
			if (diff <= GYRO_CALIBRATION_TRESHOLD) 
			{ // 4 = 0.27826087 degrees during 49*10ms measurements (490ms). 0.57deg/s difference between first and last.
				gyroZero[ZAXIS] = tmp;
			} 
			else 
				return false; //Calibration failed.
		}
	}

	return true; //Calibration successfull.
}