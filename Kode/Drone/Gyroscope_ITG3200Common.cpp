#include "Gyroscope_ITG3200Common.h"

float gyroTempBias[3] = {0.0,0.0,0.0};

void initializeGyro() {
	if ((readWhoI2C(ITG3200_ADDRESS) & ITG3200_IDENTITY_MASK) == ITG3200_IDENTITY) 
		vehicleState |= GYRO_DETECTED;
	
	gyroScaleFactor = radians(1.0 / 14.375);  //  ITG3200 14.375 LSBs per °/sec
	updateRegisterI2C(ITG3200_ADDRESS, ITG3200_RESET_ADDRESS, ITG3200_RESET_VALUE); // send a reset to the device
	updateRegisterI2C(ITG3200_ADDRESS, ITG3200_LOW_PASS_FILTER_ADDR, ITG3200_LOW_PASS_FILTER_VALUE); // 10Hz low pass filter
	updateRegisterI2C(ITG3200_ADDRESS, ITG3200_RESET_ADDRESS, ITG3200_OSCILLATOR_VALUE); // use internal oscillator 
}

void measureGyro() 
{
	sendByteI2C(ITG3200_ADDRESS, ITG3200_MEMORY_ADDRESS);
	Wire.requestFrom(ITG3200_ADDRESS, ITG3200_BUFFER_SIZE);

	int gyroADC[3];
	measureSpecificGyroADC(gyroADC);

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

void measureGyroSum() 
{
	sendByteI2C(ITG3200_ADDRESS, ITG3200_MEMORY_ADDRESS);
	Wire.requestFrom(ITG3200_ADDRESS, ITG3200_BUFFER_SIZE);

	measureSpecificGyroSum();

	gyroSampleCount++;
}

void evaluateGyroRate() 
{
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

