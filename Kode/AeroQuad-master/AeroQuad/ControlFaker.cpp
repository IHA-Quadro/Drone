#include "ControlFaker.h"

void FMSignal()
{

}

void SelectProgram()
{

}

void SonarCheck()
{

}

void CalculateAltitude()
{

}

void ApplyHeading()
{

}

void ApplySpeed()
{

}

int Input[10];
char channelsInUse = 6;

void AssignTestData()
{
	Input[0] = 1;			//x-akse
	Input[1] = 1;			//y-akse
	Input[2] = 1;			//z-akse
	Input[3] = 1;			//Throttle / speed
	Input[4] = 1;			//Mode
	Input[5] = 1;			//AUX1
	Input[6] = 1;			//AUX2
	Input[7] = 1;			//AUX3
	Input[8] = 1;			//AUX4
	Input[9] = 1;			//AUX5
}

void TestData()
{
	// Apply receiver calibration adjustment
	for(byte channel = XAXIS; channel < channelsInUse; channel++)
	{
		receiverData[channel] = Input[channel];

		receiverCommandSmooth[channel] = filterSmooth(Input[channel], receiverCommandSmooth[channel], receiverSmoothFactor[channel]);
	}

	// Reduce receiver commands using receiverXmitFactor and center around 1500
	for (byte channel = XAXIS; channel < THROTTLE; channel++) 
	{
		receiverCommand[channel] = Input[channel];
	}

	// No xmitFactor reduction applied for throttle, mode and AUX
	for (byte channel = THROTTLE; channel < channelsInUse; channel++) 
	{
		receiverCommand[channel] = Input[channel];
	}
}