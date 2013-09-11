#include "Receiver.h"

int lastReceiverChannel = 0;

float receiverXmitFactor = 0.0;
int receiverData[MAX_NB_CHANNEL] = {0,0,0,0,0,0,0,0,0,0};
int receiverZero[3] = {0,0,0};
int receiverCommand[MAX_NB_CHANNEL] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //10 input
int receiverCommandSmooth[MAX_NB_CHANNEL] = {0,0,0,0,0,0,0,0,0,0,};
float receiverSlope[MAX_NB_CHANNEL] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
float receiverOffset[MAX_NB_CHANNEL] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
float receiverSmoothFactor[MAX_NB_CHANNEL] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
int channelCal;

void initializeReceiverParam(int nbChannel = 6) {

	lastReceiverChannel = nbChannel;

	receiverCommand[XAXIS] = 1500;
	receiverCommand[YAXIS] = 1500;
	receiverCommand[ZAXIS] = 1500;
	receiverCommand[THROTTLE] = 1000;
	receiverCommand[MODE] = 1000;
	receiverCommand[AUX1] = 1000;
	receiverCommand[AUX2] = 1000;
	receiverCommand[AUX3] = 1000;
	receiverCommand[AUX4] = 1000;
	receiverCommand[AUX5] = 1000;

	for (byte channel = XAXIS; channel < lastReceiverChannel; channel++) {
		receiverCommandSmooth[channel] = 1.0;
	}
	for (byte channel = XAXIS; channel < THROTTLE; channel++) {
		receiverZero[channel] = 1500;
	}

	for (byte channel = XAXIS; channel < lastReceiverChannel; channel++) {
		receiverSlope[channel] = 1;
	}	
	for (byte channel = XAXIS; channel < lastReceiverChannel; channel++) {
		receiverOffset[channel] = 1;
	}
	for (byte channel = XAXIS; channel < lastReceiverChannel; channel++) {
		receiverSmoothFactor[channel] = 1; 
	}
}


void readReceiver()
{
	if(!_initialized)
	{
		//To start at all
		receiverCommand[THROTTLE] = 1000;

		if(_motorsArmed)
		{
			initializeReceiverParam();
			ResetInputData();
			_initialized = true;
		}

		if(!_motorsArmed && _safetyChecked)
			ArmMotors();

		if(!_safetyChecked && _calibrationPerformed)
			SafetyCheck();

		if(!_calibrationPerformed)
			PerformCalibration();		
	}
	else
	{
		//FMSignal();
		//SelectProgram();
		//SonarCheck();
		//CalculateAltitude();
		//ApplyHeading();
		//ApplySpeed();
	}
	ApplyData();
}

void ApplyData()
{
	// Apply receiver calibration adjustment
	for(byte channel = XAXIS; channel < channelsInUse; channel++)
	{
		receiverData[channel] = _controllerInput[channel];

		receiverCommandSmooth[channel] = filterSmooth(_controllerInput[channel], receiverCommandSmooth[channel], receiverSmoothFactor[channel]);
	}

	// Reduce receiver commands using receiverXmitFactor and center around 1500
	for (byte channel = XAXIS; channel < THROTTLE; channel++) 
	{
		receiverCommand[channel] = _controllerInput[channel];
	}

	// No xmitFactor reduction applied for throttle, mode and AUX
	for (byte channel = THROTTLE; channel < channelsInUse; channel++) 
	{
		receiverCommand[channel] = _controllerInput[channel];
	}
}

//Print X-, Y-, Z-axis and Throttle
void PrintStatus()
{
	if(PrintHelper.PrintConfig[STATUSMODE])
	{	
		_hzCounter++;
		for(int i = XAXIS; i < MODE +1; i++)
		{
			Serial.print(receiverCommand[i]);
			Serial.print(", ");
		}
		Serial.println();
	}
}


// return the smoothed & scaled number of radians/sec in stick movement - zero centered
const float getReceiverSIData(byte channel) 
{
	return ((receiverCommand[channel] - receiverZero[channel]) * (2.5 * PWM2RAD));  // +/- 2.5RPS 50% of full rate
}