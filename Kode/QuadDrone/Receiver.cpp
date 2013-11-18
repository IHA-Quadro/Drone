#include "Receiver.h"

int lastReceiverChannel;
int channelCal;

float receiverXmitFactor;
int receiverData[MAX_NB_CHANNEL];
int receiverZero[3];
int receiverCommand[MAX_NB_CHANNEL]; 
int receiverCommandSmooth[MAX_NB_CHANNEL];
float receiverSlope[MAX_NB_CHANNEL];
float receiverOffset[MAX_NB_CHANNEL];
float receiverSmoothFactor[MAX_NB_CHANNEL];

void initializeReceiverValues()
{
	receiverXmitFactor = 0.0;
	lastReceiverChannel = 0;

	for(int i = 0; i < MAX_NB_CHANNEL ;i++)
	{
		receiverData[i] = 0;
		receiverSlope[i] = 0.0;
	}
}

void initializeReceiverParam(int nbChannel) 
{
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
			initializeReceiverValues();
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
		FMSignal();
		SelectProgram();
		SonarCheck();
		CalculateAltitude();
		ApplyHeading();
		ApplySpeed();
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
void printNewLine()
{
	if(PrintConfig[STATUSMODE])
	{	
		for(int i = XAXIS; i < MODE +1; i++)
		{
			Serial.print(receiverCommand[i]);
			Serial.print(", ");
		}
		Serial.println();
	}
}

//return the smoothed & scaled number of radians/sec in stick movement - zero centered
const float getReceiverSIData(byte channel) 
{
	return ((receiverCommand[channel] - receiverZero[channel]) * (2.5 * PWM2RAD));  // +/- 2.5RPS 50% of full rate
}