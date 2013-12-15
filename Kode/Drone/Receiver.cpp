#include "Receiver.h"

#include "Decision.h"

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
	receiverXmitFactor = 1.0;
	lastReceiverChannel = 0;

	for(int i = 0; i < MAX_NB_CHANNEL ;i++)
	{
		receiverData[i] = 0;
		receiverSlope[i] = 0.0;
	}
}

void initializeReceiverParam(int nbChannel = 6) 
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

	for (byte channel = XAXIS; channel < lastReceiverChannel; channel++) {
		receiverCommandSmooth[channel] = 1.0;
	}
	for (byte channel = XAXIS; channel < THROTTLE; channel++) {
		receiverZero[channel] = 1500;
	}

	for (byte channel = XAXIS; channel < lastReceiverChannel; channel++) {
		receiverSlope[channel] = 1.0;
	}	
	for (byte channel = XAXIS; channel < lastReceiverChannel; channel++) {
		receiverOffset[channel] = 1.0;
	}
	for (byte channel = XAXIS; channel < lastReceiverChannel; channel++) {
		receiverSmoothFactor[channel] = 1.0; 
	}
}

void readReceiver()
{
	if(!_initialized)
	{
		if(!IsMotorKilled()) //Motor stopped
		{
			//To start at all
			receiverCommand[THROTTLE] = 1000;

			if(_motorsArmed)
			{
				initializeReceiverValues();
				initializeReceiverParam(lastReceiverChannel);

				ResetFakerData();
				ResetDecisions();
				ResetMessages();
			}

			if(!_motorsArmed && _safetyChecked)
				ArmMotors();

			if(!_safetyChecked && _calibrationPerformed)
				SafetyCheck();

			if(!_calibrationPerformed)
				PerformCalibration();	
		}
	}
	else
	{
		MaxHeightAction(); //Safety mechanism

		// AUX1 is set to ALTITUDEHOLDTRUE at the end of 'GroundTakeOff'
		//if(_controllerInput[AUX1] == ALTITUDEHOLDFALSE) 
		//{
		//GroundTakeOff();
		//GroundStart();
		//}

		DecideProgram();
		ApplyProgram(); 
		ApplySpeed();
	}

	ApplyData();
}

void ApplyData()
{
	// Apply receiver calibration adjustment
	for(byte channel = XAXIS; channel < channelsInUse; channel++)
	{
		receiverData[channel] = _controllerInput[channel]; //Only used by mavLink

		receiverCommandSmooth[channel] = filterSmooth(_controllerInput[channel], receiverCommandSmooth[channel], receiverSmoothFactor[channel]); //This is _controllerInput[channel]
	}

	// Reduce receiver commands using receiverXmitFactor and center around 1500
	for (byte channel = XAXIS; channel < THROTTLE; channel++) 
	{
		if(!_initialized)
			receiverCommand[channel] = _controllerInput[channel];
		else
			receiverCommand[channel] = ((receiverCommandSmooth[channel] - receiverZero[channel]) * receiverXmitFactor) + receiverZero[channel];
	}

	// No xmitFactor reduction applied for throttle, mode and AUX
	for (byte channel = THROTTLE; channel < channelsInUse; channel++) 
	{
		receiverCommand[channel] = _controllerInput[channel];
	}
}

const float getReceiverSIData(byte channel) 
{
	return ((receiverCommand[channel] - receiverZero[channel]) * (2.5 * PWM2RAD));  // +/- 2.5RPS 50% of full rate
}

void PrintReceiverOutput()
{
	//printInLine("Receiver input: ", STATUSMODE);
	//printInLine(receiverCommand[XAXIS], STATUSMODE);
	//printInLine(", ", STATUSMODE);
	//printInLine(receiverCommand[YAXIS], STATUSMODE);
	//printInLine(", ", STATUSMODE);
	//printInLine(receiverCommand[ZAXIS], STATUSMODE);
	//printInLine(", ", STATUSMODE);
	//printInLine(receiverCommand[THROTTLE], STATUSMODE);
	//printInLine(", ", STATUSMODE);
	//printInLine(IsMotorKilled(), STATUSMODE);
	//println(STATUSMODE);

	//printInLine("receiverCommandSmooth: ", STATUSMODE);
	//printInLine(receiverCommandSmooth[XAXIS], STATUSMODE);
	//printInLine(", ", STATUSMODE);
	//printInLine(receiverCommandSmooth[YAXIS], STATUSMODE);
	//printInLine(", ", STATUSMODE);
	//printInLine(receiverCommandSmooth[ZAXIS], STATUSMODE);
	//printInLine(", ", STATUSMODE);
	//printInLine(receiverCommandSmooth[THROTTLE], STATUSMODE);
	//printInLine(", ", STATUSMODE);
	//printInLine(receiverXmitFactor, STATUSMODE);
	//println(STATUSMODE);

	PrintMotorOutput();
}
