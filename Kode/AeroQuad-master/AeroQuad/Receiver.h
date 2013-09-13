#ifndef _AEROQUAD_RECEIVER_H_
#define _AEROQUAD_RECEIVER_H_

#include <Arduino.h>

#include "ControlFaker.h"
#include "AQMath.h"
#include "PrintDrone.h"

#define PWM2RAD 0.002 //  Based upon 5RAD for full stick movement, you take this times the RAD to get the PWM conversion factor

// Receiver variables
#define TIMEOUT 25000
#define MINCOMMAND 1000
#define MIDCOMMAND 1500
#define MAXCOMMAND 2000
#define MINDELTA 200
#define MINCHECK (MINCOMMAND + 100)
#define MAXCHECK (MAXCOMMAND - 100)
#define MINTHROTTLE (MINCOMMAND + 100)
#define LEVELOFF 100
#define MAX_NB_CHANNEL 10

int lastReceiverChannel = 0;
int channelCal;

float receiverXmitFactor;
int receiverData[MAX_NB_CHANNEL];
int receiverZero[3];
int receiverCommand[MAX_NB_CHANNEL]; 
int receiverCommandSmooth[MAX_NB_CHANNEL];
float receiverSlope[MAX_NB_CHANNEL];
float receiverOffset[MAX_NB_CHANNEL];
float receiverSmoothFactor[MAX_NB_CHANNEL];

int getRawChannelValue(byte channel);  
void readReceiver();
void FakeData();
void ApplyData();
void PrintStatus();

void initializeReceiverParam(int nbChannel);
void readReceiver();
void ApplyData();
void PrintStatus();
void setChannelValue(byte channel,int value);
const float getReceiverSIData(byte channel);
void initializeReceiverValues();

#endif
