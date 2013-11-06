#ifndef _AEROQUAD_RECEIVER_H_
#define _AEROQUAD_RECEIVER_H_

#include <Arduino.h>

#include "AQMath.h"
#include "ControlFaker.h"
#include "GlobalDefined.h"

#define PWM2RAD 0.002 //  Based upon 5RAD for full stick movement, you take this times the RAD to get the PWM conversion factor

// Receiver variables
#define TIMEOUT 25000
#define MINCOMMAND 1000
#define MIDCOMMAND 1500
#define MAXCOMMAND 2000
#define MINDELTA 200
#define MINCHECK (MINCOMMAND + 100)
#define MAXCHECK (MAXCOMMAND - 100)
#define LEVELOFF 100
#define channelsInUse 8

extern int lastReceiverChannel;
extern float receiverXmitFactor;
extern int receiverData[MAX_NB_CHANNEL];
extern int receiverZero[3];
extern int receiverCommand[MAX_NB_CHANNEL];
extern int receiverCommandSmooth[MAX_NB_CHANNEL];
extern float receiverSlope[MAX_NB_CHANNEL];
extern float receiverOffset[MAX_NB_CHANNEL];
extern float receiverSmoothFactor[MAX_NB_CHANNEL];
extern int channelCal;

void initializeReceiverParam(int nbChannel);
int getRawChannelValue(byte channel);  
void readReceiver();
void initMotor();
void setChannelValue(byte channel,int value);
void ApplyData();

void PrintMotorOutput();
void PrintReceiverOutput();

// return the smoothed & scaled number of radians/sec in stick movement - zero centered
const float getReceiverSIData(byte channel);
	
#endif