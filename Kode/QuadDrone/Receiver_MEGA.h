#ifndef _AEROQUAD_RECEIVER_MEGA_H_
#define _AEROQUAD_RECEIVER_MEGA_H_

#include <Arduino.h>
#include <pins_arduino.h>

#include "AQMath.h"
#include "GlobalDefined.h"
#include "Receiver.h"

#define RISING_EDGE 1
#define FALLING_EDGE 0
#define MINONWIDTH 950
#define MAXONWIDTH 2075
#define MINOFFWIDTH 12000
#define MAXOFFWIDTH 24000


//arduino pins 63, 64, 65, 62, 66, 67
extern  byte receiverPin[8]; // bit number of PORTK used for XAXIS, YAXIS, ZAXIS, THROTTLE, MODE, AUX

// Channel data
typedef struct {
	byte edge;
	unsigned long riseTime;
	unsigned long fallTime;
	unsigned int lastGoodWidth;
} tPinTimingData;
volatile static tPinTimingData pinData[9];

void MegaPcIntISR();

//SIGNAL(PCINT2_vect) 
//{
//	MegaPcIntISR();
//}

void initializeReceiver(int nbChannel = 6);

int getRawChannelValue(byte channel);

void setChannelValue(byte channel,int value);


#endif