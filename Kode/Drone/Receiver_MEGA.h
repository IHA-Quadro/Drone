#ifndef _AEROQUAD_RECEIVER_MEGA_H_
#define _AEROQUAD_RECEIVER_MEGA_H_


#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

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

void MegaPcIntISR();

extern volatile uint8_t *port_to_pcmask[];

volatile static uint8_t PCintLast[3];

// Channel data
typedef struct {
	byte edge;
	unsigned long riseTime;
	unsigned long fallTime;
	unsigned int lastGoodWidth;
} tPinTimingData;

volatile static tPinTimingData pinData[9];

//SIGNAL(PCINT2_vect) {
//	MegaPcIntISR();
//}

void initializeReceiver(int nbChannel);

int getRawChannelValue(byte channel);

void setChannelValue(byte channel,int value);

#endif

#endif



