#ifndef _AEROQUAD_RECEIVER_MEGA_H_
#define _AEROQUAD_RECEIVER_MEGA_H_


#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

#include "Arduino.h"
#include "Receiver.h"
#include "pins_arduino.h"
#include "AQMath.h"
#include "GlobalDefined.h"

#define RISING_EDGE 1
#define FALLING_EDGE 0
#define MINONWIDTH 950
#define MAXONWIDTH 2075
#define MINOFFWIDTH 12000
#define MAXOFFWIDTH 24000


volatile uint8_t *port_to_pcmask[] = {
	&PCMSK0,
	&PCMSK1,
	&PCMSK2
};
volatile static uint8_t PCintLast[3];
// Channel data
typedef struct {
	byte edge;
	unsigned long riseTime;
	unsigned long fallTime;
	unsigned int lastGoodWidth;
} tPinTimingData;
volatile static tPinTimingData pinData[9];

static void MegaPcIntISR();


SIGNAL(PCINT2_vect) {
	MegaPcIntISR();
}

#ifdef OLD_RECEIVER_PIN_ORDER
// arduino pins 67, 65, 64, 66, 63, 62
static byte receiverPin[6] = {5, 3, 2, 4, 1, 0}; // bit number of PORTK used for XAXIS, YAXIS, ZAXIS, THROTTLE, MODE, AUX
#else
//arduino pins 63, 64, 65, 62, 66, 67
static byte receiverPin[8] = {1, 2, 3, 0, 4, 5, 6, 7}; // bit number of PORTK used for XAXIS, YAXIS, ZAXIS, THROTTLE, MODE, AUX
#endif

void initializeReceiver(int nbChannel = 6);

int getRawChannelValue(byte channel);

void setChannelValue(byte channel,int value);

#endif

#endif



