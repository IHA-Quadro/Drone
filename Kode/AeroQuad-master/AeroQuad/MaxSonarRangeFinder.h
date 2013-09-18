#ifndef _AEROQUAD_MAX_SONAR_RANGE_FINDER_H_
#define _AEROQUAD_MAX_SONAR_RANGE_FINDER_H_


#include "InoHelper.h"
#include "RangeFinder.h"

#define MB1000 0 // Maxbotix LV-MaxSonar-EZ*
#define MB1200 1 // Maxbotix XL-MaxSonar-EZ*

#define SPIKE_FILTER_MARGIN 500 // mm ; changes bigger than this need two samples to take effect

struct rangeFinder {
	byte target;      // {ALTITUDE,FRONT,RIGHT,REAR,LEFT}_RANGE_FINDER_INDEX
	byte pin;
	byte triggerpin;
	byte type;
} rangeFinders[] = {
	// Define your rangers here
	// First ranger is given priority so it should be used for altitude
	// If using more than one ranger you should connect the 'trigger' to the 'RX' pin on the ranger.
	//
	//    { ALTITUDE_RANGE_FINDER_INDEX, A1, 24, MB1200}, 
	{ ALTITUDE_RANGE_FINDER_INDEX, A1, 0, MB1000},
	//	  { FRONT_RANGE_FINDER_INDEX,    A2, 25, MB1000},
	//	  { RIGHT_RANGE_FINDER_INDEX,    A3, 26, MB1000},
	//	  { REAR_RANGE_FINDER_INDEX,     A4, 27, MB1000},
	//	  { LEFT_RANGE_FINDER_INDEX,     A5, 28, MB1000}
};

// theoretical range at AIN=VCC
short rangerScale[] = { 
	13005, // MB10xx series
	10240, // MB12xx series
};

// 50Hz cycles needed to wait for ranging
byte rangerWait[] = {
	2, // MB1000 needs 50ms i.e. wait 2 cycles (60ms)
	4, // MB1200 needs 100ms i.e. wait 5 cycles (100ms)
};

#define RANGER_COUNT ((sizeof(rangeFinders) / sizeof(struct rangeFinder)))

// last reading used for 'spike' filtter
short lastRange[RANGER_COUNT];

byte rangerWaitCycles = 0;

byte rangerSchedule = 0;

void inititalizeRangeFinders();
void updateRangeFinders();

#endif 