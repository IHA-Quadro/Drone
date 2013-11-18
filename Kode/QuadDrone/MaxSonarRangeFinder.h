#ifndef _AEROQUAD_MAX_SONAR_RANGE_FINDER_H_
#define _AEROQUAD_MAX_SONAR_RANGE_FINDER_H_

#include <Arduino.h>

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
};
extern struct rangeFinder rangeFinders[]; 

// theoretical range at AIN=VCC
extern short rangerScale[];

// 50Hz cycles needed to wait for ranging
extern byte rangerWait[];

#define RANGER_COUNT ((sizeof(rangeFinders) / sizeof(struct rangeFinder)))

// last reading used for 'spike' filtter
extern short lastRange[];

extern byte rangerWaitCycles;

extern byte rangerSchedule;

void inititalizeRangeFinders();
void updateRangeFinders();

#endif 