#ifndef _AEROQUAD_MAX_SONAR_RANGE_FINDER_H_
#define _AEROQUAD_MAX_SONAR_RANGE_FINDER_H_

// @see http://www.arduino.cc/playground/Main/MaxSonar

#include "InoHelper.h"
#include "RangeFinder.h"


//#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(BOARD_aeroquad32)

#define MB1000 0 // Maxbotix LV-MaxSonar-EZ*
#define MB1200 1 // Maxbotix XL-MaxSonar-EZ*
#define RANGEFINDERSIZE 4
#define RANGERARRAYSIZE 8 //read 50 times, half is to bottom (25 left) and divide by three sensors = 8.333

#define SPIKE_FILTER_MARGIN 500 // mm ; changes bigger than this need two samples to take effect

struct rangeFinder {
    byte target;      // {ALTITUDE,FRONT,RIGHT,REAR,LEFT}_RANGE_FINDER_INDEX
    byte pin;
    byte triggerpin;
    byte type;
  };

extern struct rangeFinder rangeFinders[RANGEFINDERSIZE];

// theoretical range at AIN=VCC
extern short rangerScale[];

// 50Hz cycles needed to wait for ranging
extern byte rangerWait[];

#define RANGER_COUNT ((sizeof(rangeFinders) / sizeof(struct rangeFinder)))

// last reading used for 'spike' filtter
extern short lastRange[RANGER_COUNT];
extern byte rangerWaitCycles;
extern byte rangerSchedule;

struct RangerArray
{
	short data[RANGERARRAYSIZE];
	int counter;
	float average;
};

extern struct RangerArray RangerAverage[RANGER_COUNT];

void inititalizeRangeFinders();
void updateRangeFinders();
void StoreRangeValues();

//#endif 
#endif