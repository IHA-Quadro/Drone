#include "MaxSonarRangeFinder.h"

struct rangeFinder rangeFinders[]= {
	// First ranger is given priority so it should be used for altitude
	// If using more than one ranger you should connect the 'trigger' to the 'RX' pin on the ranger.
	{ ALTITUDE_RANGE_FINDER_INDEX, A1, 0, MB1000},
	//	  { FRONT_RANGE_FINDER_INDEX,    A2, 25, MB1000},
	//	  { RIGHT_RANGE_FINDER_INDEX,    A3, 26, MB1000},
	//	  { REAR_RANGE_FINDER_INDEX,     A4, 27, MB1000},
	//	  { LEFT_RANGE_FINDER_INDEX,     A5, 28, MB1000}
}; //TODO:Ret ved flere sonars

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

// last reading used for 'spike' filtter
short lastRange[RANGER_COUNT];

byte rangerWaitCycles = 0;

byte rangerSchedule = 0;


void inititalizeRangeFinders() {

  for (byte i = 0; i < RANGER_COUNT; i++) {

    rangeFinderRange[rangeFinders[i].target] = -1;
    if (rangeFinders[i].triggerpin) {
      digitalWrite(rangeFinders[i].triggerpin, LOW);
      pinMode(rangeFinders[i].triggerpin, OUTPUT);
    }
    lastRange[i] = 32000; 
    //    pinMode(rangeFinders[i].pin, INPUT);
  }
  rangerWaitCycles = 10; // allow to initialize
}

void updateRangeFinders() {

  byte rangerToRead = 0;
  byte rangerToTrigger = 0;

  if (rangerWaitCycles) {
    rangerWaitCycles--;
    return;
  }

  if (RANGER_COUNT > 1) {
    if ((rangerSchedule & 1) == 0) {
      rangerToRead = 0;
      rangerToTrigger = (rangerSchedule >> 1) + 1;
    }
    else {
      rangerToRead = (rangerSchedule >> 1) + 1;
      rangerToTrigger = 0;
    }
    rangerSchedule++;
    if (((rangerSchedule>>1) + 1) >= (byte)RANGER_COUNT) {
      rangerSchedule = 0;
    }
  }

  if (rangeFinders[rangerToTrigger].triggerpin) {
    digitalWrite(rangeFinders[rangerToTrigger].triggerpin, HIGH);
  }

  short range = (short)((long)analogRead(rangeFinders[rangerToRead].pin) * (long)(rangerScale[rangeFinders[rangerToRead].type]) / (1L<<ADC_NUMBER_OF_BITS));

  // Following will accept the sample if it's either withing "spike margin" of last raw reading or previous accepted reading
  // otherwise it's ignored as noise
  
  if ((abs(range - lastRange[rangerToRead]) < SPIKE_FILTER_MARGIN) ||
      (abs(range * 1000.0 - rangeFinderRange[rangeFinders[rangerToRead].target]) < SPIKE_FILTER_MARGIN)) {
    rangeFinderRange[rangeFinders[rangerToRead].target] = (float)range / 1000.0;
  }
  lastRange[rangerToRead] = range;
 
  rangerWaitCycles = rangerWait[rangeFinders[rangerToRead].type];

  if (rangeFinders[rangerToTrigger].triggerpin) {
    digitalWrite(rangeFinders[rangerToTrigger].triggerpin, LOW);
  }
}
