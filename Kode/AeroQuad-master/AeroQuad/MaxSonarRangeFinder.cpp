#include "MaxSonarRangeFinder.h"

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
