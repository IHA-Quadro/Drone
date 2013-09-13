#ifndef _AEROQUAD_RANGE_FINDER_H_
#define _AEROQUAD_RANGE_FINDER_H_

#include <Arduino.h>

#define INVALID_RANGE -1
#define MISSING_RANGE -2

#define ALTITUDE_RANGE_FINDER_INDEX 0
#define FRONT_RANGE_FINDER_INDEX    1
#define RIGHT_RANGE_FINDER_INDEX    2
#define REAR_RANGE_FINDER_INDEX     3
#define LEFT_RANGE_FINDER_INDEX     4

float rangeFinderRange[5]      = {-2,-2,-2,-2,-2};

float maxRangeFinderRange = 4.5;
float minRangeFinderRange = 0.0;


void    inititalizeRangeFinders();
void    updateRangeFinders();

boolean isOnRangerRange(float distance) {

  return (distance >= minRangeFinderRange) && (distance <= maxRangeFinderRange);
}

boolean isRangerPresent(byte idx) {
  
  return (rangeFinderRange[idx] > -2);
}

#endif //  #ifdef _AEROQUAD_RANGE_FINDER_H_
