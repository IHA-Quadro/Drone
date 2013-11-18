#ifndef _AEROQUAD_RANGE_FINDER_H_
#define _AEROQUAD_RANGE_FINDER_H_

#include <Arduino.h>

#define INVALID_RANGE -1
#define MISSING_RANGE -2

#define ALTITUDE_RANGE_FINDER_INDEX 0
#define FRONT_RANGE_FINDER_INDEX    1
#define RIGHT_RANGE_FINDER_INDEX    2
#define REAR_RANGE_FINDER_INDEX     4
#define LEFT_RANGE_FINDER_INDEX     3

extern float rangeFinderRange[5];
extern float maxRangeFinderRange;
extern float minRangeFinderRange;


void inititalizeRangeFinders();
void updateRangeFinders();

bool isOnRangerRange(float distance);
bool isRangerPresent(byte idx);

#endif