#include "RangeFinder.h"

float rangeFinderRange[5]      = {-2,-2,-2,-2,-2};

float maxRangeFinderRange = 4.5;
float minRangeFinderRange = 0.0;


void inititalizeRangeFinders();
void updateRangeFinders();

bool isOnRangerRange(float distance) {

  return (distance >= minRangeFinderRange) && (distance <= maxRangeFinderRange);
}

bool isRangerPresent(byte idx) {
  
  return (rangeFinderRange[idx] > -2);
}
