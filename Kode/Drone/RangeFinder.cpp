#include "RangeFinder.h"

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
