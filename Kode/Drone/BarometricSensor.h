#ifndef _AQ_BAROMETRIC_SENSOR_
#define _AQ_BAROMETRIC_SENSOR_

#include <Arduino.h>
#include "GlobalDefined.h"

extern float baroAltitude; 
extern float baroRawAltitude;
extern float baroGroundAltitude;
extern float baroSmoothFactor;
  
// **********************************************************************
// The following function calls must be defined inside any new subclasses
// **********************************************************************
void initializeBaro(); 
void measureBaro();
void measureBaroSum();
void evaluateBaroAltitude();
  
// *********************************************************
// The following functions are common between all subclasses
// *********************************************************
const float getBaroAltitude();
 
void measureGroundBaro();

#endif