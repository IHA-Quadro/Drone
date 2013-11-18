#ifndef _INOHELPER_H_
#define _INOHELPER_H_

#include "AeroQuad.h"
#include "MaxSonarRangeFinder.h"
#include "PID.h"
#include "SensorsStatus.h"

// default to 10bit ADC (AVR)
#ifndef ADC_NUMBER_OF_BITS
#define ADC_NUMBER_OF_BITS 10
#endif


#define LED_Green 13
#define LED_Red 4
#define LED_Yellow 31

#define SERIAL_PORT Serial

void RangeFinderAssign();
void Process100HzAssign();

#endif