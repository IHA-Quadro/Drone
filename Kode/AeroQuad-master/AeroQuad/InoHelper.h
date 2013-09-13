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

//********************************************************
//****************** SERIAL PORT DECLARATION *************
//********************************************************
#if defined(WirelessTelemetry) 
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define 
Serial3
#else    // force 328p to use the normal port
#define SERIAL_PORT Serial
#endif
#else  
#if defined(SERIAL_USES_USB)   // STM32 Maple
#define SERIAL_PORT SerialUSB
#undef BAUD
#define BAUD
#else
#endif
#endif

#define SERIAL_PORT Serial




void RangeFinderAssign();
void Process100HzAssign();

#endif