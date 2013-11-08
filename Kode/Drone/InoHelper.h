#ifndef InoHelper_H
#define InoHelper_H

#include "Accelerometer.h"
#include "AltitudeControlProcessor.h"
#include "AeroQuad.h"
#include "FlightCommandProcessor.h"
#include "FlightControlProcessor.h"
#include "FourtOrderFilter.h"
#include "MaxSonarRangeFinder.h"
#include "UserConfiguration.h"


//********************************************************
//****************** SERIAL PORT DECLARATION *************
//********************************************************
//#if defined(WirelessTelemetry) 
//#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
//#define SERIAL_PORT Serial3
//#else    // force 328p to use the normal port
//#define SERIAL_PORT Serial
//#endif
//#else  
//#if defined(SERIAL_USES_USB)   // STM32 Maple
//#define SERIAL_PORT SerialUSB
//#undef BAUD
//#define BAUD
//#else
#define SERIAL_PORT Serial
//#endif
//#endif  


void process100HzTask();
void process50HzTask();
void process10HzTask1();
void process10HzTask2();
void process10HzTask3();
void process5HzTask();
void process1HzTask();

void PrintAltitudeReport();
void PrintDebugReport();
void PrintSonarReport();
void PrintChosenProgram();


// called when eeprom is initialized
void initializePlatformSpecificAccelCalibration();
#endif
