#ifndef _AQ_HEADING_CONTROL_PROCESSOR_H_
#define _AQ_HEADING_CONTROL_PROCESSOR_H_

#include <Arduino.h>

#include "AeroQuad.h"
#include "FlightControlVariable.h"
#include "GlobalDefined.h"
#include "Gyroscope.h"
#include "PID.h"
#include "Receiver.h"

extern float setHeading;
extern unsigned long headingTime;

void processHeading();


#endif

