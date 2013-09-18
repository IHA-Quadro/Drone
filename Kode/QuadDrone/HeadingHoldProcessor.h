#ifndef _AQ_HEADING_CONTROL_PROCESSOR_H_
#define _AQ_HEADING_CONTROL_PROCESSOR_H_

#include "AeroQuad.h"
#include "FlightControlProcessor.h"
#include "PID.h"

float setHeading;
unsigned long headingTime;

void InitializeHeadingHoldProcessor();
void processHeading();
void ResetHeadingData();

#endif