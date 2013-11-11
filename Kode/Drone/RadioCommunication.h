#ifndef RADIO_COMMUNICATION
#define RADIO_COMMUNICATION

#include <Arduino.h>
#include <Wire.h>

#include "PrintDrone.h"

extern int radioProgram;

void SetupRadioCommunicaiton();
void ReadRadio();


#endif