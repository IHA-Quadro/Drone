#ifndef _RECEIVECOMMANDTESTDATA_h
#define _RECEIVECOMMANDTESTDATA_h

#include "Arduino.h"
#include "ControlFaker.h"

#define lower 1450
#define upper 1550
#define steady 1500

void TestAxis(byte axis);
void RotateDrone(int zaxis);
void MoveDrone(int xaxis, int yaxis);
void ThrottleDrone(int throttle);

#endif

