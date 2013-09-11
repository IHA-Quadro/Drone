/*
AeroQuad v3.0.1 - February 2012
www.AeroQuad.com
Copyright (c) 2012 Ted Carancho.  All rights reserved.
An Open Source Arduino based multicopter.

This program is free software: you can redistribute it and/or modify 
it under the terms of the GNU General Public License as published by 
the Free Software Foundation, either version 3 of the License, or 
(at your option) any later version. 

This program is distributed in the hope that it will be useful, 
but WITHOUT ANY WARRANTY; without even the implied warranty of 
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
GNU General Public License for more details. 

You should have received a copy of the GNU General Public License 
along with this program. If not, see <http://www.gnu.org/licenses/>. 
*/

#ifndef _AEROQUAD_RECEIVER_H_
#define _AEROQUAD_RECEIVER_H_

#include "Arduino.h"
#include "ControlFaker.h"
#include "AQMath.h"
#include "PrintDrone.h"

#define PWM2RAD 0.002 //  Based upon 5RAD for full stick movement, you take this times the RAD to get the PWM conversion factor

// Receiver variables
#define TIMEOUT 25000
#define MINCOMMAND 1000
#define MIDCOMMAND 1500
#define MAXCOMMAND 2000
#define MINDELTA 200
#define MINCHECK (MINCOMMAND + 100)
#define MAXCHECK (MAXCOMMAND - 100)
#define MINTHROTTLE (MINCOMMAND + 100)
#define LEVELOFF 100
#define MAX_NB_CHANNEL 10


int getRawChannelValue(byte channel);  
void readReceiver();
void FakeData();
void ApplyData();
void PrintStatus();

void initializeReceiverParam(int nbChannel = 6);
void readReceiver();
void ApplyData();
void PrintStatus();
void setChannelValue(byte channel,int value);
const float getReceiverSIData(byte channel);

#endif
