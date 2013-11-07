#ifndef RADIO_COMMUNICATION
#define RADIO_COMMUNICATION

#include <Arduino.h>
#include <Wire.h>

#include "PrintDrone.h"
#include "QueueList.h"

#define TRANCEIVER_ADDRESS 0x90
#define INDICATORVALUE 0xAA

extern QueueList<int> _queue;
extern QueueList<int> _rssiQueue;
extern int radioProgram;

void SetupRadioCommunicaiton();
//QueueList<int> GetQueueData();
int ReadRadio();
void receiveEvent(int howMany);


#endif