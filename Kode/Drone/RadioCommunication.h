#ifndef RADIO_COMMUNICATION
#define RADIO_COMMUNICATION

#include <Arduino.h>
#include <Wire.h>

#include "PrintDrone.h"
#include "QueueList.h"

#define TRANCEIVER_ADDRESS 0x90
#define PROGRAM_IDENTIFIER 254
#define RSSI_IDENTIFIER 200

extern QueueList<int> _queue;
extern QueueList<int> _rssiQueue;
extern int radioProgram;

void SetupRadioCommunicaiton();
QueueList<int> GetQueueData();
void receiveEvent(int howMany);


#endif