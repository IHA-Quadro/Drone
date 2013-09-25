// Special thanks for 1k space optimization update from Ala42
// http://aeroquad.com/showthread.php?1369-The-big-enhancement-addition-to-2.0-code&p=13359&viewfull=1#post13359

#ifndef _AQ_DATA_STORAGE_H_
#define _AQ_DATA_STORAGE_H_

#include <Arduino.h>
#include <EEPROM.h>

#include "BarometricSensor.h"
#include "MaxSonarRangeFinder.h"
#include "PID.h"

// Utilities for writing and reading from the EEPROM
float nvrReadFloat(int address);
void nvrWriteFloat(float value, int address);
long nvrReadLong(int address);
void nvrWriteLong(long value, int address);
void nvrReadPID(unsigned char IDPid, unsigned int IDEeprom);
void nvrWritePID(unsigned char IDPid, unsigned int IDEeprom);
void initializeEEPROM();
void readEEPROM();
void writeEEPROM();
void initSensorsZeroFromEEPROM();
void storeSensorsZeroToEEPROM();
void initReceiverFromEEPROM();

#endif // _AQ_DATA_STORAGE_H_

