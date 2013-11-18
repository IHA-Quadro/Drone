#ifndef _AEROQUAD_DEVICE_I2C_H_
#define _AEROQUAD_DEVICE_I2C_H_

#include <Arduino.h>
#include <Wire.h>

void sendByteI2C(int deviceAddress, byte dataValue);

byte readByteI2C();
byte readByteI2C(int deviceAddress);
int readWordI2C(int deviceAddress);
int readWordI2C();
int readShortI2C(int deviceAddress);
int readShortI2C();
int readReverseShortI2C();
int readWordWaitI2C(int deviceAddress);
int readReverseWordI2C(int deviceAddress);
byte readWhoI2C(int deviceAddress);
void updateRegisterI2C(int deviceAddress, byte dataAddress, byte dataValue);

#endif