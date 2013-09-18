#include <Wire.h>

#include "Device_I2C.h"

void sendByteI2C(int deviceAddress, byte dataValue) {

  Wire.beginTransmission(deviceAddress);
  Wire.write(dataValue);
  Wire.endTransmission();
}

byte readByteI2C() {
  return Wire.read();
}

byte readByteI2C(int deviceAddress) {

  Wire.requestFrom(deviceAddress, 1);
  return Wire.read();
}

int readWordI2C(int deviceAddress) {

  Wire.requestFrom(deviceAddress, 2);
  return (Wire.read() << 8) | Wire.read();
}

int readWordI2C() {

  return (Wire.read() << 8) | Wire.read();
}

int readShortI2C(int deviceAddress) {

  Wire.requestFrom(deviceAddress, 2);
  return readShortI2C();
}

int readShortI2C() {

  return (signed short)readWordI2C();
}

int readReverseShortI2C() {

  return (signed short)( Wire.read() | (Wire.read() << 8));
}

int readWordWaitI2C(int deviceAddress) {

  Wire.requestFrom(deviceAddress, 2); // request two bytes
  while(!Wire.available()); // wait until data available
  unsigned char msb = Wire.read();
  while(!Wire.available()); // wait until data available
  unsigned char lsb = Wire.read();
  return (((int)msb<<8) | ((int)lsb));
}

int readReverseWordI2C(int deviceAddress) {

  Wire.requestFrom(deviceAddress, 2);
  byte lowerByte = Wire.read();
  return (Wire.read() << 8) | lowerByte;
}

byte readWhoI2C(int deviceAddress) {

  // read the ID of the I2C device
  Wire.beginTransmission(deviceAddress);
  Wire.write((byte)0);
  Wire.endTransmission();
  delay(100);
  Wire.requestFrom(deviceAddress, 1);
  return Wire.read();
}

void updateRegisterI2C(int deviceAddress, byte dataAddress, byte dataValue) {

  Wire.beginTransmission(deviceAddress);
  Wire.write(dataAddress);
  Wire.write(dataValue);
  Wire.endTransmission();
}  

