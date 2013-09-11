#ifndef _AQ_SERIAL_COMM_
#define _AQ_SERIAL_COMM_

#include "GlobalDefined.h"
#include "ControlFaker.h"
#include "../MotorControl/MotorControl.h"
#include "PrintDrone.h"

void initCommunication();
bool validateCalibrateCommand(byte command);
void readSerialPID(unsigned char PIDid);
void skipSerialValues(byte number);
void InvertUART();
void StopMotors();
void readSerialCommand();
void PrintValueComma(float val);
void PrintValueComma(double val);
void PrintValueComma(char val);
void PrintValueComma(int val);
void PrintValueComma(unsigned long val);
void PrintValueComma(byte val);
void PrintValueComma(long int val);
void PrintPID(unsigned char IDPid);
void PrintDummyValues(byte number);
float getHeading();
void sendSerialTelemetry();
void readValueSerial(char *data, byte size);
float readFloatSerial();
long readIntegerSerial();
void comma();

#endif