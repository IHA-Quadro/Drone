#ifndef _AQ_SERIAL_COMM_
#define _AQ_SERIAL_COMM_

#include "AeroQuad.h"
#include "BarometricSensor.h"
#include "DataStorage.h"
#include "GlobalDefined.h"
#include "ControlFaker.h"
#include "FlightControlQuadPlus.h"
#include "InoHelper.h"
#include "Kinematics.h"
#include "MotorControl.h"
#include "PID.h"
#include "PrintDrone.h"
#include "Receiver.h"
#include "SensorsStatus.h"
#include "UserConfiguration.h"

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