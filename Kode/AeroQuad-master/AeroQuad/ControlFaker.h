#ifndef _CONTROLFAKER_h
#define _CONTROLFAKER_h

#include "Arduino.h"
#include "GlobalDefined.h"
#include "Motors.h"
#include "Accelerometer.h"
#include "Gyroscope.h"
#include "Kinematics.h"
#include "../MotorControl/MotorControl.h"

#define channelsInUse 6

void SetupControlFaker();
void KillMotor();
void FMSignal();
void SelectProgram();
void SonarCheck();
void CalculateAltitude();
void ApplyHeading();
void ArmMotors();
void SafetyCheck();
void PerformCalibration();
void ApplySpeed();
void ResetInputData();

void RotateDrone(int value);
void MoveDrone(int xaxis, int yaxis);
void ThrottleDrone(int throttle);

#endif