#ifndef _CONTROLFAKER_h
#define _CONTROLFAKER_h

#include "Accelerometer.h"
#include "GlobalDefined.h"
#include "Gyroscope.h"
#include "Kinematics.h"
#include "MotorControl.h"
#include "Motors.h"
#include "PrintDrone.h"

#define channelsInUse 6

extern bool _initialized;
extern bool _calibrationPerformed;
extern bool _motorsArmed;
extern bool _safetyChecked;

extern int maxSpinSpeed;
extern int spinSpeed;
extern int _controllerInput[channelsInUse];

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
void PrintMotorOutput();

void RotateDrone(int value);
void MoveDrone(int xaxis, int yaxis);
void ThrottleDrone(int throttle);

void AeroQuadSetup();

#endif