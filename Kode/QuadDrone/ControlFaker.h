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

extern boolean _initialized;
extern boolean _calibrationPerformed;
extern boolean _motorsArmed;
extern boolean _safetyChecked;

extern int _controllerInput[channelsInUse];

void DronePrint(String s);
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