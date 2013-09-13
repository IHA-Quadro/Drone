#ifndef _AQ_MAVLINK_H_
#define _AQ_MAVLINK_H_

#define MAV_COMPONENT_ID MAV_COMP_ID_IMU

#ifndef MAV_SYSTEM_ID
  #define MAV_SYSTEM_ID 100
#endif

// MavLink 1.0 DKP
#include "../mavlink/include/mavlink/v1.0/common/mavlink.h"

#include "AeroQuad.h"
#include "BarometricSensor.h"
#include "DataStorage.h"
#include "Gyroscope.h"
#include "InoHelper.h"
#include "Kinematics.h"
#include "Receiver.h"
#include "UserConfiguration.h"

int systemType;
int autopilotType = MAV_AUTOPILOT_GENERIC;
uint16_t len;
int systemMode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
int systemStatus = MAV_STATE_UNINIT;

// Variables for writing and sending parameters

enum parameterTypeIndicator
{
  P,
  I,
  D,
  windUpGuard,
  NONE
};

int indexCounter = 0;
int paramListPartIndicator = -1;

int parameterChangeIndicator = -1;
int parameterMatch = 0;
mavlink_param_set_t set;
char* key;

int parameterType = MAVLINK_TYPE_FLOAT;
int parameterListSize;

const char* parameterNameRateRollP = "Rate Roll_P";
const char* parameterNameRateRollI = "Rate Roll_I";
const char* parameterNameRateRollD = "Rate Roll_D";
const char* parameterNameRatePitchP = "Rate Pitch_P";
const char* parameterNameRatePitchI = "Rate Pitch_I";
const char* parameterNameRatePitchD = "Rate Pitch_D";
const char* parameterNameAttitudeRollP = "Att Roll_P";
const char* parameterNameAttitudeRollI = "Att Roll_I";
const char* parameterNameAttitudeRollD = "Att Roll_D";
const char* parameterNameAttitudePitchP = "Att Pitch_P";
const char* parameterNameAttitudePitchI = "Att Pitch_I";
const char* parameterNameAttitudePitchD = "Att Pitch_D";
const char* parameterNameAttitudeGyroRollP = "AttGyroRoll_P";
const char* parameterNameAttitudeGyroRollI = "AttGyroRoll_I";
const char* parameterNameAttitudeGyroRollD = "AttGyroRoll_D";
const char* parameterNameAttitudeGyroPitchP = "AttGyroPitc_P";
const char* parameterNameAttitudeGyroPitchI = "AttGyroPitc_I";
const char* parameterNameAttitudeGyroPitchD = "AttGyroPitc_D";
const char* parameterNameYawP = "Yaw_P";
const char* parameterNameYawI = "Yaw_I";
const char* parameterNameYawD = "Yaw_D";
const char* parameterNameHeadingP = "Heading_P";
const char* parameterNameHeadingI = "Heading_I";
const char* parameterNameHeadingD = "Heading_D";
const char* parameterNameHeadingConfig = "Heading_Conf";
const char* parameterNameAREF = "Misc_AREF";
const char* parameterNameMinThrottle = "Misc_MinThr";
const char* parameterNameTxFactor = "TX_TX Factor";
const char* parameterNameTxRollSmooth = "TX_RollSmooth";
const char* parameterNameTxPitchSmooth = "TX_PitcSmooth";
const char* parameterNameTxYawSmooth = "TX_YawSmooth";
const char* parameterNameTxThrottleSmooth = "TX_ThrSmooth";
const char* parameterNameTxModeSmooth = "TX_ModeSmooth";
const char* parameterNameTxAUX1Smooth = "TX_AUX1Smooth";
const char* parameterNameTxAUX2Smooth = "TX_AUX2Smooth";
const char* parameterNameTxAUX3Smooth = "TX_AUX3Smooth";
const char* parameterNameTxAUX4Smooth = "TX_AUX4Smooth";
const char* parameterNameTxAUX5Smooth = "TX_AUX5Smooth";
#if defined(BattMonitor)
  const char* parameterNameBattMonAlarmVoltage = "BatMo_AlarmVo";
  const char* parameterNameBattMonThrottleTarget = "BatMo_ThrTarg";
  const char* parameterNameBattMonGoingDownTime = "BatMo_DownTim";
#endif
#if defined(CameraControl)
  const char* parameterNameCamMode = "Cam_Mode";
  const char* parameterNameCamPitchMiddle = "Cam_PitchMid";
  const char* parameterNameCamRollMiddle = "Cam_RollMid";
  const char* parameterNameCamYawMiddle = "Cam_YawMid";
  const char* parameterNameCamRollServoMiddle = "Cam_ServoPitM";
  const char* parameterNameCamPitchServoMiddle = "Cam_ServoRolM";
  const char* parameterNameCamYawServoMiddle = "Cam_ServoYawM";
  const char* parameterNameCamPitchServoMin = "Cam_SerMinPit";
  const char* parameterNameCamRollServoMin = "Cam_SerMinRol";
  const char* parameterNameCamYawServoMin = "Cam_SerMinYaw";
  const char* parameterNameCamPitchServoMax = "Cam_SerMaxPit";
  const char* parameterNameCamRollServoMax = "Cam_SerMaxRol";
  const char* parameterNameCamYawServoMax = "Cam_SerMaxYaw";
#endif
#if defined(AltitudeHoldBaro) || defined(AltitudeHoldRangeFinder)
  const char* parameterNameAHminThrottleAdjust = "AH_Min Adjust";
  const char* parameterNameAHmaxThrottleAdjust = "AH_Max Adjust";
  const char* parameterNameAHBumpValue = "AH_Bump Value";
  const char* parameterNameAHPanicValue = "AH_PanicValue";
#endif
#if defined(AltitudeHoldBaro)
  const char* parameterNameAHBaroSmooth = "AH_SmoothFact";
  const char* parameterNameBaroP = "Baro_P";
  const char* parameterNameBaroI = "Baro_I";
  const char* parameterNameBaroD = "Baro_D";
  const char* parameterNameBaroWindUpGuard = "Baro_WindUp";
  const char* parameterNameZDampeningP = "Z Dampening_P";
  const char* parameterNameZDampeningI = "Z Dampening_I";
  const char* parameterNameZDampeningD = "Z Dampening_D";
#endif
#if defined(AltitudeHoldRangeFinder)
  const char* parameterNameRangeFinderP = "Range_P";
  const char* parameterNameRangeFinderI = "Range_I";
  const char* parameterNameRangeFinderD = "Range_D";
  const char* parameterNameRangeFinderWindUpGuard = "Range_WindUp";
#endif
#if defined(UseGPSNavigator)
  const char* parameterNameGPSRollP = "GPS Roll_P";
  const char* parameterNameGPSRollI = "GPS Roll_I";
  const char* parameterNameGPSRollD = "GPS Roll_D";
  const char* parameterNameGPSPitchP = "GPS Pitch_P";
  const char* parameterNameGPSPitchI = "GPS Pitch_I";
  const char* parameterNameGPSPitchD = "GPS Pitch_D";
  const char* parameterNameGPSYawP = "GPS Yaw_P";
  const char* parameterNameGPSYawI = "GPS Yaw_I";
  const char* parameterNameGPSYawD = "GPS Yaw_D";
#endif

parameterTypeIndicator paramIndicator = NONE;
float *parameterToBeChangedFloat;
byte *parameterToBeChangedByte;
int *parameterToBeChangedInt;
unsigned long *parameterToBeChangedULong;

static uint16_t millisecondsSinceBoot = 0;
long system_dropped_packets = 0;

mavlink_message_t msg;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];
mavlink_status_t status;




void evaluateParameterListSize();
void evaluateCopterType();
void initCommunication();
void updateFlightTime();
void sendSerialHeartbeat();
void sendSerialRawIMU();
void sendSerialAttitude();
void sendSerialHudData();
void sendSerialGpsPostion();
void sendSerialRawPressure();
void sendSerialRcRaw();
void sendSerialSysStatus();

void sendSerialPID(int IDPid, int8_t id_p[], int8_t id_i[], int8_t id_d[], int8_t id_windUp[], int listsize, int index);
void sendSerialParameter(float parameterID, int8_t parameterName[], int listsize, int index);
void sendSerialParameter(int parameterID, int8_t parameterName[], int listsize, int index);
void sendSerialParameter(byte parameterID, int8_t parameterName[], int listsize, int index);
void sendSerialParameter(unsigned long parameterID, int8_t parameterName[], int listsize, int index);

void sendParameterListPart1();
void sendParameterListPart2();
void sendParameterListPart3();
void sendParameterListPart4();
void sendParameterListPart5();

bool checkParameterMatch(const char* parameterName, char* key);
int findParameter(char* key);
void changeAndSendParameter();
void readSerialCommand();
void sendQueuedParameters();
void sendSerialVehicleData();
void sendSerialTelemetry();
#endif //#define _AQ_MAVLINK_H_

