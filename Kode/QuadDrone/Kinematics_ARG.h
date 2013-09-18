#ifndef _AQ_KINEMATICS_ARG_
#define _AQ_KINEMATICS_ARG_

//=====================================================================================================
// IMU.c
// S.O.H. Madgwick
// 25th September 2010
//=====================================================================================================
// Description:
//
// Quaternion implementation of the 'DCM filter' [Mayhony et al].
//
// User must define 'halfT' as the (sample period / 2), and the filter gains 'Kp' and 'Ki'.
//
// Global variables 'q0', 'q1', 'q2', 'q3' are the quaternion elements representing the estimated
// orientation.  See my report for an overview of the use of quaternions in this application.
//
// User must call 'IMUupdate()' every sample period and parse calibrated gyroscope ('gx', 'gy', 'gz')
// and accelerometer ('ax', 'ay', 'ay') data.  Gyroscope units are radians/second, accelerometer 
// units are irrelevant as the vector is normalised.
//
//=====================================================================================================

#include "AQMath.h"
#include "Kinematics.h"


extern float Kp;	// proportional gain governs rate of convergence to accelerometer/magnetometer
extern float Ki; // integral gain governs rate of convergence of gyroscope biases
extern float halfT; // half the sample period
extern float q0, q1, q2, q3;	// quaternion elements representing the estimated orientation
extern float exInt, eyInt, ezInt;	// scaled integral error

extern float previousEx;
extern float previousEy;
extern float previousEz;


void argUpdate(float gx, float gy, float gz, float ax, float ay, float az, float G_Dt);
void eulerAngles();
void initializeKinematics();
void calculateKinematics(float rollRate, float pitchRate, float yawRate, float longitudinalAccel, float lateralAccel, float verticalAccel, float G_DT);
float getGyroUnbias(byte axis);
void calibrateKinematics();
void ResetKinematicData();
#endif