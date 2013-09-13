// FlightControl.pde is responsible for combining sensor measurements and
// transmitter commands into motor commands for the defined flight configuration (X, +, etc.)
#ifndef _AQ_PROCESS_FLIGHT_CONTROL_H_
#define _AQ_PROCESS_FLIGHT_CONTROL_H_

#include "AeroQuad.h"
#include "FlightControlQuadPlus.h"
#include "Kinematics.h"
#include "PID.h"
#include "Receiver.h"

#define ATTITUDE_SCALING (0.75 * PWM2RAD)


/**
 * calculateFlightError
 *
 * Calculate roll/pitch axis error with gyro/accel data to
 * compute motor command thrust so used command are executed
 */
void calculateFlightError();

/**
 * processCalibrateESC
 * 
 * Proces esc calibration command with the help of the configurator
 */
void processCalibrateESC();


#if defined AutoLanding
  #define BARO_AUTO_LANDING_DESCENT_SPEED 0.008
  #define SONAR_AUTO_LANDING_DESCENT_SPEED 0.005
  void processAutoLandingAltitudeCorrection();
#endif

/**
 * processThrottleCorrection
 * 
 * This function will add some throttle imput if the craft is angled
 * this prevent the craft to loose altitude when angled.
 * it also add the battery throttle correction in case
 * of we are in auto-descent.
 * 
 * Special thank to Ziojo for this.
 */
void processThrottleCorrection();

/**
 * processHardManuevers
 *
 * In case of a roll/pitch stick at one edge to do a loop, this function
 * will prevent the lower throttle motor side to have too much low throtte.
 */
void processHardManuevers();

/**
 * processMinMaxCommand
 *
 * This function correct too low/max throttle when manuevering
 * preventing some wobbling behavior
 */
void processMinMaxCommand();


/**
 * processFlightControl
 *
 * Main flight control processos function
 */
void processFlightControl();

#endif 