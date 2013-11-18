// FlightControl.pde is responsible for combining sensor measurements and
// transmitter commands into motor commands for the defined flight configuration (X, +, etc.)

#ifndef _AQ_PROCESS_FLIGHT_CONTROL_H_
#define _AQ_PROCESS_FLIGHT_CONTROL_H_

#include "AeroQuad.h"
#include "AltitudeControlProcessor.h"
#include "FlightControlQuadPlus.h"
#include "FlightControlVariable.h"
#include "Gyroscope.h"
#include "Kinematics.h"
#include "UserConfiguration.h"

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

/**
 * processBatteryMonitorThrottleAdjustment
 *
 * Check battery alarm and if in alarm, increment a counter
 * When this counter reach BATTERY_MONITOR_MAX_ALARM_COUNT, then
 * we are now in auto-descent mode.
 *
 * When in auto-descent mode, the user can pass throttle keep when the
 * alarm was reach, and the throttle is slowly decrease for a minute til
 * batteryMonitorThrottle that is configurable with the configurator
 */
//#if defined BattMonitor && defined BattMonitorAutoDescent
//  void processBatteryMonitorThrottleAdjustment() {
//    
//    if (batteryMonitorAlarmCounter < BATTERY_MONITOR_MAX_ALARM_COUNT) {
//      if (batteryAlarm) {
//        batteryMonitorAlarmCounter++;
//      }
//    }
//    else {
//      #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
//        if (altitudeHoldState == ON) {
//          #if defined AltitudeHoldBaro
//            baroAltitudeToHoldTarget -= 0.01;
//          #endif
//          #if defined AltitudeHoldRangeFinder
//            if (sonarAltitudeToHoldTarget != INVALID_RANGE) {
//              sonarAltitudeToHoldTarget -= 0.01;
//            }
//          #endif
//        }
//        else {
//      #endif
//          if (batteryMonitorStartThrottle == 0) {  // init battery monitor throttle correction!
//            batteryMonitorStartTime = millis();
//            if (throttle < batteryMonitorThrottleTarget) {
//              batteryMonitorStartThrottle = batteryMonitorThrottleTarget;
//            }
//            else {
//              batteryMonitorStartThrottle = throttle; 
//            }
//          }
//          int batteryMonitorThrottle = map(millis()-batteryMonitorStartTime, 0, batteryMonitorGoingDownTime, batteryMonitorStartThrottle, batteryMonitorThrottleTarget);
//          if (batteryMonitorThrottle < batteryMonitorThrottleTarget) {
//            batteryMonitorThrottle = batteryMonitorThrottleTarget;
//          }
//          if (throttle < batteryMonitorThrottle) {
//            batteyMonitorThrottleCorrection = 0;
//          }
//          else {
//            batteyMonitorThrottleCorrection = batteryMonitorThrottle - throttle;
//          }
//      #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
//        }
//      #endif
//    }
//  }
//#endif  


//#if defined AutoLanding
  #define BARO_AUTO_LANDING_DESCENT_SPEED 0.008
  #define SONAR_AUTO_LANDING_DESCENT_SPEED 0.005
 //#endif


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

#endif //#define _AQ_PROCESS_FLIGHT_CONTROL_H_