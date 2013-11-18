#ifndef _OSD_DISPLAY_CONTROLLER_H_
#define _OSD_DISPLAY_CONTROLLER_H_

#include "OSD.h"

byte OSDsched = 0;

void updateOSD() {
  // OSD is updated fully in 8 rounds
  // 1,3,5,7 - Attitude Indicator - updated at 5Hz
  // 2       - Altitude, Heading, Timer, RSSI - updated at 1.25Hz
  // 4       - Battery info
  // 6,8     - GPS (internally 2 phases: Position & Navigation

  // Check notify first, if it did something we dont't have time for other stuff
  if (displayNotify()) {
    return;
  }

  #ifdef ShowAttitudeIndicator
    if (OSDsched&0x55) {
      byte extendedFlightMode = flightMode;
      #if defined UseGPSNavigator
        if (ON == positionHoldState) extendedFlightMode = 2;
        if (ON == navigationState) extendedFlightMode = 3;
      #endif
      displayArtificialHorizon(kinematicsAngle[XAXIS], kinematicsAngle[YAXIS], extendedFlightMode);
    }
  #endif

  if (OSDsched&0x02) {
    displayFlightTime(motorArmed);
    #if defined AltitudeHoldBaro
      displayAltitude(getBaroAltitude(), baroAltitudeToHoldTarget, altitudeHoldState);
    #endif
    #ifdef HeadingMagHold
      displayHeading(trueNorthHeading);
    #endif
    #ifdef ShowRSSI
      displayRSSI();
    #endif
  }

  if (OSDsched&0x08) {
    #ifdef BattMonitor
      displayVoltage(motorArmed);
    #endif
  }

  if (OSDsched&0x20) {
    #ifdef UseGPS
      if (haveAGpsLock()) {
        displayGPS(currentPosition, missionPositionToReach, getGpsSpeed(), getCourse(), trueNorthHeading, gpsData.sats);
      }
      else {
        displayGPS(currentPosition, currentPosition, 0, 0, trueNorthHeading, gpsData.sats);
      }
    #endif
  }

  if (OSDsched&0x80) {
    #ifdef AltitudeHoldRangeFinder
      if (motorArmed) {
        displayRanger();
      }
    #endif
  }

  OSDsched <<= 1;
  if (!OSDsched) {
    OSDsched = 0x01;
  }
}

#endif
