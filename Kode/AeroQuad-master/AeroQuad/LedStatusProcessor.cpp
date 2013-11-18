#include "LedStatusProcessor.h"

byte flashingLedState = 0; // this counter increments by one at 10Hz

void processLedStatus() {

  
  //
  // process ready state light in case we use GPS
  //
  #if defined (UseGPS)
    if (haveAGpsLock()) {
      if (isHomeBaseInitialized()) {
        digitalWrite(LED_Green, HIGH);
      }
      else {
        digitalWrite(LED_Green, (flashingLedState & 4));
      }
    }
    else { 
      digitalWrite(LED_Green, (flashingLedState & 2));
    }
  #endif
  
  //
  // process ready state light in case we use Batt monitor
  //
  #if defined (BattMonitor)
    if (batteryAlarm) {
      digitalWrite(LED_Red, flashingLedState & 4);
    } else if (batteryWarning) {
      digitalWrite(LED_Red, (flashingLedState & 15)==0);
    } else { 
      digitalWrite(LED_Red, LOW);
    }
  #endif  

  //
  // process mode light
  //
  if (flightMode == ATTITUDE_FLIGHT_MODE) {
    digitalWrite(LED_Yellow, HIGH);
  }
  else {
    digitalWrite(LED_Yellow, LOW);
  }

  flashingLedState++;
}
