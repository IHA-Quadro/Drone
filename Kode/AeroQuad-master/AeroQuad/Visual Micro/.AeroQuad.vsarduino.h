#ifndef _VSARDUINO_H_
#define _VSARDUINO_H_
//Board = Arduino Mega 2560 or Mega ADK
#define __AVR_ATmega2560__
#define 
#define ARDUINO 105
#define ARDUINO_MAIN
#define __AVR__
#define F_CPU 16000000L
#define __cplusplus
#define __inline__
#define __asm__(x)
#define __extension__
#define __ATTR_PURE__
#define __ATTR_CONST__
#define __inline__
#define __asm__ 
#define __volatile__

#define __builtin_va_list
#define __builtin_va_start
#define __builtin_va_end
#define __DOXYGEN__
#define __attribute__(x)
#define NOINLINE __attribute__((noinline))
#define prog_void
#define PGM_VOID_P int
            
typedef unsigned char byte;
extern "C" void __cxa_pure_virtual() {;}

void initPlatform();
void measureCriticalSensors();
//
void process100HzTask();
void process50HzTask();
void process10HzTask1();
void process10HzTask2();
void process10HzTask3();
void process1HzTask();
//

#include "C:\Program Files (x86)\Arduino\hardware\arduino\variants\mega\pins_arduino.h" 
#include "C:\Program Files (x86)\Arduino\hardware\arduino\cores\arduino\arduino.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\AeroQuad.ino"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\AQMath.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\AQMath.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\AQ_SoftModem.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\Accelerometer.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\Accelerometer.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\AeroQuad.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\AeroQuad.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\AltitudeControlProcessor.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\AltitudeControlProcessor.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\BarometricSensor.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\BarometricSensor.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\ControlFaker.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\ControlFaker.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\DataStorage.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\DataStorage.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\Device_I2C.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\Device_I2C.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\FlightCommandProcessor.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\FlightCommandProcessor.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\FlightControlProcessor.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\FlightControlProcessor.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\FlightControlQuadPlus.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\FlightControlQuadPlus.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\FlightControlVariable.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\FourtOrderFilter.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\FourtOrderFilter.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\GlobalDefined.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\GpsAdapter.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\GpsDataType.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\GpsNavigator.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\Gyroscope.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\Gyroscope.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\HeadingHoldProcessor.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\HeadingHoldProcessor.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\InoHelper.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\InoHelper.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\Kinematics.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\Kinematics.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\Kinematics_ARG.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\Kinematics_ARG.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\LedStatusProcessor.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\LedStatusProcessor.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\MavLink.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\MavLink.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\MaxSonarRangeFinder.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\MaxSonarRangeFinder.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\MotorControl.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\MotorControl.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\Motors.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\Motors.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\Motors_PWM_Timer.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\Motors_PWM_Timer.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\OSDDisplayController.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\OSDMenu.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\PID.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\PID.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\PrintDrone.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\PrintDrone.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\RangeFinder.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\ReceiveCommandTestData.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\ReceiveCommandTestData.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\Receiver.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\Receiver.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\Receiver_MEGA.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\Receiver_MEGA.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\SensorsStatus.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\SerialCom.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\SerialCom.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\SerialLCD.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\AeroQuad\UserConfiguration.h"
#endif
