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
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\QuadDrone.ino"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\AQMath.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\AQMath.h"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\Accelerometer.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\Accelerometer.h"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\AeroQuad.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\AeroQuad.h"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\AltitudeControlProcessor.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\AltitudeControlProcessor.h"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\BarometricSensor.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\BarometricSensor.h"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\ControlFaker.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\ControlFaker.h"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\DataStorage.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\DataStorage.h"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\Device_I2C.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\Device_I2C.h"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\FlightCommandProcessor.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\FlightCommandProcessor.h"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\FlightControlProcessor.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\FlightControlProcessor.h"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\FlightControlQuadPlus.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\FlightControlQuadPlus.h"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\FlightControlVariable.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\FlightControlVariable.h"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\FourtOrderFilter.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\FourtOrderFilter.h"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\GlobalDefined.h"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\Gyroscope.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\Gyroscope.h"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\HeadingHoldProcessor.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\HeadingHoldProcessor.h"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\InoHelper.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\InoHelper.h"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\Kinematics.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\Kinematics.h"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\Kinematics_ARG.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\Kinematics_ARG.h"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\MaxSonarRangeFinder.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\MaxSonarRangeFinder.h"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\MotorControl.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\MotorControl.h"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\Motors.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\Motors.h"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\Motors_PWM_Timer.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\Motors_PWM_Timer.h"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\PID.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\PID.h"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\PrintDrone.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\PrintDrone.h"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\RangeFinder.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\RangeFinder.h"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\ReceiveCommandTestData.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\ReceiveCommandTestData.h"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\Receiver.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\Receiver.h"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\Receiver_MEGA.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\Receiver_MEGA.h"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\SensorStatus.c"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\SensorsStatus.h"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\SerialCom.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\SerialCom.h"
#include "C:\Users\Becks\Documents\Drone\Kode\QuadDrone\UserConfiguration.h"
#endif
