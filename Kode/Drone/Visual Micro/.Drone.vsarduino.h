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
//

#include "C:\Program Files (x86)\Arduino\hardware\arduino\variants\mega\pins_arduino.h" 
#include "C:\Program Files (x86)\Arduino\hardware\arduino\cores\arduino\arduino.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\Drone.ino"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\AQMath.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\AQMath.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\Accelerometer.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\Accelerometer.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\Accelerometer_ADXL345_9DOF.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\Accelerometer_ADXL345_9DOF.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\AeroQuad.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\AeroQuad.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\AltitudeControlProcessor.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\AltitudeControlProcessor.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\BarometricSensor.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\BarometricSensor.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\BarometricSensor_BMP085.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\BarometricSensor_BMP085.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\ControlFaker.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\ControlFaker.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\DataStorage.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\DataStorage.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\Device_I2C.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\Device_I2C.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\FlightCommandProcessor.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\FlightCommandProcessor.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\FlightControlProcessor.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\FlightControlProcessor.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\FlightControlQuadPlus.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\FlightControlQuadPlus.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\FlightControlTri.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\FlightControlVariable.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\FlightControlVariable.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\FourtOrderFilter.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\FourtOrderFilter.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\GlobalDefined.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\GpsNavigator.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\Gyroscope.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\Gyroscope.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\Gyroscope_ITG3200.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\Gyroscope_ITG3200Common.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\Gyroscope_ITG3200_9DOF.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\HeadingHoldProcessor.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\InoHelper.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\InoHelper.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\Kinematics.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\Kinematics.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\Kinematics_ARG.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\LedStatusProcessor.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\MavLink.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\MaxSonarRangeFinder.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\MaxSonarRangeFinder.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\MotorControl.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\MotorControl.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\Motors.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\Motors.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\Motors_PWM_Timer.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\Motors_PWM_Timer.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\OSDDisplayController.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\OSDMenu.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\PID.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\PID.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\PrintDrone.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\PrintDrone.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\RangeFinder.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\RangeFinder.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\ReceiveCommandTestData.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\ReceiveCommandTestData.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\Receiver.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\Receiver.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\Receiver_MEGA.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\Receiver_MEGA.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\SensorsStatus.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\SensorsStatus.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\SerialCom.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\SerialCom.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\SerialLCD.h"
#include "C:\Users\Becks\Documents\Drone\Kode\Drone\UserConfiguration.h"
#endif
