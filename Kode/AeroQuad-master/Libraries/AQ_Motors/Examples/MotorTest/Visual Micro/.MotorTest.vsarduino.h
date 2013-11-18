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

void initMotors(NB_Motors motorConfig);
void initMotors(NB_Motors motorConfig);
void initMotors(NB_Motors motorConfig);
void initMotors(NB_Motors motorConfig);
//
void SteadyMotors();
void blink();
//

#include "C:\Program Files (x86)\Arduino\hardware\arduino\variants\mega\pins_arduino.h" 
#include "C:\Program Files (x86)\Arduino\hardware\arduino\cores\arduino\arduino.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\Libraries\AQ_Motors\Examples\MotorTest\MotorTest.ino"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\Libraries\AQ_Motors\Examples\MotorTest\GlobalDefined.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\Libraries\AQ_Motors\Examples\MotorTest\Motors.cpp"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\Libraries\AQ_Motors\Examples\MotorTest\Motors.h"
#include "C:\Users\Becks\Documents\Drone\Kode\AeroQuad-master\Libraries\AQ_Motors\Examples\MotorTest\Motors_PWM_Timer.h"
#endif
