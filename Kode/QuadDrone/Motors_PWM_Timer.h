#ifndef _AEROQUAD_MOTORS_PWM_TIMER_H_
#define _AEROQUAD_MOTORS_PWM_TIMER_H_

///***********************************************************/
///********************* PWMtimer Motors *********************/
///***********************************************************/
///*Some basics about the 16 bit timer:
//- The timer counts clock ticks derived from the CPU clock. Using 16MHz CPU clock
//  and a prescaler of 8 gives a timer clock of 2MHz, one tick every 0.5us. This
//  is also called timer resolution.
//- The timer is used as cyclic upwards counter, the counter period is set in the
//  ICRx register. IIRC period-1 has to be set in the ICRx register.
//- When the counter reaches 0, the outputs are set
//- When the counter reaches OCRxy, the corresponding output is cleared.
//In the code below, the period shall be 3.3ms (300hz), so the ICRx register is
// set to 6600 ticks of 0.5us/tick. It probably should be 6599, but who cares about
// this 0.5us for the period. This value is #define TOP
//The high time shall be 1000us, so the OCRxy register is set to 2000. In the code
// below this can be seen in the line "commandAllMotors(1000);"  A change of
// the timer period does not change this setting, as the clock rate is still one
// tick every 0.5us. If the prescaler was changed, the OCRxy register value would
// be different.
//*/

//  Motor   Mega Pin Port        Uno Pin Port          HEXA Mega Pin Port
//    FRONT         2  PE4              3  PD3
//    REAR          3  PE5              9  PB1
//    RIGHT         5  PE3             10  PB2                      7  PH4
//    LEFT          6  PH3             11  PB3                      8  PH5
//

#include "Motors.h"
#include "Receiver.h"

#if defined (USE_400HZ_ESC)
#define PWM_FREQUENCY 400   // in Hz
#else
#define PWM_FREQUENCY 300   // in Hz
#endif  

#define PWM_PRESCALER 8
#define PWM_COUNTER_PERIOD (F_CPU/PWM_PRESCALER/PWM_FREQUENCY)

void initializeMotors(NB_Motors numbers);
void commandAllMotors(int command);
void writeMotors();

#endif