#include "Motors_PWM_Timer.h"


void initializeMotors(NB_Motors numbers) {
	numberOfMotors = numbers;

#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	DDRE = DDRE | B00111000;                                  // Set ports to output PE3-5, OC3A, OC3B, OC3C
	if (numberOfMotors == FOUR_Motors) { 
		DDRH = DDRH | B00001000;                                // Set port to output PH3, OC4A
	} 
	else if (numberOfMotors == SIX_Motors) {                  // for 6 
		DDRH = DDRH | B00111000;                                // Set ports to output PH3-5, OC4A, OC4B, OC4C
	}
	else if (numberOfMotors == EIGHT_Motors) {                // for 8 motor
		DDRH = DDRH | B00111000;                                // Set ports to output PH3-5, OC4A, OC4B, OC4C
		DDRB = DDRB | B01100000;								  // OC1A, OC1B
	}
#else
	DDRB = DDRB | B00001110;                                  // Set ports to output PB1-3
	DDRD = DDRD | B00001000;                                  // Set port to output PD3
#endif

	commandAllMotors(1000);                                     // Initialise motors to 1000us (stopped)

#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	// Init PWM Timer 3                                       // WGMn1 WGMn2 WGMn3  = Mode 14 Fast PWM, TOP = ICRn ,Update of OCRnx at BOTTOM
	TCCR3A = (1<<WGM31)|(1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1);  // Clear OCnA/OCnB/OCnC on compare match, set OCnA/OCnB/OCnC at BOTTOM (non-inverting mode)
	TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);                 // Prescaler set to 8, that gives us a resolution of 0.5us
	ICR3 = PWM_COUNTER_PERIOD;                                // Clock_speed / ( Prescaler * desired_PWM_Frequency) #defined above.
	if (numberOfMotors == FOUR_Motors) {
		// Init PWM Timer 4
		TCCR4A = (1<<WGM41)|(1<<COM4A1);
		TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
		ICR4 = PWM_COUNTER_PERIOD;
	}
	else if ((numberOfMotors == SIX_Motors) || (numberOfMotors == EIGHT_Motors)) {  // for 8 motors
		// Init PWM Timer 4
		TCCR4A = (1<<WGM41)|(1<<COM4A1)|(1<<COM4B1)|(1<<COM4C1);
		TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
		ICR4 = PWM_COUNTER_PERIOD;
	}
	if (numberOfMotors == EIGHT_Motors){  // for 8 motors
		// Init PWM Timer 1
		TCCR1A = (1<<WGM11)|(1<<COM1A1)|(1<<COM1B1);
		TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);
		ICR1 = PWM_COUNTER_PERIOD;
	}
#else
	// Init PWM Timer 1  16 bit
	TCCR1A = (1<<WGM11)|(1<<COM1A1)|(1<<COM1B1);
	TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);
	ICR1 = PWM_COUNTER_PERIOD;
	// Init PWM Timer 2   8bit                                 // WGMn1 WGMn2 = Mode ? Fast PWM, TOP = 0xFF ,Update of OCRnx at BOTTOM
	TCCR2A = (1<<WGM20)|(1<<WGM21)|(1<<COM2A1)|(1<<COM2B1);    // Clear OCnA/OCnB on compare match, set OCnA/OCnB at BOTTOM (non-inverting mode)
	TCCR2B = (1<<CS22)|(1<<CS21);                              // Prescaler set to 256, that gives us a resolution of 16us
	// TOP is fixed at 255                                     // Output_PWM_Frequency = 244hz = 16000000/(256*(1+255)) = Clock_Speed / (Prescaler * (1 + TOP))
#endif
}

void writeMotors() 
{
	if(IsMotorKilled())
	{
		int motor;
		for(motor = MOTOR1 ; motor <= MOTOR8 ; motor++)
			motorCommand[motor] = MINTHROTTLE;
	}
#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	OCR3B = motorCommand[MOTOR1] * 2 ;
	OCR3C = motorCommand[MOTOR2] * 2 ;
	OCR3A = motorCommand[MOTOR3] * 2 ;
	OCR4A = motorCommand[MOTOR4] * 2 ;
	if (numberOfMotors == SIX_Motors || numberOfMotors == EIGHT_Motors) {
		OCR4B = motorCommand[MOTOR5] * 2 ;
		OCR4C = motorCommand[MOTOR6] * 2 ;
	}
	if (numberOfMotors == EIGHT_Motors) {
		OCR1A = motorCommand[MOTOR7] * 2 ;
		OCR1B = motorCommand[MOTOR8] * 2 ;
	}
#else
	OCR2B = motorCommand[MOTOR1] / 16 ;                       // 1000-2000 to 128-256
	OCR1A = motorCommand[MOTOR2] * 2 ;
	OCR1B = motorCommand[MOTOR3] * 2 ;
	OCR2A = motorCommand[MOTOR4] / 16 ;
#endif

}

void commandAllMotors(int command) 
{
#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	OCR3B = command * 2 ;
	OCR3C = command * 2 ;
	OCR3A = command * 2 ;
	OCR4A = command * 2 ;
	if (numberOfMotors == SIX_Motors || numberOfMotors == EIGHT_Motors) {
		OCR4B = command * 2 ;
		OCR4C = command * 2 ;
	}
	if (numberOfMotors == EIGHT_Motors) {
		OCR1A = command * 2 ;
		OCR1B = command * 2 ;
	}
#else
	OCR2B = command / 16 ;
	OCR1A = command * 2 ;
	OCR1B = command * 2 ;
	OCR2A = command / 16 ;
#endif
}
