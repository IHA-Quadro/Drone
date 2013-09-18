#include "Motors_PWM_Timer.h"

void initializeMotors(NB_Motors numbers) 
{
	numberOfMotors = FOUR_Motors;

	DDRE = DDRE | B00111000;                                  // Set ports to output PE3-5, OC3A, OC3B, OC3C
	DDRH = DDRH | B00001000;                                // Set port to output PH3, OC4A


	commandAllMotors(1000);                                     // Initialise motors to 1000us (stopped)

	// Init PWM Timer 3                                       // WGMn1 WGMn2 WGMn3  = Mode 14 Fast PWM, TOP = ICRn ,Update of OCRnx at BOTTOM
	TCCR3A = (1<<WGM31)|(1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1);  // Clear OCnA/OCnB/OCnC on compare match, set OCnA/OCnB/OCnC at BOTTOM (non-inverting mode)
	TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);                 // Prescaler set to 8, that gives us a resolution of 0.5us
	ICR3 = PWM_COUNTER_PERIOD;                                // Clock_speed / ( Prescaler * desired_PWM_Frequency) #defined above.

	// Init PWM Timer 4
	TCCR4A = (1<<WGM41)|(1<<COM4A1);
	TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
	ICR4 = PWM_COUNTER_PERIOD;

}

void writeMotors() 
{
	if(getMotorStatus())
	{
		int motor;
		for(motor = MOTOR1 ; motor <= MOTOR8 ; motor++)
			motorCommand[motor] = MINTHROTTLE;
	}

	OCR3B = motorCommand[MOTOR1] * 2;
	OCR3C = motorCommand[MOTOR2] * 2;
	OCR3A = motorCommand[MOTOR3] * 2;
	OCR4A = motorCommand[MOTOR4] * 2;
}

void commandAllMotors(int command) {
	OCR3B = command * 2 ;
	OCR3C = command * 2 ;
	OCR3A = command * 2 ;
	OCR4A = command * 2 ;
}