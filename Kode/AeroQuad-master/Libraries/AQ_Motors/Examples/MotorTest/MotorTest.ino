/*
  AeroQuad v3.0 - April 2011
  www.AeroQuad.com 
  Copyright (c) 2011 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multicopter.
 
  This program is free software: you can redistribute it and/or modify 
  it under the terms of the GNU General Public License as published by 
  the Free Software Foundation, either version 3 of the License, or 
  (at your option) any later version. 

  This program is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
  GNU General Public License for more details. 

  You should have received a copy of the GNU General Public License 
  along with this program. If not, see <http://www.gnu.org/licenses/>. 
*/

//#define MOTOR_PWM
#define MOTOR_PWM_Timer
//#define MOTOR_APM
//#define MOTOR_I2C

#define NB_MOTOR_4
//#define NB_MOTOR_6
//#define NB_MOTOR_8


#if defined MOTOR_PWM
  #include <Motors_PWM.h>
  
  void initMotors(NB_Motors motorConfig) {
    initializeMotors(motorConfig); 
  }
#elif defined MOTOR_PWM_Timer
  #include <Motors_PWM_Timer.h>
  
  void initMotors(NB_Motors motorConfig) {
    initializeMotors(motorConfig); 
  }

#elif defined MOTOR_APM
  #include <Motors_APM.h>
 
  void initMotors(NB_Motors motorConfig) {
    initRC();
    initializeMotors(motorConfig); 
  }
  
#elif defined MOTOR_I2C
  #include <Wire.h>
  #include <Device_I2C.h>

   void initMotors(NB_Motors motorConfig) {
    Wire.begin();
    initializeMotors(motorConfig); 
  }

  
#endif

#if defined (NB_MOTOR_4)
  #define NB_MOTOR 4
  #define NB_MOTOR_CONFIG FOUR_Motors
#elif defined (NB_MOTOR_6)
  #define NB_MOTOR 6
  #define NB_MOTOR_CONFIG SIX_Motors
#else
  #define NB_MOTOR 8
  #define NB_MOTOR_CONFIG EIGHT_Motors
#endif

int led = 13;
int maxSpeed = 1220;
int minSpeed = 1200;
int initSpeed = 1000;
double pusherScaling = 1;


void setup() {
	Serial.begin(115200);
	pinMode(led, OUTPUT);   
	initMotors(NB_MOTOR_CONFIG);
}

//void motorTest()
//{
//	for(int motorTrust = minSpeedNonPusher; motorTrust < maxSpeedNonPusher; motorTrust+=20) 
//	{
//		for(byte motor = 0; motor < NB_MOTOR; motor++)
//		{
//			motorCommand[motor] = motorTrust;
//			//if(motor == 0 || motor == 3)
//			//	motorCommand[motor] = maxSpeedPusher; //(int)(motorTrust*pusherScaling);
//			//else
//			//	motorCommand[motor] = maxSpeedNonPusher;
//		}
//		writeMotors();
//		delay(200); //Why?
//	}
//
//	for(int motorTrust = maxSpeedNonPusher; motorTrust > minSpeedNonPusher; motorTrust-=20) 
//	{
//		for(byte motor = 0; motor < NB_MOTOR; motor++)
//		{
//			motorCommand[motor] = motorTrust;
//		 //   if(motor == 0 || motor == 3)
//			//	motorCommand[motor] = minSpeedPusher; //(int)(motorTrust*pusherScaling);
//			//else
//			//	motorCommand[motor] = minSpeedNonPusher;
//		}
//		writeMotors();
//		delay(200); //Why?
//	}
//}

bool _init = false;

byte pusherBlade1 = 0;
byte pusherBlade2 = 3;
byte normalBlade1 = 1;
byte normalBlade2 = 2;

//Initialize the motors from minSpeed rpm to maxSpeed.
//Sets the motors steady for maxSpeed
void SteadyMotors()
{
	if(!_init)
	{
		for(int motorTrust = initSpeed; motorTrust < minSpeed; motorTrust+=10) 
		{
			for(byte motor = 0; motor < NB_MOTOR; motor++)
			{
				//if(motor == pusherBlade1 || motor == pusherBlade2)
					motorCommand[motor] = motorTrust;
				//else
				//	motorCommand[motor] = -motorTrust;
			}
			writeMotors();
			delay(200); //Why?
		}
		_init = true;
	}

	for(byte motor = 0; motor < NB_MOTOR; motor++)
	{
		if(motor == pusherBlade1 || motor == pusherBlade2)
			motorCommand[motor] = (int)((maxSpeed*pusherScaling < minSpeed ? minSpeed : maxSpeed*pusherScaling));
		else
			motorCommand[motor] = maxSpeed;
	}
	writeMotors();
	delay(200);

}


int blinkTimer = 200;
void blink() {
  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(blinkTimer);               // wait for a second
  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  delay(blinkTimer);               // wait for a second
}
 

void loop()
{
	SteadyMotors();
	delay(200);
}








