//#include <Arduino.h>
//#include "RadioCommunication.h"
//#include <Wire.h>
//
//int radioProgram;
//QueueList<int> _queue;
//QueueList<int> _rssiQueue;
//
//void setup()
//{
//	Wire.begin(TRANCEIVER_ADDRESS);// join i2c bus with address 0x90
//	Wire.onReceive(receiveEvent);  // register event
//	_queue.EmptyList();
//	_rssiQueue.EmptyList();
//	Serial.begin(115200);
//}
//
//void GetQueueData()
//{
//	_rssiQueue.EmptyList();
//	cli();
//	//Serial.println(_queue.count());
//
//	while(_queue.count() > 1) //Greater than 1 allows at least 2 readings
//	{
//		int reading = _queue.pop();
//		//Serial.println(reading);
//
//		if(reading == PROGRAM_IDENTIFIER)
//		{
//			radioProgram = _queue.pop();
//			Serial.print("P");
//			Serial.print(radioProgram);
//		}
//		else if( reading == RSSI_IDENTIFIER)
//		{
//			int n = _queue.pop();
//			Serial.print("RSSI: ");
//			Serial.println(n);
//			_rssiQueue.push(n);
//		}
//	}
//	_queue.EmptyList();
//	sei();
//}
//
//void testProgram()
//{
//	//for(int i = 0; i < 10; i++)
//	//{
//	//	_queue.push(1);
//	//}
//
//	Serial.print("Count: ");
//	Serial.println(_queue.count());
//	_queue.EmptyList();
//}
//
//void loop()
//{
//	delay(100);
//	GetQueueData();
//
//	//while(_rssiQueue.count() != 0)
//	//{
//	//	Serial.print("RSSI: ");
//	//	Serial.println(_rssiQueue.pop());
//	//}
//
//	//testProgram();
//}
//
//
//void receiveEvent(int howMany)
//{
//	cli();
//	while(Wire.available())  
//	{
//		int c = Wire.read();
//		_queue.push(c);
//		//if(c == RSSI_IDENTIFIER)
//		//	Serial.print("RSSI: ");
//		//else
//		//	Serial.println(c);
//	}
//	sei();
//}

#include <Arduino.h>
#include <wiring_private.h>
#include <pins_arduino.h>

void setup()
{
	Serial.begin(115200);
	Serial.println("Test");
}

int PinIsLow(uint8_t pin)
{
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  if (port == NOT_A_PIN) 
    return LOW;

  return (*portOutputRegister(port) & bit) ? HIGH : LOW;
}

void loop()
{
  int outPin = 20;
  int state;
  pinMode(outPin, OUTPUT);
  digitalWrite(outPin, state);
  
  state = PinIsLow(outPin);
	Serial.println(state);

	delay(500);
}