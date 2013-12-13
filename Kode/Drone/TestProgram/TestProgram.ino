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

//#include <Arduino.h>
//#include <wiring_private.h>
//#include <pins_arduino.h>
//
//void setup()
//{
//	Serial.begin(115200);
//	Serial.println("Test");
//}
//
//int PinIsLow(uint8_t pin)
//{
//  uint8_t bit = digitalPinToBitMask(pin);
//  uint8_t port = digitalPinToPort(pin);
//  if (port == NOT_A_PIN) 
//    return LOW;
//
//  return (*portOutputRegister(port) & bit) ? HIGH : LOW;
//}
//
//void loop()
//{
//  int outPin = 20;
//  int state;
//  pinMode(outPin, OUTPUT);
//  digitalWrite(outPin, state);
//  
//  state = PinIsLow(outPin);
//	Serial.println(state);
//
//	delay(500);
//}

#define SCANNER 1


#ifdef SCANNER
#include <Arduino.h>
#include <Wire.h>
#include "Device_I2C.h"
#include "QueueList.h"

#define INDICATORVALUE 0xAA
#define TRANCEIVER_ADDRESS 0x90

int radioProgram = 0;
QueueList<float> *_queue = NULL;


void setup()
{
	Wire.begin();
	Serial.begin(115200);

	_queue = new QueueList<float>();
	_queue->setPrinter(Serial);
}

int ReadRadio()
{
	int indicator, rssiValue, programValue;
	Wire.requestFrom(TRANCEIVER_ADDRESS, 2); //Request 3 bytes from radio

	while(Wire.available())
	{
		indicator = Wire.read(); //First value is always an indicator if program is with or not

		_queue->push((float)Wire.read());
		programValue = _queue->PeekLastElementFilter(0);
		if(_queue->count() > 3)
			_queue->pop();
	}
	
	if(indicator == INDICATORVALUE)
	{
		Serial.print("Program: ");
		Serial.print(indicator);
	}
	else
	{
		Serial.print("Strange number: ");
		Serial.print(indicator);
	}
	
	Serial.print(" : ");
	Serial.println(programValue);

	return programValue;
}

void loop()
{
	ReadRadio();
	delay(50);
}
#else
#include <Wire.h>


void setup()
{
	Wire.begin();

	Serial.begin(115200);
	Serial.println("\nI2C Scanner");
}


void loop()
{
	byte error, address;
	int nDevices;

	Serial.println("Scanning...");

	nDevices = 0;
	for(address = 1; address < 255; address++ ) 
	{
		// The i2c_scanner uses the return value of
		// the Write.endTransmisstion to see if
		// a device did acknowledge to the address.
		Wire.beginTransmission(address);
		error = Wire.endTransmission();

		if (error == 0)
		{
			Serial.print("I2C device found at address 0x");
			if (address<16) 
				Serial.print("0");
			Serial.print(address,HEX);
			Serial.println("  !");

			nDevices++;
		}
		else if (error==4) 
		{
			Serial.print("Unknow error at address 0x");
			if (address<16) 
				Serial.print("0");
			Serial.println(address,HEX);
		}    
	}
	if (nDevices == 0)
		Serial.println("No I2C devices found\n");
	else
		Serial.println("done\n");

	delay(5000);           // wait 5 seconds for next scan
}
#endif // !SCANNER

