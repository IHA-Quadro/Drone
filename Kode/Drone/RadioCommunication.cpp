#include "RadioCommunication.h"

int radioProgram;
QueueList<int> _queue;
QueueList<int> _rssiQueue;

void SetupRadioCommunicaiton()
{
	Wire.begin(TRANCEIVER_ADDRESS);// join i2c bus with address 0x90
	Wire.onReceive(receiveEvent);  // register event
	radioProgram = 0;

	_queue.setPrinter(Serial);
	_queue.EmptyList();
	_rssiQueue.EmptyList();
	_rssiQueue.setPrinter(Serial);
}

//Return all RSSI-values and stores a program, if any selected.
QueueList<int> GetQueueData()
{
	_rssiQueue.EmptyList();
	cli();
	while(_queue.count() > 1) //Greater than 1 allows at least 2 readings
	{
		int reading = _queue.pop();
		//printNewLine(reading, RADIOMODE);

		if(reading == PROGRAM_IDENTIFIER)
		{
			radioProgram = _queue.pop();
			//printInLine("P", RADIOMODE);
			//printInLine(radioProgram, RADIOMODE);

		}
		else if(reading == RSSI_IDENTIFIER)
		{
			int n = _queue.pop();
			//printInLine("RSSI: ", RADIOMODE);
			//printNewLine(n, RADIOMODE);
			_rssiQueue.push(n);
		}
	}
	_queue.EmptyList();
	sei();

	return _rssiQueue;
}

void receiveEvent(int howMany)
{
	cli();
	while(Wire.available())  
	{
		int c = Wire.read();
		//printNewLine(c,RADIOMODE);
		Serial.print(c);
		_queue.push(c);   // receive byte as a character
	}
	sei();
}
