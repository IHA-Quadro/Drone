
#include "RadioCommunication.h"
#include "HelperFile.h"

int radioProgram;
QueueList<int> _queue;
QueueList<int> _rssiQueue;

void setup()
{
	Wire.begin(TRANCEIVER_ADDRESS);// join i2c bus with address 0x90
	Wire.onReceive(receiveEvent);  // register event
	radioProgram = 0;

	_queue.~QueueList(); //Empty queue
	_queue.setPrinter(Serial);
}


//Return all RSSI-values and stores a program, if any selected.
QueueList<int> GetQueueData()
{
	cli();
	while(_queue.count() > 1) //Greater than 1 allows at least 2 readings
	{
		int reading = _queue.pop();
		printNewLine(reading, RADIOMODE);

		if(reading == PROGRAM_IDENTIFIER)
		{
			radioProgram = _queue.pop();
			printInLine("P", RADIOMODE);
			printInLine(radioProgram, RADIOMODE);

		}
		else if( reading == RSSI_IDENTIFIER)
		{
			int n = _queue.pop();
			printInLine("RSSI: ", RADIOMODE);
			printNewLine(n, RADIOMODE);
			_rssiQueue.push(n);
		}
	}
	sei();
	return _rssiQueue;
}

void receiveEvent(int howMany)
{
	cli();
	int c;

	while(Wire.available())  
	{
		c = Wire.read();
		_queue.push(c);   // receive byte as a character
		//printInLine(c, RADIOMODE);
	}
	sei();
}


void loop()
{

	GetQueueData();

}
