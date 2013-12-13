#include "RadioCommunication.h"
#include "Device_I2C.h"
#include "QueueList.h"

#define TRANCEIVER_ADDRESS 0x90
#define INDICATORVALUE 0xAA

int radioProgram;
QueueList<float> *_queue = NULL;

void SetupRadioCommunicaiton()
{
	Wire.begin(TRANCEIVER_ADDRESS);// join i2c bus with address 0x90
	radioProgram = 0;

	_queue = new QueueList<float>();
	_queue->setPrinter(Serial);
}

void ReadRadio()
{
	int indicator, programValue;

	Wire.requestFrom(TRANCEIVER_ADDRESS, 2); //Request 2 bytes from radio

	indicator = Wire.read(); //First value is always an indicator if program is with or not
	_queue->push((float)Wire.read());

	programValue = _queue->PeekLastElementFilter(0);
	if(_queue->count() > 10)
		_queue->pop();

	if(indicator == INDICATORVALUE)
	{
		printInLine("New program: ", RADIOMODE);
		printNewLine(programValue, RADIOMODE);
	}
	radioProgram = programValue; //TODO: Valider dette
}
