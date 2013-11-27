#include "RadioCommunication.h"
#include "Device_I2C.h"

#define TRANCEIVER_ADDRESS 0x90
#define INDICATORVALUE 0xAA

int radioProgram;

void SetupRadioCommunicaiton()
{
	Wire.begin(TRANCEIVER_ADDRESS);// join i2c bus with address 0x90
	radioProgram = 0;
}

void ReadRadio()
{
	int indicator, programValue;

	Wire.requestFrom(TRANCEIVER_ADDRESS, 2); //Request 2 bytes from radio

	//while(Wire.available())
	//{
	indicator = Wire.read(); //First value is always an indicator if program is with or not
	programValue = Wire.read(); //Optional - it 'indicator' == INDICATORVALUE this should be read
	//}

	if(indicator == INDICATORVALUE)
	{
		printNewLine(programValue, RADIOMODE);
	}
	radioProgram = programValue; //TODO: Valider dette
}
