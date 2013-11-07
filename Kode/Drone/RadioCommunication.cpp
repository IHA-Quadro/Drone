#include "RadioCommunication.h"
#include "Device_I2C.h"

int radioProgram;

void SetupRadioCommunicaiton()
{
	Wire.begin(TRANCEIVER_ADDRESS);// join i2c bus with address 0x90
	radioProgram = 0;
}

int ReadRadio()
{
	int is = readWhoI2C(TRANCEIVER_ADDRESS);

	Wire.requestFrom(TRANCEIVER_ADDRESS, 3); //Request 3 bytes from radio
	int indicator = Wire.read(); //First value is always an indicator if program is with or not
	int rssiValue = Wire.read(); //Always sent as RSSI-value
	int programValue = Wire.read(); //Optional - it 'indicator' == INDICATORVALUE this should be read

	//printInLine("Is: ", RADIOMODE);
	//printInLine(is, RADIOMODE);
	//printInLine(" - ", RADIOMODE);
	//printInLine(indicator, RADIOMODE);
	//printInLine(" - ", RADIOMODE);
	//printInLine(rssiValue, RADIOMODE);
	//printInLine(" - ", RADIOMODE);
	//printNewLine(programValue, RADIOMODE);

	if(indicator == INDICATORVALUE)
		radioProgram = programValue;

	return rssiValue;
}