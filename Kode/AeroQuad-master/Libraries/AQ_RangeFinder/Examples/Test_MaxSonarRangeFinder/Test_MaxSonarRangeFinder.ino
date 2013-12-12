//#include "SensorsStatus.h"
//#include "MaxSonarRangeFinder.h"
//
//void setup() 
//{
//	Serial.begin(115200);
//	inititalizeRangeFinders();
//}
//
//void loop() {
//  
//	updateRangeFinders();
//  
//	Serial.print("Distance = {");
//	Serial.print(rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX]);
//	Serial.print(" : ");
//	Serial.print(rangeFinderRange[FRONT_RANGE_FINDER_INDEX]);
//	Serial.print(" : ");
//	Serial.print(rangeFinderRange[RIGHT_RANGE_FINDER_INDEX]);
//	Serial.print(" : ");
//	Serial.print(rangeFinderRange[LEFT_RANGE_FINDER_INDEX]);
//	Serial.println("}");
//	delay(10);
//}

//#define I2C_READ
//#define I2C_SCANNER

#ifdef I2C_READ
#include <Arduino.h>
#include <Wire.h>

#define TRANCEIVER_ADDRESS 0x90
#define INDICATORVALUE 0x3F

void SetupRadioCommunicaiton();
void ReadRadio();

int radioProgram;

void setup()
{
	Serial.begin(115200);            // start serial for output
	Serial.println("All setup");

	SetupRadioCommunicaiton();
}

void loop()
{
	delay(100);
	Serial.print(".");
	
	ReadRadio();
}

void SetupRadioCommunicaiton()
{
	Wire.begin(TRANCEIVER_ADDRESS);// join i2c bus with address 0x90
	radioProgram = 0;
}

void ReadRadio()
{
	int indicator, programValue;

	Wire.requestFrom(TRANCEIVER_ADDRESS, 2); //Request 2 bytes from radio

	indicator = Wire.read(); //First value is always an indicator if program is with or not
	programValue = Wire.read(); //Optional - it 'indicator' == INDICATORVALUE this should be read

	if(indicator == INDICATORVALUE)
		Serial.print(programValue);

	radioProgram = programValue; //TODO: Valider dette
}
#endif

#ifdef I2C_SCANNER
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
#endif