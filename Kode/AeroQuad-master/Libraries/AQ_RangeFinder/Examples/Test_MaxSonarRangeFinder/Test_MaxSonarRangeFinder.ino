//#define ADC_NUMBER_OF_BITS 10
//
//#include "SensorsStatus.h"
//#include "MaxSonarRangeFinder.h"
//
//void setup() {
//  Serial.begin(115200);
//  inititalizeRangeFinders();
//}
//
//void loop() {
//  
//  updateRangeFinders();
//  
//  Serial.print("Distance = {");
//	//Serial.print(rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX]);
//	//Serial.print(" : ");
//	Serial.print(rangeFinderRange[FRONT_RANGE_FINDER_INDEX]);
//	//Serial.print(" : ");
//	//Serial.print(rangeFinderRange[RIGHT_RANGE_FINDER_INDEX]);
//	//Serial.print(" : ");
//	//Serial.print(rangeFinderRange[LEFT_RANGE_FINDER_INDEX]);
//	Serial.println("}");
//	delay(10);
//}


#include <Wire.h>
#include "QueueList.h"

#define TRANCEIVER_ADDRESS 0x90

QueueList<char> queue;

void setup()
{
	Wire.begin(TRANCEIVER_ADDRESS);// join i2c bus with address 0x90
	Wire.onReceive(receiveEvent);  // register event
	//queue = new test::QueueList<char>();

	Serial.begin(115200);            // start serial for output
	Serial.println("All setup");
}

void loop()
{
	delay(100);

	while(!queue.isEmpty())
		Serial.println(queue.pop());
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
	while(1 < Wire.available())  // loop through all but the last
	{
		char c = Wire.read();   // receive byte as a character
		queue.push(c);
	//	Serial.print(c);           // print the character
	}
	//int x = Wire.read();      // receive byte as an integer
	//Serial.println(x);           // print the integer
}