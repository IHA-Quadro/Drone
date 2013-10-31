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

QueueList<int> queue;

void setup()
{
	Wire.begin(TRANCEIVER_ADDRESS);// join i2c bus with address 0x90
	Wire.onReceive(receiveEvent);  // register event
	queue.setPrinter(Serial);


	Serial.begin(115200);            // start serial for output
	Serial.println("All setup");
}

void loop()
{
	delay(300);
	//Serial.print("Queue size: ");
	//Serial.print(queue.count());
	//Serial.print(": ");

	cli();
	while(!queue.isEmpty())
	{
		int	c = queue.pop();

		if(c == 82)
			Serial.print("R");
		else
		{
			Serial.print(c);
			Serial.print(" ");
		}
	}
	sei();

	Serial.println();
}


// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
	cli();
	while(Wire.available())  // loop through all but the last
	{
		queue.push(Wire.read());   // receive byte as a character
	}
	sei();
}
