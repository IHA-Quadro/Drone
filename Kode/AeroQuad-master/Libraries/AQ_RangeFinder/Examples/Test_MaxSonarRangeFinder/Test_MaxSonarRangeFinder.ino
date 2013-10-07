#define ADC_NUMBER_OF_BITS 10

#include "SensorsStatus.h"
#include "MaxSonarRangeFinder.h"

void setup() {
  Serial.begin(115200);
  inititalizeRangeFinders();
}

void loop() {
  
  updateRangeFinders();
  
  Serial.print("Distance = {");
	Serial.print(rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX]);
	//Serial.print(" : ");
	//Serial.print(rangeFinderRange[FRONT_RANGE_FINDER_INDEX]);
	Serial.println("}");
	delay(100);
}
