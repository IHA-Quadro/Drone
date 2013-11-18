#include "BarometricSensor.h"

float baroAltitude      = 0.0; 
float baroRawAltitude   = 0.0;
float baroGroundAltitude = 0.0;
float baroSmoothFactor   = 0.02;

const float getBaroAltitude() {
	return baroAltitude - baroGroundAltitude;
}

void measureGroundBaro() {
	// measure initial ground pressure (multiple samples)
	float altSum = 0.0;
	for (int i=0; i < 25; i++) {
		measureBaro();
		altSum += baroRawAltitude;
		delay(12);
	}
	baroGroundAltitude = altSum / 25;
}
