#include "PID.h"

struct PIDdata PID[LAST_PID_IDX];
float windupGuard;

float updatePID(float targetPosition, float currentPosition, struct PIDdata *PIDparameters) {

	// AKA PID experiments
	const float deltaPIDTime = (currentTime - PIDparameters->previousPIDTime) / 1000000.0;

	PIDparameters->previousPIDTime = currentTime;  // AKA PID experiments
	float error = targetPosition - currentPosition;

	if (inFlight) {
		PIDparameters->integratedError += error * deltaPIDTime;
	}
	else {
		PIDparameters->integratedError = 0.0;
	}
	PIDparameters->integratedError = constrain(PIDparameters->integratedError, -PIDparameters->windupGuard, PIDparameters->windupGuard);
	float dTerm = PIDparameters->D * (currentPosition - PIDparameters->lastError) / (deltaPIDTime * 100); // dT fix from Honk
	PIDparameters->lastError = currentPosition;

	return (PIDparameters->P * error) + (PIDparameters->I * PIDparameters->integratedError) + dTerm;
}

void zeroIntegralError() {
  for (byte axis = 0; axis <= ATTITUDE_YAXIS_PID_IDX; axis++) {
    PID[axis].integratedError = 0;
    PID[axis].previousPIDTime = currentTime;
  }
}
