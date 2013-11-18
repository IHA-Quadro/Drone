#ifndef _AQ_SENSORS_STATE_H_
#define _AQ_SENSORS_STATE_H_

extern unsigned long vehicleState;

#define GYRO_DETECTED         0x001
#define ACCEL_DETECTED        0x002
#define MAG_DETECTED          0x004
#define BARO_DETECTED         0x008
#define HEADINGHOLD_ENABLED   0x010
#define ALTITUDEHOLD_ENABLED  0x020
#define BATTMONITOR_ENABLED   0x040
#define CAMERASTABLE_ENABLED  0x080
#define RANGE_ENABLED         0x100

#endif