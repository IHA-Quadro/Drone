#include "MaxSonarRangeFinder.h"

short rangerScale[] = { 
	13005, // MB10xx series
	10240, // MB12xx series
};

byte rangerWait[] = {
	2, // MB1000 needs 50ms i.e. wait 2 cycles (60ms)
	4, // MB1200 needs 100ms i.e. wait 5 cycles (100ms)
};

struct rangeFinder rangeFinders[RANGEFINDERSIZE] = {
	// Define your rangers here
	// First ranger is given priority so it should be used for altitude
	// If using more than one ranger you should connect the 'trigger' to the 'RX' pin on the ranger.
	//
	//    { ALTITUDE_RANGE_FINDER_INDEX, A1, 24, MB1200}, 
	{ ALTITUDE_RANGE_FINDER_INDEX, A1, 24, MB1000}, //A1, 0
	{ FRONT_RANGE_FINDER_INDEX,    A4, 27, MB1000}, //A2, 25
	{ RIGHT_RANGE_FINDER_INDEX,    A2, 26, MB1000},
	//{ REAR_RANGE_FINDER_INDEX,     A4, 27, MB1000},
	{ LEFT_RANGE_FINDER_INDEX,     A3, 25, MB1000} //A5, 28
};

short lastRange[RANGER_COUNT];
byte rangerWaitCycles = 0;
byte rangerSchedule = 0;
byte rangerToRead = 0;

struct RangerArray RangerAverage[RANGER_COUNT];

void inititalizeRangeFinders() 
{
	for (byte i = 0; i < RANGER_COUNT; i++) 
	{
		rangeFinderRange[rangeFinders[i].target] = -1;

		if (rangeFinders[i].triggerpin) 
		{			
			digitalWrite(rangeFinders[i].triggerpin, LOW);
			pinMode(rangeFinders[i].triggerpin, OUTPUT);
		}

		lastRange[i] = 32000; 

		//pinMode(rangeFinders[i].pin, INPUT);
	}

	for(byte sonar = 0; sonar < RANGER_COUNT; sonar++)
	{
		RangerAverage[sonar].queue.EmptyList();
		RangerAverage[sonar].average = 0;
	}

	rangerWaitCycles = 10; // allow to initialize
}

void updateRangeFinders() 
{
	rangerToRead = 0;
	byte rangerToTrigger = 0;

	if (rangerWaitCycles) 
	{
		rangerWaitCycles--;
		return;
	}

	if (RANGER_COUNT > 1) 
	{
		if ((rangerSchedule & 1) == 0) 
		{
			rangerToRead = 0;
			rangerToTrigger = (rangerSchedule >> 1) + 1;
		}
		else 
		{
			rangerToRead = (rangerSchedule >> 1) + 1;
			rangerToTrigger = 0;
		}

		rangerSchedule++;

		if (((rangerSchedule>>1) + 1) >= (byte)RANGER_COUNT) 
			rangerSchedule = 0;
	}

	if (rangeFinders[rangerToTrigger].triggerpin) 
		digitalWrite(rangeFinders[rangerToTrigger].triggerpin, HIGH);

	long read = analogRead(rangeFinders[rangerToRead].pin);
	long scale = (long)(rangerScale[rangeFinders[rangerToRead].type]) / (1L<<ADC_NUMBER_OF_BITS);

	short range = (short)(read * scale);


	// Following will accept the sample if it's either withing "spike margin" of last raw reading or previous accepted reading
	// otherwise it's ignored as noise
	if ( ((abs(range - lastRange[rangerToRead]) < SPIKE_FILTER_MARGIN) || 
		(abs(range * 1000.0 - rangeFinderRange[rangeFinders[rangerToRead].target]) < SPIKE_FILTER_MARGIN)) && range < 3600) 
	{
		rangeFinderRange[rangeFinders[rangerToRead].target] = (float)range / 1000.0;
	}

	lastRange[rangerToRead] = range;

	rangerWaitCycles = rangerWait[rangeFinders[rangerToRead].type];

	if (rangeFinders[rangerToTrigger].triggerpin) 
		digitalWrite(rangeFinders[rangerToTrigger].triggerpin, LOW);

	//if(rangerToTrigger != ALTITUDE_RANGE_FINDER_INDEX)
		StoreRangeValues(rangerToTrigger);
}

//Store the lastest value in an array, take the average and save it along
void StoreRangeValues(int ranger)
{
	if(RangerAverage[ranger].queue.count() >= RANGERARRAYSIZE)	
		RangerAverage[ranger].queue.pop();

	RangerAverage[ranger].queue.push(lastRange[ranger]);

	RangerAverage[ranger].average = RangerAverage[ranger].queue.PeekAverage()/1000;
}