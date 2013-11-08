#include "InputAnalysis.h"

#define SIDE_SONAR_DISTANCE 80
#define FRONT_SONAR_DISTANCE 30

int _radioProgram;
int _sonarProgram;
//int _serialProgram;
bool _leftWarning;
bool	_rightWarning; 
bool	_frontWarning;

void ResetInputAnalysis()
{
	_radioProgram = 0;
	_sonarProgram = 0;
//	_serialProgram = 0;

	_leftWarning = false;
	_rightWarning = false;
	_frontWarning = false;

	SetupRadioCommunicaiton();
}

void AnalyseSonarInput()
{
	//Check if drone is too close to an object
	if(RangerAverage[FRONT_RANGE_FINDER_INDEX].average < FRONT_SONAR_DISTANCE)
	{
		_frontWarning = true;
	}

	if(RangerAverage[RIGHT_RANGE_FINDER_INDEX].average < SIDE_SONAR_DISTANCE)
	{
		_rightWarning = true;
	}

	if(RangerAverage[LEFT_RANGE_FINDER_INDEX].average < SIDE_SONAR_DISTANCE)
	{
		_leftWarning = true;
	}
}

//Read from input queue
void AnalyseRadioInput()
{
	int rssiValue = ReadRadio();
	//printNewLine(rssiValue, RADIOMODE);

	//float average = 0;
	//int count = queue2.count();
	//int number;
	//printInLine("Count: ", RADIOMODE);
	//printInLine(count, RADIOMODE);

	//while(queue2.count() != 0)
	//{
	//	number = queue2.pop();
	//	average += number;
	//}
	//average = average/count;

	//printNewLine(average, RADIOMODE);
	//_radioProgram = radioProgram;
	//queue2.~QueueList();
}

//Return radio selected program
int GetRadioProgram()
{
	return _radioProgram;
}

//Return Serial selected program
//int GetSerialProgram()
//{
//	return _serialProgram;
//}

//Return warning if left sonar is too close to object
bool GetLeftWarning()
{
	return _leftWarning;
}

//Return warning if right sonar is too close to object
bool GetRightWarning()
{
	return _rightWarning;
}

//Return warning if front sonar is too close to object
bool GetFrontWarning()
{
	return _frontWarning;
}
