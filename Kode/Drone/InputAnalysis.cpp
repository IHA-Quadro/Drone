#include "InputAnalysis.h"

#define SIDE_SONAR_DISTANCE 120 //cm
#define FRONT_SONAR_DISTANCE 100 //cm

int _radioProgram;
int _sonarProgram;
bool _leftWarning;
bool	_rightWarning; 
bool	_frontWarning;

void ResetInputAnalysis()
{
	_radioProgram = 0;
	_sonarProgram = 0;

	_leftWarning = false;
	_rightWarning = false;
	_frontWarning = false;

	SetupRadioCommunicaiton();
}

//Check if drone is too close to an object
void AnalyseSonarInput()
{	
	if(programInput.data.aux1 == ALTITUDEHOLDTRUE)
	{
		_frontWarning = (RangerAverage[FRONT_RANGE_FINDER_INDEX].average * 100 + 8 < FRONT_SONAR_DISTANCE) ? true : false;

		_rightWarning = (RangerAverage[RIGHT_RANGE_FINDER_INDEX].average * 100 + 8 < SIDE_SONAR_DISTANCE) ? true : false;

		_leftWarning = (RangerAverage[LEFT_RANGE_FINDER_INDEX].average * 100 + 8 < SIDE_SONAR_DISTANCE) ? true : false;
	}
}

//Read from input queue
void AnalyseRadioInput()
{
	ReadRadio();
	_radioProgram = radioProgram;
}

//Return radio selected program
int GetRadioProgram()
{
	return _radioProgram;
}

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
