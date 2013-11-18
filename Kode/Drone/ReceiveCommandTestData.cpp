//#include "ReceiveCommandTestData.h"
//
//int _stepSize = 5;
//bool _inverseSteps = false;
//int _starterSpeed = 1200;
//int throttleSpeed = _starterSpeed;
//int miliSecCounter = 0;
//int miliSecCounterStamp = 0;
//bool miliSecCounterActive = false;
//bool stableHeight = false;
//
//int data[2] = {0,0};
//
//void ResetReceiveCommandTestData()
//{
//	_inverseSteps = false;
//	isAutoLandingInitialized = false;
//	miliSecCounterActive = false;
//	inititalizeRangeFinders();
//}
//
//void TestAxis(byte axis)
//{
//	int step = _stepSize;
//	_controllerInput[THROTTLE] = 1250;
//
//	_controllerInput[axis] += step;
//
//	//Inverse step on lower an upper
//	_inverseSteps = (_controllerInput[axis] < lower || _controllerInput[axis] > upper ? true : false);
//
//	if(_inverseSteps == true)
//		_stepSize = _stepSize*(-1);
//
//	IsMotorKilled();
//}
//
//void RotateDrone(int zaxis)
//{
//	if(zaxis != 1500)
//	{
//		printInLine("Rotating drone ", STATUSMODE);
//
//		if(zaxis < 1500)
//			printInLine("to the left to ", STATUSMODE);
//		else
//			printInLine("to the right to ", STATUSMODE);
//
//		printNewLine(zaxis, STATUSMODE);
//	}
//
//	_controllerInput[ZAXIS] = zaxis;
//}
//
//void MoveDrone(int xaxis, int yaxis)
//{
//	if(xaxis != 1500)
//	{
//		printInLine("Tilting drone ", STATUSMODE);
//
//		if(xaxis < 1500)
//			printInLine("to the left to ", STATUSMODE);
//		else
//			printInLine("to the right to ", STATUSMODE);
//
//		printNewLine(xaxis, STATUSMODE);
//	}
//
//	if(yaxis != 1500)
//	{
//		printInLine("Tilting drone ", STATUSMODE);
//
//		if(yaxis < 1500)
//			printInLine("backwards to ", STATUSMODE);
//		else
//			printInLine("forward to ", STATUSMODE);
//
//		printNewLine(yaxis, STATUSMODE);
//	}
//
//	_controllerInput[XAXIS] = xaxis;
//	_controllerInput[YAXIS] = yaxis;
//}
//
//static void MiliSecOverflow(int timeSpanInMiliSec)
//{
//	if(miliSecCounter > timeSpanInMiliSec+10)
//	{
//		miliSecCounterActive =false;
//	}
//}
//
////Use initTime to accelerate to 
//void AccelerateSpeed(int maxSpeed, int initTime)
//{
//	if(miliSecCounter < initTime) //Wait before real speed is applied
//	{
//		if(spinSpeed < 1250)
//			spinSpeed += 5;
//	}
//	else
//	{
//		if(spinSpeed >= maxSpeed)
//			spinSpeed = maxSpeed;
//		else
//			spinSpeed += 10;
//
//		MiliSecOverflow(initTime);
//	}
//}
//
////Measure specific sensor
////Output in cm
//int MeasureSonar(byte sonarId)
//{
//	return (int)(RangerAverage[sonarId].average *100) + 8; 
//}
//
//void PrintSonarData(byte sonarID)
//{
//	printNewLine(MeasureSonar(sonarID), STATUSMODE);
//}
//
//static void NotLanding()
//{
//	_controllerInput[AUX3] = AUTOLANDFALSE;
//	isAutoLandingInitialized = false;
//}
//
//int calcAverage(int height)
//{
//	data[0] = data[1];
//	data[1] = height;
//
//	return (data[0]+data[1])/2;
//}
//
////Keep height at first parameter, accelerating with second paramter's time 
//static void LiftToSonar(int steadyHeight, int initTime)
//{
//	NotLanding();
//
//	if(_controllerInput[AUX1] == ALTITUDEHOLDTRUE) //If autohold enable
//		return; //Skip the rest
//
//	int sonarHeight = MeasureSonar(ALTITUDE_RANGE_FINDER_INDEX); //Bottom sonar
//
//	int average = calcAverage(sonarHeight);
//
//	if( (average + STEADYTOLERANCE > steadyHeight) && (average - STEADYTOLERANCE < steadyHeight))
//	{
//		if(!stableHeight)
//			printNewLine("Enabling Altitude Hold", STATUSMODE);
//
//		_controllerInput[AUX1] = ALTITUDEHOLDTRUE;
//		stableHeight = true;
//	}
//
//	else if(average + STEADYTOLERANCE < steadyHeight) //Not high enough
//	{
//		stableHeight = false;
//
//		if(miliSecCounter > initTime) //Still not high enough after initTime
//		{
//			miliSecCounter = 0;
//			spinSpeed += 10;
//			printInLine("SpinSpeed = ", STATUSMODE);
//			printInLine(spinSpeed, STATUSMODE);
//			printInLine(" ; ", STATUSMODE);
//			printInLine(average, STATUSMODE);
//			println(STATUSMODE);
//		}
//	}
//	else if(average  - STEADYTOLERANCE > steadyHeight) //Too high 
//	{
//		stableHeight = false;
//
//		if(miliSecCounter > initTime) //Still too high after initTime
//		{
//			miliSecCounter = 0;
//			spinSpeed -= 5;
//			printInLine("SpinSpeed = ", STATUSMODE);
//			printInLine(spinSpeed, STATUSMODE);
//			printInLine(" ; ", STATUSMODE);
//			printInLine(average, STATUSMODE);
//			println(STATUSMODE);
//		}
//	}
//}
//
//static void LandDrone()
//{
//	_controllerInput[AUX1] = ALTITUDEHOLDTRUE; //Hold altitude?
//	_controllerInput[AUX3] = AUTOLANDTRUE; //Activate autolanding
//}
//
////Rotate drone to 'axisValue' and reset value after 'miliReset'-miliseconds
//void RotateDroneInTime(int axisValue, int miliReset)
//{
//	if(miliSecCounter < miliReset)
//		RotateDrone(axisValue);
//
//	else
//		RotateDrone(1500);
//}
//
////Tilt drone to a side and/or direction for 'miliReset'-milisecond, before reseting
//void TiltDroneInTime(int xAxisValue, int yAxisValue, int miliReset)
//{
//	if(miliSecCounter < miliReset)
//	{
//		if(xAxisValue != 0 && xAxisValue != 1500)
//		{
//			MoveDrone(xAxisValue, _controllerInput[YAXIS]);
//		}
//
//		if(yAxisValue != 0 && yAxisValue != 1500)
//		{
//			MoveDrone(_controllerInput[XAXIS], yAxisValue);
//		}
//	}
//
//	else
//		MoveDrone(1500, 1500);
//}
//
//void RunProgram(ProgramInput input)
//{
//	miliSecCounterActive = true;
//
//	if(input.ProgramID != _previousProgram.ProgramID)
//	{
//		miliSecCounter = 0;
//		_previousProgram = input;
//	}
//
//	switch (input.ProgramID)
//	{
//	case 1:
//		AeroQuadSetup();
//		break;
//
//	case 2:
//		AccelerateSpeed(input.AdditionalData, input.TimeSpanInMiliSec);
//		break;
//
//	case 3:
//		PrintSonarData(input.AdditionalData);
//		break;
//
//	case 4:
//		LiftToSonar(input.height, input.TimeSpanInMiliSec);
//		break;
//
//	case 5:
//		LandDrone();
//		break;
//
//	case 6:
//		RotateDroneInTime(input.data.zAxis, input.TimeSpanInMiliSec);
//		break;
//
//	case 7:
//		TiltDroneInTime(input.data.xAxis, input.data.yAxis, input.TimeSpanInMiliSec);
//		break;
//
//	default:
//		miliSecCounter = 0;
//		break;
//	}
//}
