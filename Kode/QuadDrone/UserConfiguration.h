#ifndef _UserConfigutation_H_
#define _UserConfigutation_H_

/****************************************************************************
				The AeroQuad Manual can be found here:
		http://aeroquad.com/showwiki.php?title=Book:AeroQuad+Manual
 ****************************************************************************/

#define AeroQuadMega_v21	// Arduino Mega with AeroQuad Shield v2.1
#define quadPlusConfig		//Det skulle være denne.

// MOTOR ADVANCE CONFIG SECTION
//#define CHANGE_YAW_DIRECTION	// only needed if you want to reverse the yaw correction direction

#define USE_400HZ_ESC			// For ESC that support 400Hz update rate, ESC OR PLATFORM MAY NOT SUPPORT IT


//
// *******************************************************************************************************************************
// Optional Sensors
// Warning:  If you enable HeadingMagHold or AltitudeHold and do not have the correct sensors connected, the flight software may hang
// For more information on how to activate theese features with your transmitter
// Please refer to http://aeroquad.com/showwiki.php?title=Using+the+transmitters+sticks+and+switches+to+operate+your+AeroQuad
// *******************************************************************************************************************************
#define AltitudeHoldBaro			// Enables Barometer
#define AltitudeHoldRangeFinder	// Enables Altitude Hold with range finder, not displayed on the configurator (yet)
#define AutoLanding				// Enables auto landing on channel AUX3 of the remote, NEEDS AltitudeHoldBaro AND AltitudeHoldRangeFinder to be defined

//*******************************************************************************************************************************
// Receiver Setup
// For more information on how to connect your receiver to your AeroQuad board please refer to http://aeroquad.com/showwiki.php?title=Connecting+the+receiver+to+your+AeroQuad+board
// *******************************************************************************************************************************

//
// *******************************************************************************************************************************
// Define how many channels are connected from your R/C receiver
//*******************************************************************************************************************************
#define LASTCHANNEL 6

#endif