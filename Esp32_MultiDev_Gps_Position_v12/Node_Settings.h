/**
* This file defines the node specific data that should be changed for each node (sensor).
* 
*/

#define GPS_DATA_INTERVAL_SECONDS 10 // the GPS data were retrieved every xx interval seconds

// select flipped orientation if uncommented
//#define DISPLAY_ORIENTATION_FLIPPED   // display 180 degrees right rotated 

#define WaitGPSFixSeconds 10        // time in seconds to wait for a new GPS fix 
#define WaitFirstGPSFixSeconds 120  // time to seconds to wait for the first GPS fix at startup

// taken from Google Maps: Rhine Tower in Duesseldorf, Northrhine Westfalia, Germany
// 51.21800467274397, 6.761672316462719
#define GPS_REFERENCE_LAT 51.21800467274397
#define GPS_REFERENCE_LON 6.761672316462719
