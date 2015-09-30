#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <time.h>


//****************************** Global Definitions ******************************

//CONTROL LAW DEFINITIONS					 	
#define MIN_DIST_FROM_LINE 0		// [meters]
#define MAX_TURN_RADIUS 7			// [degrees]
#define SPRING_TERM	-24
#define DAMPING_TERM -1.9

//GENERAL DEFINITIONS
#define true 1
#define false 0
#define tLength 100	 		// Filler Values - Need Real Data
#define tRadius	50	 		// Filler Values - Need Real Data

#define DISTANCE_FROM_ROBOT_TO_LOCATION_SAMPLED_CAM1 = 1;   % [meters]
#define DISTANCE_FROM_ROBOT_TO_LOCATION_SAMPLED_CAM2 = 1;   % [meters]

#define initialRobotHeadingAngle 0		// [degrees]

typedef struct { double x; double y; } position; 


//****************************** Global Varibles ******************************

double estAngleFromTrack;		// Corresponds to -90 to 90 degress (see *.c)
double estDisFromTrack;			// Corresponds to -1 to 1 meters (see *.c)

double estDisTraveled;			// [m] Estimated distance traveled in last timestep
double estAngleOfPrevTurn;		// [degrees] Estimated last angle of turn

double curRobotHeadingAngle;	// [degrees]

int rcDisabled; 				// If remote control is enabled, this flag will be false = 0;

position globalLoc;				// Global Location of Ranger on digital map	 


//****************************** MAIN CODE ******************************

int Controller();
int steer(double phi);
int updateVar();
double* calculateSensorData(double sensorREV_Input, double sensorFWD_Input);
void updateModel(double phi);
double modelController();
double linearController();


//****************************** HELPER FUNCTIONS ******************************  

/*
	Initialize Robot Location and internal map
 */
 void initialize () {
 	curRobotHeadingAngle = 0;

	globalLoc.x = 0;			// For debugging purposes, assume default starting point
	globalLoc.y = tRadius;		// For debugging purposes, assume default starting point

	rcDisabled = true;		   	// For debugging purposes, always set to true

	//TODO: CALCULATE TRACK
}



/*
 	Helper Function 
 	Compute Hypotenuse
 */
double hypotenuse (position a, position b) {

	return sqrt( (a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y) );

}



/*
 	Helper Function (MOVE TO *.h file)
 	Compute new position from hypotenuse [m] and angle [degrees]
 */
position newLocation (position old, double stridelen, double angle) {
	
	position newloc;

	newloc.y = old.y + stridelen * sin(angle * pi/180);
	newloc.x = old.x + stridelen * cos(angle * pi/180);

	return newloc;
} 	   
