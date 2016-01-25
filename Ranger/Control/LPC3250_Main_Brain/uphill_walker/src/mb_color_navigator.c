/*
	This code the highest level navigation software for automonous steering 
	indoors around Barton Hall track.

	Receives aproximate angle offset from tangent of track 
		* -90 to 90 degrees
		* -90 = current heading is perpendicular to the left of desired path
		* 90 = current heading is perpendicular to the right
		* 0 = current path is desired tragectory along track
	
	Receives distance from red/green border
		* 0-255
		* 0 = 3 meters to the left from desired position along track edge
		* 255 = 3 meters to the right from desired position along track edge

	Computes desired tragectory, and sends commands to steer Ranger

	Calculates where along track the robot should be, reconciles discrepancies 
		between inputs and theroical location of robot on track.

	Written by David Bjanes (dab355) Spring 2011

*/

//***********************************************************************************************

// Helper Functions
#include "mb_color_navigator.h"


/*
 	Track State Machine
		Barton track reigions
		

		Assumed starting point
				 || 
		 		 ||
		(+y)  	 \/
		 /\		 |  ~North  I   |
		  |	   	 *--------------|
		  |	   / | 				| \ 
statler	  |	 /	  				    \			 lynah
		  |/	 |				|	  \
	~West |		 			 	       |  ~East II
	  IV  |----(0,0)------------|------|--------------->	(+x)						 	 
		  |		 					   |
		  |\	 |			  	|     /
		  |	 \	 				    /
		  |	   \ |				| / 
     	 \/	     |--------------|	
        (-y)     |	III ~South	|
				
		
*/								
enum location {north, south, east, west};

typedef struct{	double x; double y; }; position; 


/*
 *	Global Definitions
 */
#define true 1
#define false 0
#define tLength 100	 		// Filler Values - Need Real Data
#define tRadius	50	 		// Filler Values - Need Real Data


//**********************************************************************************************

/*
 *	Global Varibles
 */
double angleFromTrack;		// 0-255 -> corresponds to -90 to 90 degress (see above)
double disFromTrack;		// 0-255 -> corresponds to -3 to 3 meters (see above)

double estDisTraveled;		// Estimated distance traveled in last timestep
double estAngleOfPrevTurn;	// Estimated last angle of turn

int rcDisabled; 			// If remote control is enabled, this flag will be false = 0;

position globalLoc		// Global Location of Ranger on digital map	 
 	

//**********************************************************************************************

/*
 *	Function called from main block of code
 *  Returns true (1) if sucessful, -1, if unsucessful
 */ 
int steer()	{
	rcDisabled = true;		   	// For debugging purposes, always set to true
	globalLoc.x = 0;			// For debugging purposes, assume default starting point
	globalLoc.y = tRadius;		// For debugging purposes, assume default starting point



	if (rcDisabled) {		  	// Checks for that remode control option is disabled
		return -1;
	}

	// If an error occurs
	if (updateVar() == 0) {		// Checks enums for changes to inputs
								// updates angleFromTrack, disFromTrack
	   							
	}

	updateModel();	 			// Checks/updates computer model to see if robot is on 
								// desired trajectory
								// Updates state machine

	commandSteer();				// Send commands to steering board (set values of enums)
}


/*
 	Updates global varibles
 		Scales varibles such that 
 			- disFromTrack = double [-3, 3]
 			- angle = double [-90,90]
 
 	Returns true if no errors and correctly updates, false o/w
 */
int updateVar() {


}


/*
 	Updates computer model
 	Uses estimated distance travled, angle, to calculate where robot is
 	
 	Ideal position (No Error Correction) 
 		assumes only error is perpendicular distance from track edge
 		assumes	estDisTraveled is perfectly parallel with track edge
 */				    
void updateModel() {

	switch (location)
	{
		case north:	globalLoc.x += estDisTraveled; 
					globalLoc.y = tRadius * 2 + disFromTrack;
					break;

		case east:	globalLoc.x = 
					globalLoc.y = 
					break;

		case south:	globalLoc.x -= estDisTraveled; 
					globalLoc.y = disFromTrack;
					break;

		case west:	globalLoc.x += 
					globalLoc.y = 
					break;

		default: break; // Do nothing

	}
}


/*
 	Updates computer model
 	Uses estimated distance travled, angle, to calculate where robot is
 	Estimated position
 */
void commandSteer() {
	

}



//*******************(ALL BELOW - MOVE TO *.h file)************************************************


/*
 	Helper Function 
 	Compute Hypotenuse
 */
double hypoten (position a, position b) {

	return sqrt( (a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y) );

}


/*
 	Helper Function (MOVE TO *.h file)
 	Compute new position from hypotenuse and angle
 */
position newLocation (position old, double hypoten, double angle) {


}

	   