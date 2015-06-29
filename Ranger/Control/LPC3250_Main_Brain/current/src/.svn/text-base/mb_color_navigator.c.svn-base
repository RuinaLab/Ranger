/*
	This code the highest level navigation software for automonous steering 
	indoors around Barton Hall track.

	Assuming robot is traveling CCW around track:

	Receives aproximate angle offset from tangent of track 
		* -90 to 90 degrees
		* -90 = current heading is perpendicular to the left of desired path
		* 90 = current heading is perpendicular to the right
		* 0 = current path is desired tragectory along track
	
	Receives distance from red/green border
		* [-1,1]
		* 1 -> 1 meter(s) to the left from desired position along track edge
		* -1 -> 1 meter(s) to the right from desired position along track edge

	Computes desired tragectory, and sends commands to steer Ranger

	Calculates where along track the robot should be, reconciles discrepancies 
		between inputs and theroical location of robot on track.

	Written by David Bjanes (dab355) Spring 2011


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

//****************************** Includes ******************************	

#include "mb_color_navigator.h"						

/*
 *	Function called from main block of code
 *  Returns true (1) if sucessful, 0, if unsucessful
 */ 
int Controller()	{
	
	// Robot will turn this angle in this timestep
	// DEFAULT = no change in current heading
	double angleOfTurn = 0;	


	// INITIALIZE VALUES
	initialize();	
				

	// Checks for that remode control option is disabled
	if (rcDisabled) {		  	
		return false;
	}

	// Set ERROR flag when updating values
	int ERROR = updateVar();			

	// Runs linear controller w/Control Law (calculates next step)
	linearAngle = linearController();
	
	// Checks/updates computer model to see if robot is on 
	// desired trajectory
	modelAngle = modelController();	
	
	
	//CURRENTLY SET FOR ALL SENSOR INPUT
	int c1 = 1;
	int c2 = 0;
	if (ERROR == true)
	{
		//STEER WITHOUT SENSORY INPUT
		c1 = 0; c2 = 1;
	}  	

	// CONTROL LAW - MODEL VS. SENSOR DATA
	angleOfTurn = c1 * linearAngle + c2 * modelAngle;
							 
	// Send commands to steering board (set values of enums)							
	steer(angleOfTurn);
	
	// Update robot model of location
	updateModel(angleOfTurn);	
		
	return ERROR;
}


/*
 *	Commands robot to steer
 */
int steer(double phi) {

	//TODO: SEND COMMAND TO ROBOT TO STEER PHI DEGREES

	return false;
}


/*
 *	SecondTierControl
 */
double* calculateSensorData(double sensorREV_Input, double sensorFWD_Input ) 
{
	//Range Of Inputs [-1, 1]
    
    y = sensorREV_Input - sensorFWD_Input;
    x = sqrt(DISTANCE_FROM_ROBOT_TO_LOCATION_SAMPLED_CAM1^2 +
			 DISTANCE_FROM_ROBOT_TO_LOCATION_SAMPLED_CAM2^2    );
    
	double output[2];

	output[0] = atan2(y,x);									//angleOfRobot
	output[1] = (sensorREV_Input + sensorFWD_Input)/2; 		//distToLine
	
	return output;
}


/*
 	Updates global varibles
 		Scales varibles such that 
 			- disFromTrack = double [-3, 3]
 			- angle = double [-90,90]
 
 	Returns true if no errors and correctly updates, false o/w
 */
int updateVar() {

	// IF ERROR OCURRS SET FLAG
	int ERROR = true;

	//TODO: UPDATE THESE VALUES
	rcDisabled = true;

	//TODO: UPDATE THESE VALUES
	/*
	 * 	Range from [-1, 1]: Orientied such that if robot is walking counter clock wise 
	 * 		around track:
	 *		if robot is further on GREEN inside (i.e. too far left of desired line)
	 *			inputs > 0
	 *		if robot is further on RED outside (i.e. too far right of desired line)
	 *			inputs < 0
	 */
	double sensorREV_Input = 0;
	double sensorFWD_Input = 0;

	double *calculated_Data

	calculated_Data = calculateSensorData(sensorREV_Input, sensorFWD_Input);

	/*
	 * 	Orientied such that if robot is walking counter clock wise around track:
	 *		if robot must turn right to steer back to track heading, angle < 0
	 *	    if robot must turn left to steet back to track heading, angle > 0
	 */	
	estAngleFromTrack = calculated_Data[0];	 	//[degrees]

	/*
	 * 	Orientied such that if robot is walking counter clock wise around track:
	 *		if robot is further on GREEN inside (i.e. too far left of desired line)
	 *			estDisFromTrack > 0
	 *		if robot is further on RED outside (i.e. too far right of desired line)
	 *			estDisFromTrack < 0
	 */
	estDisFromTrack = calculated_Data[1];	  	//[meters]

	//TODO: UPDATE THESE VALUES
	/*
	 * 	Forward motion corresponds to positive distance traveled
	 */
	estDisTraveled = 0;		  	//[meters]

				
	return ERROR;
}



/*
 	Updates computer model
 	Uses estimated distance travled, angle, to calculate where robot is
 */				    
void updateModel(double phi) {
	
	globalLoc = newLocation(globalLoc, estDisTraveled, phi);

}



/*
 * 	Calculate Next Position Based on internal model 
 */
double modelController() {

	 //TODO: CONTROLLER?

	 return 0;
 }


/*
 	Updates computer model
 	Uses estimated distance travled, angle, to calculate where robot is
 	Estimated position
 */
double linearController() {
	
	// DEFAULT NO CORRECTION OF CURRENT TRAJECTORY
	double phi = 0;			

	// THRESHOLD, correct steering only if robot is greater than MIN_DIST_FROM_LINE
	if (abs(estDisFromTrack) >  MIN_DIST_FROM_LINE) 
	{
		// CONTROL LAW
		// phi [degrees] = [const]*[meters] + [const]*[degrees]
		phi	= SPRING_TERM * estDisFromTrack + DAMPING_TERM * -estAngleFromTrack;


		// MAX STEERING ANGLE OF RANGER
		// LIMITATION OF ROBOT HARDWARE 
		if (phi > MAX_TURN_RADIUS)
		{
			phi = MAX_TURN_RADIUS;
		}
		else if (phi < -MAX_TURN_RADIUS) 
		{
			phi = -MAX_TURN_RADIUS;  
		}
	}
	
	return phi;	
}



