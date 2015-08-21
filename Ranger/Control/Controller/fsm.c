#include <mb_includes.h> 
#include <motorControl.h>
#include <fsm.h>
#include <RangerMath.h>

#define PI 3.141592653589793
#define SCISSOR_OFFSET 0.1
#define	SCISSOR_RATE 1.3

/* States for the FSM */
enum States {
	SET_UP,
	OUT_GLIDE,
	OUT_PUSH,
	INN_GLIDE,
	INN_PUSH,
	HOLD_DOUBLE,
	FLIGHT,
};

static enum States current_state = SET_UP; //keeps track of the current state the FSM is in 

static const float l = 0.96;  // robot leg length
static const float d = 0.14;  // robot foot joint eccentricity
static const float Phi = 1.8;  // ankle joint orientation constant

////TODO - what is the behavior of float vs static float in the following sense:  ?
/*
- A global static variable is one that can only be accessed in the file where it is created. This variable is said to have file scope.
- A global variable is a variable that is defined outside all functions and available to all functions. These variables are unaffected by scopes and are always available, which means that a global variable exists until the program ends.
*/

/* Paramers used in the FSM */
float ANK_FLIP_KP; //medium high
float ANK_FLIP_KD;
float ANK_PUSH_KP; //high, more push off
float ANK_PUSH_KD;
float ANK_HOLD_KP; //low, moderately hold the feet, easier to push off
float ANK_HOLD_KD;
float HIP_KP;
float HIP_KD; 
float HIP_REF_HOLD; //relative reference angle for the hip 
float HIP_REF_TRANS_ANGLE;  
float ANK_REF_HOLD; //absolute refernce angles for the ankles
float ANK_REF_PUSH;
float ANK_REF_FLIP;

/* Read in values for parameters used in fsm.c from Labview. */
void param_update(void){
	HIP_REF_HOLD = mb_io_get_float(ID_CTRL_HIP_REF_HOLD);
	HIP_REF_TRANS_ANGLE = mb_io_get_float(ID_CTRL_HIP_TRANS_ANGLE);
	ANK_REF_HOLD = mb_io_get_float(ID_CTRL_ANK_REF_HOLD); 
	ANK_REF_PUSH = mb_io_get_float(ID_CTRL_ANK_REF_PUSH);
	ANK_REF_FLIP = mb_io_get_float(ID_CTRL_ANK_REF_FLIP);
	ANK_FLIP_KP = mb_io_get_float(ID_CTRL_ANK_FLIP_KP);	 
	ANK_FLIP_KD = mb_io_get_float(ID_CTRL_ANK_FLIP_KD);
	ANK_PUSH_KP = mb_io_get_float(ID_CTRL_ANK_PUSH_KP);	 
	ANK_PUSH_KD = mb_io_get_float(ID_CTRL_ANK_PUSH_KD);
	ANK_HOLD_KP =  mb_io_get_float(ID_CTRL_ANK_HOLD_KP); 
	ANK_HOLD_KD =  mb_io_get_float(ID_CTRL_ANK_HOLD_KD);
	HIP_KP = mb_io_get_float(ID_CTRL_HIP_KP);
	HIP_KD= mb_io_get_float(ID_CTRL_HIP_KD); 
}

//Ranger goes into flight mode when it's been lifted up in the air for a duration longer than 200 clock cycles (~0.4s).   
//Variable flight_count keeps track of the number of continuous clock cylces that Ranger's been in the air 
static const int flight_threshold = 200; 
static int flight_count = 0; 

/* Sets the initial state of the FSM. */
void fsm_init(void){
	current_state = SET_UP;
	flight_count = 0;
}

/* Runs the current state the FSM is in. 
 * Parameters:
 *		ID_CTRL_TEST_W1 = current_state;	
 */
void fsm_run(void){
	struct ControllerData ctrlHip;
	struct ControllerData ctrlAnkOut;
	struct ControllerData ctrlAnkInn;
	
	//turns on gravity compensation in the hip motor
	ctrlHip.GC = 1;
	//turns off gravity compensation in the ankle motors
	ctrlAnkOut.GC = 0;
	ctrlAnkInn.GC = 0;

	angles_update();  //updates all angle variables used in the code 
	fsm_update(); 	//updates FSM current state

	switch (current_state){
	case SET_UP: /*initial setup done in the air*/
		/* gets the robot ready for walking by positioning the ankle joints to the right angles in the air */
		mb_io_set_float(ID_CTRL_TEST_W1, 0);
		// hold outer feet and flip up inner feet
		out_ank_track_abs(&ctrlAnkOut, ANK_REF_HOLD, 0.0, 0.0, ANK_HOLD_KP, ANK_HOLD_KD);
		inn_ank_track_abs(&ctrlAnkInn, ANK_REF_FLIP, 0.0, 0.0, ANK_FLIP_KP, ANK_FLIP_KD);
		// hip tracks zero angle
		hip_track_rel(&ctrlHip, 0.0, 0.0, HIP_KP, HIP_KD);
		break;
	case OUT_GLIDE:	/*swing inner leg*/	  
		mb_io_set_float(ID_CTRL_TEST_W1, 1);
		// hold outer feet and flip up inner feet
		out_ank_track_abs(&ctrlAnkOut, ANK_REF_HOLD, 0.0, 0.0, ANK_HOLD_KP, ANK_HOLD_KD);
		inn_ank_track_abs(&ctrlAnkInn, ANK_REF_FLIP, 0.0, 0.0, ANK_FLIP_KP, ANK_FLIP_KD);
		// adjust hip angle, outer on ground
		hip_scissor_track_outer(&ctrlHip, SCISSOR_OFFSET, SCISSOR_RATE,HIP_KP, HIP_KD);
		break;
	case OUT_PUSH:	/*push off outer feet and land inner feet*/					   
		mb_io_set_float(ID_CTRL_TEST_W1, 2);
		// push down outer feet and hold inner feet
	    out_ank_track_abs(&ctrlAnkOut, ANK_REF_PUSH, 0.0, 0.0, ANK_PUSH_KP, ANK_PUSH_KD);
	 	inn_ank_track_abs(&ctrlAnkInn, ANK_REF_HOLD, 0.0, 0.0, ANK_HOLD_KP, ANK_HOLD_KD);
	    hip_track_rel(&ctrlHip, HIP_REF_HOLD, 0.0, HIP_KP, HIP_KD);
		break;
	case INN_GLIDE:	/*swing outer leg*/
		mb_io_set_float(ID_CTRL_TEST_W1, 3);
		// flip up outer feet and hold inner feet
		out_ank_track_abs(&ctrlAnkOut, ANK_REF_FLIP, 0.0, 0.0, ANK_FLIP_KP, ANK_FLIP_KD);
		inn_ank_track_abs(&ctrlAnkInn, ANK_REF_HOLD, 0.0, 0.0, ANK_HOLD_KP, ANK_HOLD_KD);
		hip_scissor_track_inner(&ctrlHip, SCISSOR_OFFSET, SCISSOR_RATE, HIP_KP, HIP_KD);
		break;
	case INN_PUSH:	/*push off inner feet and land outer feet*/
		mb_io_set_float(ID_CTRL_TEST_W1, 4);
		//push down inner feet and hold outer feet
		out_ank_track_abs(&ctrlAnkOut, ANK_REF_HOLD, 0.0, 0.0, ANK_HOLD_KP, ANK_HOLD_KD);
		inn_ank_track_abs(&ctrlAnkInn, ANK_REF_PUSH, 0.0, 0.0, ANK_PUSH_KP, ANK_PUSH_KD);
		hip_track_rel(&ctrlHip, -HIP_REF_HOLD, 0.0, HIP_KP, HIP_KD);
		break;
	case HOLD_DOUBLE: /*double stance*/   ////TODO: - what are we doing here?
	////TODO - this is not used - say so, and why	(panic state)
		/* This state is intended to be a panic state, that puts Ranger in double stance when it falls the wrong direction, 
		 but it's currently not being used. */
		mb_io_set_float(ID_CTRL_TEST_W1, 5);
		// hold outer feet
		out_ank_track_abs(&ctrlAnkOut, ANK_REF_HOLD, 0.0, 0.0, ANK_HOLD_KP, ANK_HOLD_KD);
		// hold inner feet
		inn_ank_track_abs(&ctrlAnkInn, ANK_REF_HOLD, 0.0, 0.0, ANK_HOLD_KP, ANK_HOLD_KD);
		// adjust hip angle
		if(th0>0){
			hip_track_rel(&ctrlHip, -HIP_REF_HOLD, 0.0, HIP_KP, HIP_KD); //outer leg is in front
		}else{
			hip_track_rel(&ctrlHip, HIP_REF_HOLD, 0.0, HIP_KP, HIP_KD);	//inner leg is in front
		}
		break;
	case FLIGHT: /*flight mode: disables all motors when robot is in the air */ 
		mb_io_set_float(ID_CTRL_TEST_W1, 6);
		motor_off(&ctrlAnkOut);
		motor_off(&ctrlAnkInn);
		motor_off(&ctrlHip);
		flight_count = 0;
		break;	
	default: /*state doesn't exist*/
		////TODO - pass error message to LabVIEW?
		break;
	}

	// run the PD controllers if current state is not the flight state 
	controller_hip(&ctrlHip);
	controller_ankleInner(&ctrlAnkInn);
	controller_ankleOuter(&ctrlAnkOut);
}


/* Updates current state of the FSM. */
void fsm_update(void){
 	// Enters the flight state if all feet are off ground for 200 continuous clock cycles 
	if(current_state != SET_UP){ 
		// checks for ground contact when it's not in the setup state 
		// because Ranger is always in the air during setup and we don't want to disable its motors
		if(!FI_on_ground() && !FO_on_ground()){
			flight_count++;
		}else{
			flight_count = 0;
		}
		if(flight_count >= flight_threshold){
			current_state = FLIGHT;
		}
	}

	switch(current_state){
	case SET_UP:
		// ready to walk when placed on ground					
		if(FO_on_ground()){
			current_state = OUT_GLIDE;	
		}
		break;
	case OUT_GLIDE:	/*swing inner leg*/
		if(th0 < -HIP_REF_TRANS_ANGLE){ //inner leg in front, outer leg in the back 
			current_state = OUT_PUSH;
		} 		
		/*if( th0<-trans_angle && dth0<0 ){ //falling forward, push off
			current_state = OUT_PUSH;	
		}else if( th0>trans_angle && dth0>0 ){ //falling backward, emergency!
			current_state = HOLD_DOUBLE;
		}*/
		break;
	case OUT_PUSH: /*land inner leg*/
		if(FI_on_ground()){ //inner feet have landed 
		 	correct_gyro_angle(); //all feet are on ground
			current_state = INN_GLIDE;
		}
		break;
	case INN_GLIDE: /*swing outer leg*/
		if(th0 > HIP_REF_TRANS_ANGLE){ //outer leg in front, inner leg in the back 
			current_state = INN_PUSH;	
		}
		/*if( th1<-trans_angle && dth1<0 ){ //falling forward, push off
			current_state = INN_PUSH;	
		}else if( th1>trans_angle && dth1>0 ){ //falling backward, emergency!
			current_state = HOLD_DOUBLE;
		} */
		break;
	case INN_PUSH: /*land outer leg*/
		if(FO_on_ground()){ //outer feet have landed
			correct_gyro_angle(); 
			current_state = OUT_GLIDE;
		}
		break;
	case HOLD_DOUBLE: /*double stance*/ 
		current_state = HOLD_DOUBLE; //absorbing state, no exit transition
		break;
	case FLIGHT:
		current_state = FLIGHT;	  //absorbing...	
		break;
	default: /*state doesn't exist*/
		break;
	}
}			   


/* Corrects the gyro angle (integrated from gyro rate) every step Ranger takes
 * using the gyro angle (calculated from geometry). 
 * Sets the corrected gyro angle in the estimator code. 
 * Requires: 
 *		Both feet on ground & the function angles_update() called 
 */
void correct_gyro_angle(void){
	float Slope = 0.0;  // Ground slope (assume linear)
	float x, y, stepLength, qr_geo, qr_int, qr_new;

	/* Ranger geometry:
	 * [x;y] = vector from outer foot virtual center to the inner foot
	 * virtual center, in a frame that is rotated such that qr = 0
	 * These functions were determined using computer math. The code can
	 * be found in:
	 * templates/Estimator/legAngleEstimator/Derive_Eqns.m
	 */
	x = l*Sin(qh) - d*Sin(Phi - q1 + qh) + d*Sin(Phi - q0);
	y = l + d*Cos(Phi - q1 + qh) - l*Cos(qh) - d*Cos(Phi - q0);

	stepLength = Sqrt(x*x + y*y);
	qr_geo = Atan(y/x) + Slope;	 //gyro angle calculated from geometry 
	qr_int = get_prev_gyro_angle();	//gyro angle integrated from gyro rate (from the preivous ms)
	qr_new = 0.95*qr_int + 0.05*qr_geo;	//new gyro angle that's a weighted average of the two above
										//NOTE: ratio of 0.9:0.1 (used in Anoop's code) makes Ranger fall forward 

	set_gyro_angle(qr_new);	//updates the gyro angle in the estimator code
	
	return;
}
