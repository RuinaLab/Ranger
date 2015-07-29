#include <mb_includes.h> 
#include <motorControl.h>
#include <fsm.h>
#include <rangermath.h>

enum States {
	OUT_SWING,
	OUT_PUSH,
	INN_SWING,
	INN_PUSH,
	HOLD_DOUBLE,
};

#define PI 3.141592653589793
#define ANK_FAST_KP 50
#define ANK_FAST_KD 3 
#define ANK_SLOW_KP 30
#define ANK_SLOW_KD 15 
#define HIP_KP 40
#define HIP_KD 10
#define HIP_SCISSOR_GAIN 1.5
#define HIP_REF_HOLD 0.3 //relative angle for hip
#define ANK_REF_HOLD 0.0 //absolute angles for ankle
#define ANK_REF_PUSH -0.6
#define ANK_REF_FLIP 1.5
#define uMAX_ANK 4
#define uMAX_HIP 8 //2*uMAX_ANK


static float th_ref = 0.0;
static enum States current_state = OUT_SWING; 
static float qr, qh, dqr, dqh, q0, q1, dq0, dq1; //relative
static float th0, th1, dth0, dth1; //absolute 

/* Updates the global absolute/relative angles in Ranger. */
void angles_update(void){
	 // set relative angles/angular rates
	qr = get_out_angle();	//angle integrated from rate gyro	
	qh = get_in_angle();	//hip angle
	dqr = get_out_ang_rate();
	dqh = get_in_ang_rate();
	q0 = mb_io_get_float(ID_MCFO_RIGHT_ANKLE_ANGLE);
	q1 = mb_io_get_float(ID_MCFI_MID_ANKLE_ANGLE);
	dq0 = mb_io_get_float(ID_MCFO_RIGHT_ANKLE_RATE);
	dq1 = mb_io_get_float(ID_MCFI_ANKLE_RATE);

	// set absolute angles
	th0 = -qr;		//angle of outer leg
	th1 = qh - qr;	//angle of inner leg 
	dth0 = -dqr; 
	dth1 = dqh - dqr;
}


/* Sets the initial state of the FSM. */
void fsm_init(void){
	current_state = OUT_SWING;
}


/* Runs current state the FSM is in. */
void fsm_run(void){
	struct ControllerData ctrlHip;
	struct ControllerData ctrlAnkOut;
	struct ControllerData ctrlAnkInn;
	float c0, c1;

	switch (current_state){
	case OUT_SWING:	/*swing inner leg*/
		// hold outer feet
		out_ank_track_abs(&ctrlAnkOut, ANK_REF_HOLD, 0.0, 0.0, ANK_SLOW_KP, ANK_SLOW_KD);
		// flip up inner feet
		inn_ank_track_abs(&ctrlAnkInn, ANK_REF_FLIP, 0.0, 0.0, ANK_FAST_KP, ANK_FAST_KD);
		// adjust hip angle, outer on ground
		c0 = HIP_SCISSOR_GAIN;
		c1 = 1.0;
		hip_scissor_track(&ctrlHip, c0, c1, HIP_KP, HIP_KD);
		break;

	case OUT_PUSH:	/*land inner leg*/
		// push down outer feet
	    out_ank_track_abs(&ctrlAnkOut, ANK_REF_PUSH, 0.0, 0.0, ANK_FAST_KP, ANK_FAST_KD);
		// hold inner feet
	 	inn_ank_track_abs(&ctrlAnkInn, ANK_REF_HOLD, 0.0, 0.0, ANK_FAST_KP, ANK_FAST_KD);
		// adjust hip angle	
		hip_track_rel(&ctrlHip, HIP_REF_HOLD, 0.0, HIP_KP, HIP_KD);
		break;

	case INN_SWING:	/*swing outer leg*/
		// flip up outer feet 
		out_ank_track_abs(&ctrlAnkOut, ANK_REF_FLIP, 0.0, 0.0, ANK_FAST_KP, ANK_FAST_KD);
		// hold inner feet
		inn_ank_track_abs(&ctrlAnkInn, ANK_REF_HOLD, 0.0, 0.0, ANK_SLOW_KP, ANK_SLOW_KD);
		// adjust hip angle
		c0 = 1.0;
		c1 = HIP_SCISSOR_GAIN;
		hip_scissor_track(&ctrlHip, c0, c1, HIP_KP, HIP_KD);
		break;

	case INN_PUSH:	/*land outer leg*/
		// hold outer feet
		out_ank_track_abs(&ctrlAnkOut, ANK_REF_HOLD, 0.0, 0.0, ANK_FAST_KP, ANK_FAST_KD);
		// push down inner feet
		inn_ank_track_abs(&ctrlAnkInn, ANK_REF_PUSH, 0.0, 0.0, ANK_FAST_KP, ANK_FAST_KD);
		// adjust hip angle
	    hip_track_rel(&ctrlHip, -HIP_REF_HOLD, 0.0, HIP_KP, HIP_KD);   
		break;

	case HOLD_DOUBLE: /*double stance*/
		// hold outer feet
		out_ank_track_abs(&ctrlAnkOut, ANK_REF_HOLD, 0.0, 0.0, ANK_FAST_KP, ANK_FAST_KD);
		// hold inner feet
		inn_ank_track_abs(&ctrlAnkInn, ANK_REF_HOLD, 0.0, 0.0, ANK_FAST_KP, ANK_FAST_KD);
		// adjust hip angle
		if(th0>0){
			hip_track_rel(&ctrlHip, -HIP_REF_HOLD, 0.0, HIP_KP, HIP_KD); //outer leg is in front
		}else{
			hip_track_rel(&ctrlHip, HIP_REF_HOLD, 0.0, HIP_KP, HIP_KD);	//inner leg is in front
		}
		break;
	
	default: /*state doesn't exist*/
		break;
	}
	
	// saturate the PD controllers
	smooth_saturate(&ctrlHip, uMAX_HIP, qh, dqh);
	smooth_saturate(&ctrlAnkOut, uMAX_ANK, q0, dq0);
	smooth_saturate(&ctrlAnkInn, uMAX_ANK, q1, dq1);

	// run the PD controllers 
	controller_hip(&ctrlHip);
	controller_ankleInner(&ctrlAnkInn);
	controller_ankleOuter(&ctrlAnkOut);
}


/* Updates current state of the FSM. */
void fsm_update(void){
	float trans_angle = 0.5*HIP_REF_HOLD;

	switch(current_state){
	case OUT_SWING:	/*swing inner leg*/
		if( th0<-trans_angle && dth0<0 ){ //falling forward, push off
			current_state = OUT_PUSH;	
		}else if( th0>trans_angle && dth0>0 ){ //falling backward, emergency!
			current_state = HOLD_DOUBLE;
		}
		break;
	case OUT_PUSH: /*land inner leg*/
		if(FI_on_ground()){ //inner feet have landed 
		 	current_state = INN_SWING;
		}
		break;
	case INN_SWING: /*swing outer leg*/
		if( th1<-trans_angle && dth1<0 ){ //falling forward, push off
			current_state = INN_PUSH;	
		}else if( th1>trans_angle && dth1>0 ){ //falling backward, emergency!
			current_state = HOLD_DOUBLE;
		}
		break;
	case INN_PUSH: /*land outer leg*/
		if(FO_on_ground()){ //outer feet have landed
			current_state = OUT_SWING;
		}
		break;
	case HOLD_DOUBLE: /*double stance*/ 
		current_state = HOLD_DOUBLE; //absorbing state, no exit transition
		break;
	default: /*state doesn't exist*/
		break;
	}
}

 
/* Returns the torque needed to compensate for gravity pull on the legs. */
float hip_gravity_compensation(void){
	float u = leg_m * g * leg_r;
	
	if(FI_on_ground() && !FO_on_ground()){
		//track outer, only inner feet on ground
		return u * Sin(th1); 
	}else if(!FI_on_ground() && FO_on_ground()){
		//track inner, only outer feet on ground 	
		return -u * Sin(th0);	
	}
	return 0.0;
}


/* Computes the controller set-points for tracking a RELATIVE angle in the hip.*/
void hip_track_rel(struct ControllerData * ctrlData, float qh_ref, float dqh_ref, float KP, float KD){
	ctrlData->xRef = qh_ref;
	ctrlData->vRef = dqh_ref;
	ctrlData->uRef = hip_gravity_compensation();

	ctrlData->kp = KP;
	ctrlData->kd = KD;
	return;
}


/* Make Ranger track a double stance. */
void hip_scissor_track(struct ControllerData * ctrlData, float c0, float c1, float KP, float KD){
	//use this equation: th_ref = c0*(-qr) + c1*(qh-qr)
	ctrlData->xRef = (th_ref + qr*(c0+c1)) / c1;
	ctrlData->vRef = dqr*(c0+c1)/c1;
	ctrlData->uRef = 0.0;
	//ctrlData->uRef = hip_gravity_compensation();

	ctrlData->kp = KP;
	ctrlData->kd = KD;
	return;
}


/* Computes the outer ankle controller set-points to track the desired absolute angle and rate. */
void out_ank_track_abs(struct ControllerData * ctrlData, float phi0_ref, float dphi0_ref, float u_ref, float KP, float KD){
	//convert absolute to relative
	//q0 = pi/2 - phi0 + th0
	//th0 = -qr
	ctrlData->xRef = PI/2 - phi0_ref - qr +0.15;
	if(th0 < 0){
		ctrlData->xRef += 0.3;	//outer leg in the back, add more offset
	}
	ctrlData->vRef = -dphi0_ref - dqr;
	ctrlData->uRef = u_ref;

	ctrlData->kp = KP;
	ctrlData->kd = KD;
	return;
}


/* Computes the inner ankle controller set-points to track the desired absolute angle and rate. */
void inn_ank_track_abs(struct ControllerData * ctrlData, float phi1_ref, float dphi1_ref, float u_ref, float KP, float KD){
	//convert absolute to relative
	//q1 = pi/2 - phi1 + th1 
	//th1 = qh - qr
	ctrlData->xRef = PI/2 - phi1_ref + qh - qr +0.15;
	if(th1 < 0){
		ctrlData->xRef += 0.3;	//inner leg in the back, add more offset
	}
	ctrlData->vRef = -dphi1_ref + dqh - dqr;
	ctrlData->uRef = u_ref;

	ctrlData->kp = KP;
	ctrlData->kd = KD;
	return;
}			   

/* this function is a PD controller with a smooth saturation
 * function wrapped around it.
 */
void smooth_saturate(struct ControllerData * ctrlData, float uMax, float q, float dq){
	float pdCtrlEffort = ctrlData->kp *(ctrlData->xRef - q) + ctrlData->kd * (ctrlData->vRef - dq);
	ctrlData->uRef += uMax*(2/PI)*Atan((PI/2)*pdCtrlEffort);
	mb_io_set_float(ID_CTRL_TEST_W0, uMax*(2/PI)*Atan((PI/2)*pdCtrlEffort));
	ctrlData->kp = 0; 
	ctrlData->kd = 0;
	return;
}

void test_foot(void){
	struct ControllerData ctrlAnkOut;
	struct ControllerData ctrlAnkInn;

	//update all the angle parameters
	angles_update();	 

	//hold inner & outer feet
	out_ank_track_abs(&ctrlAnkOut, ANK_REF_HOLD, 0.0, 0.0, ANK_FAST_KP, ANK_FAST_KD);
	inn_ank_track_abs(&ctrlAnkInn, ANK_REF_HOLD, 0.0, 0.0, ANK_FAST_KP, ANK_FAST_KD);
	
	smooth_saturate(&ctrlAnkOut, uMAX_ANK, q0, dq0);
	smooth_saturate(&ctrlAnkInn, uMAX_ANK, q1, dq1);

	//mb_io_set_float(ID_CTRL_TEST_W1,ctrlAnkInn.uRef);
	controller_ankleInner(&ctrlAnkInn);
	controller_ankleOuter(&ctrlAnkOut);

	return;
}
