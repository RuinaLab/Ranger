#include <mb_includes.h> 
#include <motorControl.h>
#include <fsm.h>
#include <rangermath.h>
#include <math.h> //for tanh

enum States {
	OUT_SWING,
	OUT_PUSH,
	INN_SWING,
	INN_PUSH,
	HOLD_DOUBLE,
	FLIGHT,
};

#define PI 3.141592653589793
#define ANK_FAST_KP 3//3.5//50
#define ANK_FAST_KD 0.5//0.8//3 
#define ANK_SLOW_KP 3//3.5//30
#define ANK_SLOW_KD 2.5//0.8//15 
/*#define HIP_KP 3 //6.9//40
#define HIP_KD 2.9 //3.1//10
*/
#define HIP_SCISSOR_GAIN 1.5
#define uMAX_ANK 4
#define uMAX_HIP 8  //2*uMAX_ANK

#define SCISSOR_OFFSET 0.1
#define	SCISSOR_RATE 1.3

static float th_ref = 0.0;
static enum States current_state = OUT_SWING; 
static float qr, qh, dqr, dqh, q0, q1, dq0, dq1; //relative
static float th0, th1, dth0, dth1; //absolute 

float ANK_FLIP_KP;
float ANK_FLIP_KD;
float ANK_PUSH_KP;
float ANK_PUSH_KD;
float ANK_HOLD_KP;
float ANK_HOLD_KD;
float HIP_KP;
float HIP_KD; 

float HIP_REF_HOLD;// 0.3 //relative angle for hip  17.2 degrees
float HIP_REF_TRANS_ANGLE; //0.12 relative angle
float ANK_REF_HOLD;// -0.05 //absolute angles for ankle
float ANK_REF_PUSH;// -0.8
float ANK_REF_FLIP;// 1.5


/* Updates the global absolute/relative angles in Ranger. */
void angles_update(void){
	 // set relative angles/angular rates
	qr = get_out_angle();	//angle integrated from rate gyro	
	qh = get_in_angle();	//hip angle
	dqr = get_out_ang_rate();
	dqh = get_in_ang_rate();
	q0 = mb_io_get_float(ID_E_MCFO_RIGHT_ANKLE_ANGLE);
	q1 = mb_io_get_float(ID_E_MCFI_MID_ANKLE_ANGLE);
	dq0 = mb_io_get_float(ID_E_MCFO_RIGHT_ANKLE_RATE);
	dq1 = mb_io_get_float(ID_E_MCFI_ANKLE_RATE);

	// set absolute angles
	th0 = -qr;		//angle of outer leg
	th1 = qh - qr;	//angle of inner leg 
	dth0 = -dqr; 
	dth1 = dqh - dqr;
}

/* Read values for parameters from Labview. */
void param_update(void){
	HIP_REF_HOLD = mb_io_get_float(ID_CTRL_HIP_REF_HOLD);
	HIP_REF_TRANS_ANGLE = mb_io_get_float(ID_CTRL_HIP_TRANS_ANGLE);
	ANK_REF_HOLD = mb_io_get_float(ID_CTRL_ANK_REF_HOLD) - 0.25;
	ANK_REF_PUSH = mb_io_get_float(ID_CTRL_ANK_REF_PUSH) - 0.25;
	ANK_REF_FLIP = mb_io_get_float(ID_CTRL_ANK_REF_FLIP) - 0.25;
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
	
	 ANK_FLIP_KP = mb_io_get_float(ID_CTRL_ANK_FLIP_KP);
	 ANK_FLIP_KD = mb_io_get_float(ID_CTRL_ANK_FLIP_KD);
	 ANK_PUSH_KP = mb_io_get_float(ID_CTRL_ANK_PUSH_KP);
	 ANK_PUSH_KD = mb_io_get_float(ID_CTRL_ANK_PUSH_KD);
	 ANK_HOLD_KP =  mb_io_get_float(ID_CTRL_ANK_HOLD_KD);
	 ANK_HOLD_KD =  mb_io_get_float(ID_CTRL_ANK_HOLD_KD);
	 HIP_KP = mb_io_get_float(ID_CTRL_HIP_KP);
	 HIP_KD= mb_io_get_float(ID_CTRL_HIP_KD); 


	angles_update();
	fsm_update();

	switch (current_state){
	case OUT_SWING:	/*swing inner leg*/
		mb_io_set_float(ID_CTRL_TEST_W1, 0);
		// hold outer feet
		out_ank_track_abs(&ctrlAnkOut, ANK_REF_HOLD, 0.0, 0.0, ANK_HOLD_KP, ANK_HOLD_KD);
		// flip up inner feet
		inn_ank_track_abs(&ctrlAnkInn, ANK_REF_FLIP, 0.0, 0.0, ANK_FLIP_KP, ANK_FLIP_KD);
		// adjust hip angle, outer on ground
		hip_scissor_track_outer(&ctrlHip, SCISSOR_OFFSET, SCISSOR_RATE,HIP_KP, HIP_KD);
		break;

	case OUT_PUSH:	/*land inner leg*/
		mb_io_set_float(ID_CTRL_TEST_W1, 1);
		// push down outer feet
	    out_ank_track_abs(&ctrlAnkOut, ANK_REF_PUSH, 0.0, 0.0, ANK_PUSH_KP, ANK_PUSH_KD);
		// hold inner feet
	 	inn_ank_track_abs(&ctrlAnkInn, ANK_REF_HOLD, 0.0, 0.0, ANK_FLIP_KP, ANK_FLIP_KD);
		// adjust hip angle	
		hip_track_rel(&ctrlHip, HIP_REF_HOLD, 0.0, HIP_KP, HIP_KD);
		break;

	case INN_SWING:	/*swing outer leg*/
		mb_io_set_float(ID_CTRL_TEST_W1, 2);
		// flip up outer feet 
		out_ank_track_abs(&ctrlAnkOut, ANK_REF_FLIP, 0.0, 0.0, ANK_FLIP_KP, ANK_FLIP_KD);
		// hold inner feet
		inn_ank_track_abs(&ctrlAnkInn, ANK_REF_HOLD, 0.0, 0.0, ANK_HOLD_KP, ANK_HOLD_KD);
		// adjust hip angle
		hip_scissor_track_inner(&ctrlHip, SCISSOR_OFFSET, SCISSOR_RATE,HIP_KP, HIP_KD);
		break;

	case INN_PUSH:	/*land outer leg*/
		mb_io_set_float(ID_CTRL_TEST_W1, 3);
		// hold outer feet
		out_ank_track_abs(&ctrlAnkOut, ANK_REF_HOLD, 0.0, 0.0, ANK_FLIP_KP, ANK_FLIP_KD);
		// push down inner feet
		inn_ank_track_abs(&ctrlAnkInn, ANK_REF_PUSH, 0.0, 0.0, ANK_PUSH_KP, ANK_PUSH_KD);
		// adjust hip angle
	    hip_track_rel(&ctrlHip, -HIP_REF_HOLD, 0.0, HIP_KP, HIP_KD);   
		break;

	case HOLD_DOUBLE: /*double stance*/
		mb_io_set_float(ID_CTRL_TEST_W1, 4);
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
	case FLIGHT: 
		mb_io_set_float(ID_CTRL_TEST_W1, 5);
		// flip up outer feet
		out_ank_track_abs(&ctrlAnkOut, ANK_REF_HOLD, 0.0, 0.0, ANK_FLIP_KP, ANK_FLIP_KD);
		// flip up inner feet
		inn_ank_track_abs(&ctrlAnkOut, ANK_REF_HOLD, 0.0, 0.0, ANK_FLIP_KP, ANK_FLIP_KD);
	default: /*state doesn't exist*/
		break;
	}

	// run the PD controllers 
	controller_hip(&ctrlHip);
	controller_ankleInner(&ctrlAnkInn);
	controller_ankleOuter(&ctrlAnkOut);
}


/* Updates current state of the FSM. */
void fsm_update(void){
	float trans_angle = 0.1;//0.5*HIP_REF_HOLD - 0.05;

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
	case FLIGHT:
		if(FI_on_ground()){
			current_state = INN_SWING;
		}else if(FO_on_ground()){
			current_state = OUT_SWING;
		}
		break;
	default: /*state doesn't exist*/
		break;
	}

	if(!FI_on_ground() && !FO_on_ground()){
		current_state = FLIGHT;
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
	//ctrlData->uRef = hip_gravity_compensation();
	ctrlData->uRef = 0.0;

	ctrlData->kp = KP;
	ctrlData->kd = KD;
	return;
}


/* Make Ranger track a double stance. */
void hip_scissor_track(struct ControllerData * ctrlData, float c0, float c1, float KP, float KD){
	//use this equation: th_ref = c0*(-qr) + c1*(qh-qr)
	ctrlData->xRef = (th_ref + qr*(c0+c1)) / c1;
	ctrlData->vRef = dqr*(c0+c1)/c1;
	//ctrlData->uRef = 0.0;
	ctrlData->uRef = hip_gravity_compensation();

	ctrlData->kp = KP;
	ctrlData->kd = KD;
	return;
}

void hip_scissor_track_outer(struct ControllerData * ctrlData, float offset, float rate, float KP, float KD){
	float th1Ref = offset - rate * th0;
	float dth1Ref = -rate*dth0; 
	
	ctrlData->xRef = th1Ref - th0;
	ctrlData->vRef = dth1Ref - dth0; 
	//ctrlData->uRef = 0.0;
	ctrlData->uRef = hip_gravity_compensation();
	ctrlData->kp = KP;
	ctrlData->kd = KD;
}

void hip_scissor_track_inner(struct ControllerData * ctrlData, float offset, float rate, float KP, float KD){
	float th0Ref = offset - rate * th1;
	float dth0Ref = -rate*dth1; 
	
	ctrlData->xRef = th1 - th0Ref;
	ctrlData->vRef = dth1 - dth0Ref;
	//ctrlData->uRef = 0.0;
	ctrlData->uRef = hip_gravity_compensation();
 	ctrlData->kp = KP;
	ctrlData->kd = KD;
}

#define ZERO_POS_INN 1.75
#define ZERO_POS_OUT 1.8

/* Computes the outer ankle controller set-points to track the desired absolute angle and rate. */
void out_ank_track_abs(struct ControllerData * ctrlData, float phi0_ref, float dphi0_ref, float u_ref, float KP, float KD){
	//convert absolute to relative
	//q0 = pi/2 - phi0 + th0
	//th0 = -qr
	ctrlData->xRef = ZERO_POS_OUT - phi0_ref - qr; //+ 0.25;//+0.15;
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
	ctrlData->xRef = ZERO_POS_INN - phi1_ref + qh - qr; //+0.25;//+0.15;
	ctrlData->vRef = -dphi1_ref + dqh - dqr;
	ctrlData->uRef = u_ref;

	ctrlData->kp = KP;
	ctrlData->kd = KD;
	return;
}			   


enum testStates {
	one,
	two,
	three,
	four,
	five,
	six,
};

static enum testStates test_state = one; 
int count = 0;
int first_step = 1;

void test_init(void){
	test_state = one;
	first_step = 1;
	count = 0;
}

void test_fsm_hip(void){
	struct ControllerData ctrlHip;
	struct ControllerData ctrlAnkOut;
	struct ControllerData ctrlAnkInn;
		
/*	float PUSH_KP = 7;
	float PUSH_KD = 2;
	float HOLD_KP = 4;
	float HOLD_KD = 0.5;
	float FLIP_KP =	3;
	float FLIP_KD = 0.5; */

	ANK_FLIP_KP = mb_io_get_float(ID_CTRL_ANK_FLIP_KP);
	ANK_FLIP_KD = mb_io_get_float(ID_CTRL_ANK_FLIP_KD);
	ANK_PUSH_KP = mb_io_get_float(ID_CTRL_ANK_PUSH_KP);
	ANK_PUSH_KD = mb_io_get_float(ID_CTRL_ANK_PUSH_KD);
	ANK_HOLD_KP =  mb_io_get_float(ID_CTRL_ANK_HOLD_KP);
	ANK_HOLD_KD =  mb_io_get_float(ID_CTRL_ANK_HOLD_KD);
	HIP_KP = mb_io_get_float(ID_CTRL_HIP_KP);
	HIP_KD= mb_io_get_float(ID_CTRL_HIP_KD);

	angles_update();

	switch(test_state){
	case one:  //swing innner leg 
		mb_io_set_float(ID_CTRL_TEST_W1, 10);
		// hold outer feet
		out_ank_track_abs(&ctrlAnkOut, ANK_REF_HOLD, 0.0, 0.0, ANK_HOLD_KP, ANK_HOLD_KD);
		// flip up inner feet
		inn_ank_track_abs(&ctrlAnkInn, ANK_REF_FLIP, 0.0, 0.0, ANK_FLIP_KP, ANK_FLIP_KD);
		
		/*if(first_step){
			// adjust hip
			hip_scissor_track_outer(&ctrlHip, 0, 1.5, HIP_KP, HIP_KD); //high KP and KD for hip here
	
			if(th0<-0.15){ //inner leg in front, outer leg in the back 
				test_state = two;
				first_step = 0;
			}
		}else{ */
			// adjust hip
			hip_scissor_track_outer(&ctrlHip, SCISSOR_OFFSET, SCISSOR_RATE, HIP_KP, HIP_KD); //high KP and KD for hip here
	
			if(th0<-HIP_REF_TRANS_ANGLE){ //inner leg in front, outer leg in the back 
				test_state = two;
			}
		/*}*/
		break;
	case two: //push off outer feet 
		mb_io_set_float(ID_CTRL_TEST_W1, 30);
		// push down outer feet
	    out_ank_track_abs(&ctrlAnkOut, ANK_REF_PUSH, 0.0, 0.0, ANK_PUSH_KP, ANK_PUSH_KD);
		// hold inner feet
	 	inn_ank_track_abs(&ctrlAnkInn, ANK_REF_HOLD, 0.0, 0.0, ANK_HOLD_KP, ANK_HOLD_KD);
		// decrease the angle between two legs
	    hip_track_rel(&ctrlHip, HIP_REF_HOLD, 0.0, HIP_KP, HIP_KD);
		
		if(q0>1.9 ){ //outer ankle angle
			test_state = three;
		}		
		break;
	case three: //swing outer leg
		mb_io_set_float(ID_CTRL_TEST_W1, 40);
		// flip up outer feet 
		out_ank_track_abs(&ctrlAnkOut, ANK_REF_FLIP, 0.0, 0.0, ANK_FLIP_KP, ANK_FLIP_KD);
		// hold inner feet
		inn_ank_track_abs(&ctrlAnkInn, ANK_REF_HOLD, 0.0, 0.0, ANK_HOLD_KP, ANK_HOLD_KD);
		// adjust hip angle
		hip_scissor_track_inner(&ctrlHip, SCISSOR_OFFSET, SCISSOR_RATE, HIP_KP, HIP_KD);
		
		if(th0>HIP_REF_TRANS_ANGLE-0.01){ //outer leg in front, inner leg in the back 
			test_state = four;	
		}
		break;
	case four:
		mb_io_set_float(ID_CTRL_TEST_W1, 60);
		//push down inner feet and hold outer feet
		out_ank_track_abs(&ctrlAnkOut, ANK_REF_HOLD, 0.0, 0.0, ANK_HOLD_KP, ANK_HOLD_KD);
		inn_ank_track_abs(&ctrlAnkInn, ANK_REF_PUSH, 0.0, 0.0, ANK_PUSH_KP, ANK_PUSH_KD);
		hip_track_rel(&ctrlHip, -HIP_REF_HOLD, 0.0, HIP_KP, HIP_KD);

		if(q1>1.9 ){ //inner ankle angle
			test_state = one;
		}	
		break;
	}

	//controller_hip(&ctrlHip);
	controller_ankleInner(&ctrlAnkInn);
	controller_ankleOuter(&ctrlAnkOut);
}

void test_fsm(void){
	struct ControllerData ctrlAnkOut;
	struct ControllerData ctrlAnkInn;
	float trans_angle = 0.08;//0.5*HIP_REF_HOLD-0.3;

	ANK_FLIP_KP = mb_io_get_float(ID_CTRL_ANK_FLIP_KP);
	ANK_FLIP_KD = mb_io_get_float(ID_CTRL_ANK_FLIP_KD);
	ANK_PUSH_KP = mb_io_get_float(ID_CTRL_ANK_PUSH_KP);
	ANK_PUSH_KD = mb_io_get_float(ID_CTRL_ANK_PUSH_KD);
	ANK_HOLD_KP =  mb_io_get_float(ID_CTRL_ANK_HOLD_KP);
	ANK_HOLD_KD =  mb_io_get_float(ID_CTRL_ANK_HOLD_KD);
	HIP_KP = mb_io_get_float(ID_CTRL_HIP_KP);
	HIP_KD= mb_io_get_float(ID_CTRL_HIP_KD);

	angles_update();

	switch(test_state){
	case one: //OUT_SWING
		mb_io_set_float(ID_CTRL_TEST_W1, 1);
		// hold outer feet
		out_ank_track_abs(&ctrlAnkOut, ANK_REF_HOLD, 0.0, 0.0, ANK_HOLD_KP, ANK_HOLD_KD);
		// flip up inner feet
		inn_ank_track_abs(&ctrlAnkInn, ANK_REF_FLIP, 0.0, 0.0, ANK_FLIP_KP, ANK_FLIP_KD);

		if( th0<-trans_angle){ //inner leg in front, outer leg in the back 
			test_state = two;	
		}
		break;
	case two: //OUT_PUSH		
		mb_io_set_float(ID_CTRL_TEST_W1, 2);		
		// push down outer feet
	    out_ank_track_abs(&ctrlAnkOut, ANK_REF_PUSH, 0.0, 0.0, ANK_PUSH_KP, ANK_PUSH_KD);
		// hold inner feet
	 	inn_ank_track_abs(&ctrlAnkInn, ANK_REF_HOLD, 0.0, 0.0, ANK_FLIP_KP, ANK_FLIP_KD);
		if(q0 > 2.2){ //inner feet have landed 
		 	test_state = three;
		}
		break;
	case three: //INN_SWING
		mb_io_set_float(ID_CTRL_TEST_W1, 3);		
		// flip up outer feet 
		out_ank_track_abs(&ctrlAnkOut, ANK_REF_FLIP, 0.0, 0.0, ANK_FLIP_KP, ANK_FLIP_KD);
		// hold inner feet
		inn_ank_track_abs(&ctrlAnkInn, ANK_REF_HOLD, 0.0, 0.0, ANK_HOLD_KP, ANK_HOLD_KD);
		if(th0>trans_angle){ //inner leg in back, outer leg in front
			test_state = four;
		}
		break; 
	case four: //INN_PUSH
		mb_io_set_float(ID_CTRL_TEST_W1, 4);
		// hold outer feet
		out_ank_track_abs(&ctrlAnkOut, ANK_REF_HOLD, 0.0, 0.0, ANK_FLIP_KP, ANK_FLIP_KD);
		// push down inner feet
		inn_ank_track_abs(&ctrlAnkInn, ANK_REF_PUSH, 0.0, 0.0, ANK_PUSH_KP, ANK_PUSH_KD);
		if(q1 > 2.2){
			test_state = one;
		}
		break;
	}

	controller_ankleInner(&ctrlAnkInn);
	controller_ankleOuter(&ctrlAnkOut);
}

