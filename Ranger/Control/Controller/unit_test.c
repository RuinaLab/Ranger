#include <mb_includes.h> 
#include <motorControl.h>
#include <fsm.h>
#include <unit_test.h>
#include "Trajectory.h"
#include "TrajData.h"
#include "RangerMath.h"	//for Sin()

#define PI 3.141592653589793				   
#define DATA TRAJ_DATA_Test0

static float hip_kp, hip_kd, scissor_offset, scissor_rate, ank_kp, ank_kd, ank_hold, ank_push, ank_flip;

/* Updates parameters read from LABVIEW. */
void param_update_test(void){
	hip_kp = mb_io_get_float(ID_CTRL_HIP_KP);	  //16, 27
	hip_kd = mb_io_get_float(ID_CTRL_HIP_KD);	  //3, 3.8
	scissor_offset = 0.1;
	scissor_rate = 1.3;
	
	ank_kp = 7;
	ank_kd = 1;
	ank_hold = mb_io_get_float(ID_CTRL_ANK_REF_HOLD); 
	ank_push = mb_io_get_float(ID_CTRL_ANK_REF_PUSH);
	ank_flip = mb_io_get_float(ID_CTRL_ANK_REF_FLIP); 
}


/* Tests gravity compensation with PD controller turned off (one leg on ground, one leg in air).
 * Parameters:
 *   - ID_CTRL_TEST_R0 = reference hip angle
 * Expected behavior: 
 * 	 - gravity compensation should be able hold the legs at the reference angle 
 *	 -(may need to manually move the leg to the reference angle first due to friction)	
 */
void test_gravity_compensation(void){
	struct ControllerData ctrlHip;
	struct ControllerData ctrlAnkOut;
	struct ControllerData ctrlAnkInn;
	float on_ground = 0; // 0=outer leg on ground, 1=inner leg on ground

	float th1_test = mb_io_get_float(ID_CTRL_TEST_R0);
	float th0_test = mb_io_get_float(ID_CTRL_TEST_R0);

	float leg_m = 2.5;
	float leg_r = 0.15;
	float g = 9.8;
	float u = leg_m * g * leg_r;

   	//calculate gravity compensation 
	if(FI_on_ground() && !FO_on_ground()){
		//only inner feet on ground
		u = -u * Sin(th0_test); 
	}else if(!FI_on_ground() && FO_on_ground()){
		//only outer feet on ground 	
		u =  u * Sin(th1_test);	
	}else{
		u = 0;
	}

	angles_update();
	param_update_test();
	
	//sets correct ankle angles 	
	if(on_ground == 0){
		out_ank_track_abs(&ctrlAnkOut, ank_hold, 0.0, 0.0, ank_kp, ank_kd);
		inn_ank_track_abs(&ctrlAnkInn, ank_flip, 0.0, 0.0, ank_kp, ank_kd);
	}else{
		out_ank_track_abs(&ctrlAnkOut, ank_flip, 0.0, 0.0, ank_kp, ank_kd);
		inn_ank_track_abs(&ctrlAnkInn, ank_hold, 0.0, 0.0, ank_kp, ank_kd);	
	}

	//turns off pd controller, only include gravity compensation
	ctrlHip.kp = 0.0;
	ctrlHip.kd = 0.0;
	ctrlHip.xRef = 0.0;
	ctrlHip.vRef = 0.0;
	ctrlHip.uRef = u;	

	//run controllers
	controller_ankleInner(&ctrlAnkInn);
	controller_ankleOuter(&ctrlAnkOut);
	controller_hip(&ctrlHip);
}

/* This function is used for doing system ID for the ankle
 * motors. Assume that the robot is hanging in the air. 
 * This function sends current commands to the inner ankle motors.
 *
 */
void test_ankle_current_control(void){

	struct ControllerData ctrlHip;
	struct ControllerData ctrlAnkOut;
	struct ControllerData ctrlAnkInn;

	float iLow = -0.15;  // minimum ankle current
	float iUpp = 0.15;  // Maximum ankle current					   		
	
	float current;  // = mb_io_get_float(ID_CTRL_TEST_R0);  //  Nominal current
	float frequency = mb_io_get_float(ID_CTRL_TEST_R1);  //  Input frequency

	float time = 0.001 * mb_io_get_float(ID_TIMESTAMP);

	float period;  // period for reference function
	float arg;  // input for trig functions
	float xRef;  // reference ankle angle
	float vRef;  // reference angle rate
	float kp;   // proportional gaint
	float kd;   // derivative gain		

	arg = 2.0*PI*time*frequency;  
	current = 0.5*(iLow + iUpp) + 0.5*(iUpp-iLow)*Sin(arg);

	mb_io_set_float(ID_CTRL_TEST_W0,current);  // check that the sine function is working

	// Direct current control over the inner feet:
	mb_io_set_float(ID_MCFI_COMMAND_CURRENT, current);
	mb_io_set_float(ID_MCFI_STIFFNESS, 0.0);
	mb_io_set_float(ID_MCFI_DAMPNESS, 0.0);

	// Do nothing with the outer ankles or hip:

	mb_io_set_float(ID_MCFO_COMMAND_CURRENT, 0.0);
	mb_io_set_float(ID_MCFO_STIFFNESS, 0.0);
	mb_io_set_float(ID_MCFO_DAMPNESS, 0.0);

	mb_io_set_float(ID_MCH_COMMAND_CURRENT, 0.0);
	mb_io_set_float(ID_MCH_STIFFNESS, 0.0);
	mb_io_set_float(ID_MCH_DAMPNESS, 0.0);

}

/* Test hip controller with outer feet on ground and inner feet flipped up. 
 * Parameters:
 *   - ID_CTRL_TEST_W1 = reference hip angle
 */
void test_hip_outer(void){
	struct ControllerData ctrlHip;
	struct ControllerData ctrlAnkOut;
	struct ControllerData ctrlAnkInn;

	float hip_track = mb_io_get_float(ID_CTRL_TEST_R0);

	angles_update();
	param_update_test();
	
	out_ank_track_abs(&ctrlAnkOut, ank_hold, 0.0, 0.0, ank_kp, ank_kd);
	inn_ank_track_abs(&ctrlAnkInn, ank_flip, 0.0, 0.0, ank_kp, ank_kd);
	// tests hip scissor tracking function
	hip_scissor_track_outer(&ctrlHip, scissor_offset, scissor_rate, hip_kp, hip_kd); 
	// tests hip relative angle tracking function
	//mb_io_set_float(ID_CTRL_TEST_W1, hip_track);
	//hip_track_rel(&ctrlHip, hip_track, 0.0, hip_kp, hip_kd);	

	controller_hip(&ctrlHip);
	controller_ankleInner(&ctrlAnkInn);
	controller_ankleOuter(&ctrlAnkOut);
} 


/* Inner leg standing, swing outer leg */
void test_hip_inner(void){
	struct ControllerData ctrlHip;
	struct ControllerData ctrlAnkOut;
	struct ControllerData ctrlAnkInn;

	float hip_track = mb_io_get_float(ID_CTRL_TEST_R0);

	angles_update();
	param_update_test();
	
	out_ank_track_abs(&ctrlAnkOut, ank_flip, 0.0, 0.0, ank_kp, ank_kd);
	inn_ank_track_abs(&ctrlAnkInn, ank_hold, 0.0, 0.0, ank_kp, ank_kd);
	// tests inner hip scissor tracking function
	hip_scissor_track_inner(&ctrlHip, scissor_offset, scissor_rate, hip_kp, hip_kd); 
	//simple_scissor_track_inner(&ctrlHip, hip_kp, hip_kd);
	// tests hip relative angle tracking function
	//mb_io_set_float(ID_CTRL_TEST_W1, hip_track);
	//hip_track_rel(&ctrlHip, hip_track, 0.0, hip_kp, hip_kd);	

	controller_hip(&ctrlHip);
	controller_ankleInner(&ctrlAnkInn);
	controller_ankleOuter(&ctrlAnkOut);
}


/* Make the feet track an absolute angle while making the hip angle track a zero angle.
 * Parameters:
 *   - ID_CTRL_TEST_W1 = absolute outer ankle angle
 *	 - ID_CTRL_TEST_W2 = absolute inner ankle angle
 *	 - ID_CTRL_TEST_W3 = absolute reference ankle angle
 */
void test_feet(void){
	struct ControllerData ctrlHip;
	struct ControllerData ctrlAnkOut;
	struct ControllerData ctrlAnkInn;
	float ank_track = 0.0;
	
	float hip_track = 0.0;
	float ph0, ph1;

	//update all the angle parameters
	angles_update();
	param_update_test();	 

	//calls higher level function to set the struct fields
	out_ank_track_abs(&ctrlAnkOut, ank_hold, 0.0, 0.0, ank_kp, ank_kd);
	inn_ank_track_abs(&ctrlAnkInn, ank_hold, 0.0, 0.0, ank_kp, ank_kd);
	hip_track_rel(&ctrlHip, hip_track, 0.0, hip_kp, hip_kd);
	
	ph0 = ZERO_POS_OUT - qr -q0; 
	ph1 = ZERO_POS_INN + qh - qr - q1;	   
	//mb_io_set_float(ID_CTRL_TEST_W1, ph0); //absolute outer ankle angle
	//mb_io_set_float(ID_CTRL_TEST_W2, ph1); //absolute inner ankle angle
  	//mb_io_set_float(ID_CTRL_TEST_W3, 0); //absolute reference ankle angle 

	controller_ankleInner(&ctrlAnkInn);
	controller_ankleOuter(&ctrlAnkOut);
	//controller_hip(&ctrlHip);
}



/* Robot inner leg standing, swings outer leg
 * Hip motor tracks Sine-curve reference
 */
void hip_motor_test(void){
	struct ControllerData ctrlHip;
	struct ControllerData ctrlAnkOut;
	struct ControllerData ctrlAnkInn;
	float qUpp = 0.2;  // Maximum ankle angle
	float qLow = -0.2;  // minimum ankle angle					   

	float time = 0.001 * mb_io_get_float(ID_TIMESTAMP);

	float period;  // period for reference function
	float arg;  // input for trig functions
	float xRef;  // reference ankle angle
	float vRef;  // reference angle rate
	float kp;   // proportional gaint
	float kd;   // derivative gain		
	
	period = 2;

	arg = 2*PI*time/period;
	xRef = 0.5*(qLow + qUpp) + 0.5*(qUpp-qLow)*Sin(arg);

	vRef = (qUpp-qLow)*(PI/period)*Cos(arg);

	//mb_io_set_float(ID_CTRL_TEST_W7,xRef);
	//mb_io_set_float(ID_CTRL_TEST_W8,vRef);

	//kp=28, kd=4, give good tracking for the ankles 
	kp = 16; //mb_io_get_float(ID_CTRL_TEST_R0); 
	kd = 3; //mb_io_get_float(ID_CTRL_TEST_R1);

	ctrlHip.kp = kp;
	ctrlHip.kd = kd;
	ctrlHip.xRef = xRef;
	ctrlHip.vRef = vRef;
	//ctrlHip.uRef = 0.0;
	ctrlHip.uRef = hip_gravity_compensation();	//does not help much
	
	angles_update();
	param_update_test();
	out_ank_track_abs(&ctrlAnkOut, ank_flip, 0.0, 0.0, ank_kp, ank_kd);
	inn_ank_track_abs(&ctrlAnkInn, ank_hold, 0.0, 0.0, ank_kp, ank_kd);
	
	controller_ankleInner(&ctrlAnkInn);
	controller_ankleOuter(&ctrlAnkOut);
	controller_hip(&ctrlHip);
}

/* Simpler implementation of hip scissor tracking. */
void simple_scissor_track_inner(struct ControllerData * ctrlData, float KP, float KD){ 
	float th0Ref = -2 * th1;
	float dth0Ref = 0.0; //-2 * dth1; 
	
	ctrlData->xRef = th1 - th0Ref;
	ctrlData->vRef = 0.0; //dth1 - dth0Ref;
	ctrlData->uRef = 0.0;
	//ctrlData->uRef = hip_gravity_compensation();
 	ctrlData->kp = KP;
	ctrlData->kd = KD;	
}

/* Robot Hanging in the air
 * Hip motor off
 * Ankle motors track Sin(Sin(t)) reference (with useful paramters)
 * Parameters:
 *  // - ID_CTRL_TEST_R0 = kp
 *	// - ID_CTRL_TEST_R1 = kd
 *   - ID_CTRL_TEST_W7 = reference angle
 *	 - ID_CTRL_TEST_W8 = reference angular rate
 */
void ankle_motor_test(void){
	struct ControllerData ctrlHip;
	struct ControllerData ctrlAnkOut;
	struct ControllerData ctrlAnkInn;

	float qUpp = 2.4;  // Maximum ankle angle
	float qLow = 0.2;  // minimum ankle angle					   

	float time = 0.001 * mb_io_get_float(ID_TIMESTAMP);

	float frequency;  // period for reference function
	float arg;  // input for trig functions
	float xRef;  // reference ankle angle
	float vRef;  // reference angle rate
	float kp;   // proportional gaint
	float kd;   // derivative gain		

	frequency = mb_io_get_float(ID_CTRL_TEST_R2);

	arg = 2.0*PI*time*frequency;  
	xRef = 0.5*(qLow + qUpp) + 0.5*(qUpp-qLow)*Sin(arg);
	vRef = (qUpp-qLow)*(PI*frequency)*Cos(arg);

	mb_io_set_float(ID_CTRL_TEST_W0,xRef);
	mb_io_set_float(ID_CTRL_TEST_W1,vRef);

	//kp=7, kd=1, give good tracking for the ankles 
	kp = mb_io_get_float(ID_CTRL_TEST_R0); 
	kd = mb_io_get_float(ID_CTRL_TEST_R1);

	// Run a PD-controller on the outer foot angles:
	ctrlAnkOut.kp = kp;
	ctrlAnkOut.kd = kd;
	ctrlAnkOut.xRef = xRef;
	ctrlAnkOut.vRef = vRef;
	ctrlAnkOut.uRef = 0.0;
	controller_ankleOuter(&ctrlAnkOut);	

	// Run a PD-controller on the inner foot angles:
	ctrlAnkInn.kp = kp;
	ctrlAnkInn.kd = kd;
	ctrlAnkInn.xRef = xRef;
	ctrlAnkInn.vRef = vRef;
	ctrlAnkInn.uRef = 0.0;
	controller_ankleInner(&ctrlAnkInn);

	// Do nothing with the outer ankles or hip:

	mb_io_set_float(ID_MCFO_COMMAND_CURRENT, 0.0);
	mb_io_set_float(ID_MCFO_STIFFNESS, 0.0);
	mb_io_set_float(ID_MCFO_DAMPNESS, 0.0);

	mb_io_set_float(ID_MCH_COMMAND_CURRENT, 0.0);
	mb_io_set_float(ID_MCH_STIFFNESS, 0.0);
	mb_io_set_float(ID_MCH_DAMPNESS, 0.0);
}

/* Runs a simple test of the motor controllers, connecting the LabView parameter
 * to the set-points of the hip and ankle controllers.
 */
 void test_motor_control() {
 		struct ControllerData ctrlHip;
		struct ControllerData ctrlAnkOut;
		struct ControllerData ctrlAnkInn;

	// Run a PD-controller on the hip angle:
		ctrlHip.kp = mb_io_get_float(ID_CTRL_TEST_R0);
		ctrlHip.kd = mb_io_get_float(ID_CTRL_TEST_R1);
		ctrlHip.xRef = mb_io_get_float(ID_CTRL_TEST_R2);
		ctrlHip.vRef = mb_io_get_float(ID_CTRL_TEST_R3);
		ctrlHip.uRef = mb_io_get_float(ID_CTRL_TEST_R4);
		controller_hip(&ctrlHip);

	// Run a PD-controller on the outer foot angles:
		ctrlAnkOut.kp = mb_io_get_float(ID_CTRL_TEST_R5);
		ctrlAnkOut.kd = mb_io_get_float(ID_CTRL_TEST_R6);
		ctrlAnkOut.xRef = mb_io_get_float(ID_CTRL_TEST_R7);
		ctrlAnkOut.vRef = mb_io_get_float(ID_CTRL_TEST_R8);
		ctrlAnkOut.uRef = mb_io_get_float(ID_CTRL_TEST_R9);
		controller_ankleOuter(&ctrlAnkOut);

	// Run a PD-controller on the inner foot angles:
		ctrlAnkInn.kp = mb_io_get_float(ID_CTRL_TEST_R5);
		ctrlAnkInn.kd = mb_io_get_float(ID_CTRL_TEST_R6);
		ctrlAnkInn.xRef = mb_io_get_float(ID_CTRL_TEST_R7);
		ctrlAnkInn.vRef = mb_io_get_float(ID_CTRL_TEST_R8);
		ctrlAnkInn.uRef = mb_io_get_float(ID_CTRL_TEST_R9);
		controller_ankleInner(&ctrlAnkInn);
 }

/* Turns all motors off at high level by setting KP=0 and KD=0 */
void motors_off(void){
	struct ControllerData ctrlHip;
	struct ControllerData ctrlAnkOut;
	struct ControllerData ctrlAnkInn;

	// Run a PD-controller on the hip angle:
		ctrlHip.kp = 0.0;
		ctrlHip.kd = 0.0;
		ctrlHip.xRef = 0.0;
		ctrlHip.vRef = 0.0;
		ctrlHip.uRef = 0.0;
		controller_hip(&ctrlHip);
	
	// Run a PD-controller on the outer foot angles:
		ctrlAnkOut.kp = 0.0;
		ctrlAnkOut.kd = 0.0;
		ctrlAnkOut.xRef = 0.0;
		ctrlAnkOut.vRef = 0.0;
		ctrlAnkOut.uRef = 0.0;
		controller_ankleOuter(&ctrlAnkOut);
	
	// Run a PD-controller on the inner foot angles:
		ctrlAnkInn.kp = 0.0;
		ctrlAnkInn.kd = 0.0;
		ctrlAnkInn.xRef = 0.0;
		ctrlAnkInn.vRef = 0.0;
		ctrlAnkInn.uRef = 0.0;
		controller_ankleInner(&ctrlAnkInn);
}

/* Helps figure out sign convention for the motors. */
 void test_sign(void){
		struct ControllerData ctrlHip;
		struct ControllerData ctrlAnkOut;
		struct ControllerData ctrlAnkInn;

	// Run a PD-controller on the hip angle:
		ctrlHip.kp = 0;
		ctrlHip.kd = 0;
		ctrlHip.xRef = 0;
		ctrlHip.vRef = 0;
		ctrlHip.uRef = mb_io_get_float(ID_CTRL_TEST_R0);
		controller_hip(&ctrlHip);

	// Run a PD-controller on the outer foot angles:
		ctrlAnkOut.kp = 0;
		ctrlAnkOut.kd = 0;
		ctrlAnkOut.xRef = 0;
		ctrlAnkOut.vRef = 0;
		ctrlAnkOut.uRef = mb_io_get_float(ID_CTRL_TEST_R1);
		controller_ankleOuter(&ctrlAnkOut);

	// Run a PD-controller on the inner foot angles:
		ctrlAnkInn.kp = 0;
		ctrlAnkInn.kd = 0;
		ctrlAnkInn.xRef = 0;
		ctrlAnkInn.vRef = 0;
		ctrlAnkInn.uRef = mb_io_get_float(ID_CTRL_TEST_R2);
		controller_ankleInner(&ctrlAnkInn); 
 }


 static int counter = 0;  
 /* Makes the inner foot periodically tracks hold/push/flip. */
 void test_inner_foot(void) { 
		struct ControllerData ctrlAnkInn;

	// Set up the angle reference, so that inner foot simulates when it's walking		
		if(counter <= 2000){
			ctrlAnkInn.xRef = param_joint_ankle_hold;
		}else if (counter <= 4000){
			ctrlAnkInn.xRef = param_joint_ankle_push;
		}else if (counter <= 6000){
			ctrlAnkInn.xRef = param_joint_ankle_flip;
		}else{
			counter = 0;
		}		
		counter ++;

	// Run a PD-controller on the inner foot angles:
		ctrlAnkInn.kp = 3; 
 		ctrlAnkInn.kd = 0.5;
		ctrlAnkInn.vRef = 0;
		ctrlAnkInn.uRef = 0;
		controller_ankleInner(&ctrlAnkInn);
 }

 /* Runs a simple test of the frequency controllers */
 void test_freq_control(void) {
 		struct ControllerData ctrlHip;
		struct ControllerData ctrlAnkOut;
		struct ControllerData ctrlAnkInn;

		
	/*	float hipRefAmp;  // square wave reference amplitude
		float hip_xi;   // controller damping ratio
		float hip_wn;   // controller natural frequency

		hipRefAmp = mb_io_get_float(ID_CTRL_TEST_R0);
 		hip_xi = mb_io_get_float(ID_CTRL_TEST_R1);
 		hip_wn = mb_io_get_float(ID_CTRL_TEST_R2);


	// Run a PD-controller on the hip angle:
	// The outer foot should trace a step-function, swing periodically between +/- hipRefAmp
		ctrlHip.wn = hip_wn;
		ctrlHip.xi = hip_xi;
		if(counter <= 1000){
			ctrlHip.xRef = hipRefAmp;
		}else if (counter <= 2000){
			ctrlHip.xRef = -hipRefAmp;
		}else{
			counter = 0;
		}
		
		ctrlHip.vRef = 0;
		ctrlHip.uRef = 0;

		counter ++;	
		controller_hip(&ctrlHip);

	    mb_io_set_float(ID_CTRL_TEST_W0, ctrlHip.Cp);
		mb_io_set_float(ID_CTRL_TEST_W1, ctrlHip.Cd);
		mb_io_set_float(ID_CTRL_TEST_W2, ctrlHip.kp);
		mb_io_set_float(ID_CTRL_TEST_W3, ctrlHip.kd);
	*/
	
	// Set up the angle reference, so that outer&inner foot trace a step-function
	// (swing periodically between 0.5 and 1.5 rad)  			
		if(counter <= 1000){
			ctrlAnkInn.xRef = 1.5;
			ctrlAnkOut.xRef = 1.5;
		}else if (counter <= 2000){
			ctrlAnkInn.xRef = 0.5;
			ctrlAnkOut.xRef = 0.5;
		}else{
			counter = 0;
		}		
		counter ++;
					
	// Run a PD-controller on the outer foot angles:
		ctrlAnkOut.kp = 3;
		ctrlAnkOut.kd = 0.5;
		ctrlAnkOut.vRef = 0;
		ctrlAnkOut.uRef = 0;
		controller_ankleOuter(&ctrlAnkOut);

	// Run a PD-controller on the inner foot angles:
		ctrlAnkInn.kp = 3;
		ctrlAnkInn.kd = 0.5;
		ctrlAnkInn.vRef = 0;
		ctrlAnkInn.uRef = 0;
		controller_ankleInner(&ctrlAnkInn);
		
 }

 /* Runs a simple test that makes the inner leg traces a Sin wave 
  *	while making both inner and outer feet stay flat
  */
 void test_trajectory(void) {
 		struct ControllerData ctrlHip;
		struct ControllerData ctrlAnkOut;
		struct ControllerData ctrlAnkInn;
		float in_angle, out_angle;
		float torque;

	// Compute the position/slope/curvature of a 5th order polynomial at a given time 
		int row = sizeof(DATA)/sizeof(DATA[0][0])/4;
		float max_t = DATA[row-1][0];
		poly_coeff *COEFFS = data_to_coeff(DATA, row);
		
		float sys_t = mb_io_get_float(ID_TIMESTAMP)/1000;//converts the system_time from ms to s
		float phi;
		poly_coeff c;
		float y, yd, ydd;
		float t = Fmod(sys_t, max_t);
		int index = getIndex(t, COEFFS, row-1); //length of the the COEFFS array is row-1
		if(index == -1){
			//input time less than the time interval
			index = 0;
			c = COEFFS[index];
			phi = 0;
		}else if(index == -2){
			//input time greater than the time interval
			index = row-2; 
			c = COEFFS[index];
			phi = 1;
		}else{
			//found the time interval, evaluate the polynomial at time t
			c = COEFFS[index];
			phi = (t-c.t0)/(c.t1-c.t0);
		}
		
		y = getY(c, phi);
		yd = getYd(c, phi);
		ydd = getYdd(c, phi);	
	
	// Calculate the toque needed to compensate for gravity pull
		in_angle = mb_io_get_float(ID_MCH_ANGLE);  // gets the hip angle (angle b/t inner&outer legs; pos when inner leg is in front)
		torque = leg_m * g * leg_r * Sin(in_angle); 

	// Run a PD-controller on the hip angle:
		ctrlHip.kp = 6;
		ctrlHip.kd = 2;
		
		ctrlHip.xRef = y;
		//mb_io_set_float(ID_CTRL_TEST_W0, ctrlHip.xRef);
		ctrlHip.vRef = yd;
		ctrlHip.uRef = torque;
		controller_hip(&ctrlHip);

	// Run a PD-controller on the outer foot angles: make the feet stay flat wrt the ground
		ctrlAnkOut.kp = 3;
		ctrlAnkOut.kd = 0.5;
		ctrlAnkOut.xRef = FO_flat_angle();
		ctrlAnkOut.vRef = 0.0;
		ctrlAnkOut.uRef = 0.0;
		controller_ankleOuter(&ctrlAnkOut);

	
	// Run a PD-controller on the inner foot angles: make the feet stay flat wrt the ground
		ctrlAnkInn.kp = 3;
		ctrlAnkInn.kd = 0.5;
		ctrlAnkInn.xRef = FI_flat_angle(); 	
		ctrlAnkInn.vRef = 0.0;
		ctrlAnkInn.uRef = 0.0;
		controller_ankleInner(&ctrlAnkInn);
 }

 /* A test that makes the inner leg track a Sin wave generated uSing Sin function from RangerMath. */ 
 void track_Sin(void){
 	 	struct ControllerData ctrlHip;
		float sys_t = mb_io_get_float(ID_TIMESTAMP)/1000;//converts the system_time from ms to s
		float t = Fmod(sys_t, 2*PI);

	// Run a PD-controller on the hip angle:
		ctrlHip.kp = 6;
		ctrlHip.kd = 2;
				
		if(t > PI){
		 	t = -2*PI + t;
		}
		ctrlHip.xRef = Sin(t)/4;
		mb_io_set_float(ID_CTRL_TEST_W0, ctrlHip.xRef);
		ctrlHip.vRef = Cos(t)/4;
		ctrlHip.uRef = 0.0;
		controller_hip(&ctrlHip);
 }

 /* Make sthe robot stand in double stance, the hip motor adjusts the inner leg according to the absolute angle
  * calculated from gyro rate. 
  * Inner feet always flipped all the way up; outer feet stay flat relative to the ground. 
  */
 void double_stance(void){
 		struct ControllerData ctrlHip;
		//struct ControllerData ctrlAnkOut;
		//struct ControllerData ctrlAnkInn;
		float out_angle, in_angle, torque; 			 

		out_angle = get_out_angle();
	
	// Run a PD-controller on the hip angle:
		ctrlHip.kp = 6.9;
		ctrlHip.kd = 3.1;
		/*ctrlHip.wn = 3.5;
		ctrlHip.xi = 0.8;
		*/
		ctrlHip.xRef = 2*out_angle;
		ctrlHip.vRef = 0.0;
		
	// Calculate the toque needed to compensate for gravity pull
		//in_angle = get_in_angle();  // gets the hip angle (angle b/t inner&outer legs; pos when inner leg is in front)
		//torque = leg_m * g * leg_r * Sin(in_angle);
		//ctrlHip.uRef = torque; // adding this torque makes the motor shuts off more frequently
		controller_hip(&ctrlHip);

	// Call function implemented in fsm.c to set the feet flat on ground --> works!
		//test_foot();

	// Run a PD-controller on the outer foot angles: make the feet stay flat wrt the ground
		//ctrlAnkOut.wn = 7;
		//ctrlAnkOut.xi = 0.8;
		/*ctrlAnkOut.kp = 3.5;
		ctrlAnkOut.kd = 0.8;  
		ctrlAnkOut.xRef = FO_flat_angle();
		ctrlAnkOut.vRef = 0.0;
		ctrlAnkOut.uRef = 0.0;
		controller_ankleOuter(&ctrlAnkOut);
		 */
	
	// Run a PD-controller on the inner foot angles: make the inner feet always flip up
		//ctrlAnkInn.wn = 7;
		//ctrlAnkInn.xi = 0.8;
 		/*ctrlAnkInn.kp =	3.5;
		ctrlAnkInn.kd = 0.8;
		ctrlAnkInn.xRef = FI_flat_angle();
		ctrlAnkInn.vRef = 0.0;
		ctrlAnkInn.uRef = 0.0;
		controller_ankleInner(&ctrlAnkInn);
		*/
 } 

 /* Flip all feet up. */
 void foot_flip(void){
 		struct ControllerData ctrlAnkOut;
		struct ControllerData ctrlAnkInn;

	// Flip outer feet
		ctrlAnkOut.kp = 3;
		ctrlAnkOut.kd = 0.5;
		ctrlAnkOut.xRef = param_joint_ankle_flip;
		ctrlAnkOut.vRef = 0.0;
		ctrlAnkOut.uRef = 0.0;
		controller_ankleOuter(&ctrlAnkOut);
	// Flip inner feet
		ctrlAnkInn.kp = 3;
		ctrlAnkInn.kd = 0.5;
		ctrlAnkInn.xRef = param_joint_ankle_flip; 	
		ctrlAnkInn.vRef = 0.0;
		ctrlAnkInn.uRef = 0.0;
		controller_ankleInner(&ctrlAnkInn);
 }

 /* Returns the relative angle that makes the outer foot stay flat wrt ground. */
 float FO_flat_angle(void){
  		float out_angle, FO_angle;
		out_angle = get_out_angle();  // gets the absolute angle (of the outer leg) wrt ground; pos when outer leg is forward  
		
		FO_angle = (PI/2 - out_angle + 0.2); //adds an offset of 0.15 rad to make the feet more stable 

		if(out_angle > 0){
			//outer leg is in back, add offset to outer ankle angle
			FO_angle += 0.4;  
		}

		return FO_angle;
 }


 /* Returns the relative angle that makes the inner foot stay flat wrt ground. */
 float FI_flat_angle(void){
 		float in_angle, out_angle, FI_angle;
		out_angle = get_out_angle();  // gets the absolute angle (of the outer leg) wrt ground; pos when outer leg is forward  
		in_angle = get_in_angle();

		FI_angle = (PI/2 + (in_angle - out_angle) + 0.2); //adds an offset of 0.15 rad	to make the fee more stable
		
		if(out_angle < 0){
			//inner leg is in back, add offset to inner ankle angle 
			FI_angle += 0.4; 
		}
		
		return FI_angle;	
 } 


enum StepProgress {
	PrePush,
	Push,
	Stride,
	Land,
	PostLand,
};

static int count_step = 0;
static int count_rock = 0;
static enum StepProgress stepProgress = Push;
#define thirty 0.25 //30 redians

 /*	Makes the robot move forward one step. */
 void step(void){
	   	struct ControllerData ctrlHip;
		struct ControllerData ctrlAnkOut;
		struct ControllerData ctrlAnkInn;
		
		float out_angle, in_angle, torque, feetInn_angle; 		
		feetInn_angle = mb_io_get_float(ID_MCFI_MID_ANKLE_ANGLE);


		out_angle = get_out_angle();
	// Set up a PD-controller on the hip angle:
		ctrlHip.kp = 6;
		ctrlHip.kd = 2;
		ctrlHip.vRef = 0.0;	   	
		// Calculate the toque needed to compensate for gravity pull
		in_angle = mb_io_get_float(ID_MCH_ANGLE);  // gets the hip angle (angle b/t inner&outer legs; pos when inner leg is in front)
		torque = leg_m * g * leg_r * Sin(in_angle);

	// Set up PD-controller on the outer foot angles
		ctrlAnkOut.kp = 3;
		ctrlAnkOut.kd = 0.5;
		ctrlAnkOut.vRef = 0.0;
		ctrlAnkOut.uRef = 0.0;

	// Set up PD-controller on the inner foot angles
		ctrlAnkInn.kp = 3;
		ctrlAnkInn.kd = 0.5;
		ctrlAnkInn.vRef = 0.0;
		ctrlAnkInn.uRef = 0.0;

		switch (stepProgress) {
		case PrePush:
			set_UI_LED(4, 'b'); 
			count_step ++;
			if(count_step >= 1000){
				stepProgress = Push;
				count_step = 0;
			}
			break;
		case Push:
			set_UI_LED(4, 'g'); //turns green LED on
			// swing inner leg forward
			ctrlHip.xRef = thirty;
			// Tried 1. outer feet stay flat on the ground
			//ctrlAnkOut.xRef = FO_flat_angle();
			// Tried 2. also push down outer feet
			//ctrlAnkOut.xRef = param_joint_ankle_push;//FO_flat_angle() + 1;//0.7;
			// Working 3. rock the outer feet
			if( count_step%800 <= 400){
				 ctrlAnkOut.xRef = param_joint_ankle_push/1;
			}else{
				 ctrlAnkOut.xRef = param_joint_ankle_push/1.3;
			}
			// push down inner feet
			ctrlAnkInn.xRef = param_joint_ankle_push; 			
			count_step ++;
			
			// switch to next state if the inner feet are pushed down far enough & outer feet are pushing down
			if(feetInn_angle>1.9 && count_step%800 > 200 && count_step%800 < 400){
				stepProgress = Stride;
				count_step = 0;
			} 
			break;
		case Stride:
			set_UI_LED(4, 'r');	//turns red LED on
			// swing inner leg forward
			ctrlHip.xRef = thirty*1.2;
			// outer feet flip up a little bit to prevent falling forward
			ctrlAnkOut.xRef = FO_flat_angle() - 0.5;
			// flip the inner feet all the way up 
			ctrlAnkInn.xRef = param_joint_ankle_flip;  
			
			// switch to next state if the inner leg is in front of outer leg
			if(in_angle > (thirty/5)){
				stepProgress = Land;
			}
			break;
		case Land:
			set_UI_LED(4, 'b');	//turn blue LED on
			//
			ctrlHip.xRef = thirty*1.2;
			// outer feet stay flat on the ground
			ctrlAnkOut.xRef = FO_flat_angle();
			// push down the inner feet	to prevent falling forward
			ctrlAnkInn.xRef = param_joint_ankle_push;
			
			count_step ++;
			if(count_step>=200){
				stepProgress = PostLand;
				count_step = 0;
			}
			break;
		case PostLand:
			set_UI_LED(4, 'y');
			//	
			ctrlHip.xRef = thirty*1.2;
			// both outer&inner feet stay flat on the ground
			ctrlAnkOut.xRef = FO_flat_angle();
			ctrlAnkInn.xRef = FI_flat_angle();
		
			break; 			
		}  
		
		// run the PD controllers 
		controller_hip(&ctrlHip);
		controller_ankleInner(&ctrlAnkInn);
		controller_ankleOuter(&ctrlAnkOut);
 }

 // initialize the FSM for walking
 void setPush(void){
 		stepProgress = PrePush;
		count_step = 0;
		count_rock = 0;
 }