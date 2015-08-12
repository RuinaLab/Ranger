#include <mb_includes.h> 
#include <motorControl.h>

#include "Trajectory.h"
#include "TrajData.h"
#include "math.h" //for fmod()
#include "RangerMath.h"	//for Sin()
#include "fsm.h"

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

#define PI 3.141592653589793
#define DATA TRAJ_DATA_Test0
#define thirty 0.52

float leg_m = 2.5;
float leg_r = 0.15;
float g = 9.8;
float param_joint_ankle_flip = 0.3;
float param_joint_ankle_push = 2.5;
float param_joint_ankle_hold = 1.662;

/* PD controller constants */ 
static const float param_hip_motor_const = 1.188;  // (Nm/Amp) Motor Constant, including gear box
static const float param_hip_spring_const = 8.045;  // (Nm/rad) Hip spring constant
static const float param_hip_spring_ref = 0.00;  // (rad) Hip spring reference angle
static const float param_hip_joint_inertia = 0.5616; // (kg-m^2) Swing leg moment of inertia about the hip joint

static const float param_ank_motor_const = 0.612;  // (Nm/Amp) Motor Constant, including gear box
static const float param_ank_spring_const = 0.134;  // (Nm/rad) Ankle spring constant
static const float param_ank_spring_ref = 1.662;  // (rad) Ankle spring reference angle
static const float param_ank_joint_inertia = 0.07;//0.01; // (kg-m^2) Ankle moment of inertia about ankle joint
//the rise time stays roughly the same for ankle joint intertia in the range of 0.04-0.15 
//(~2s and matches the matlab simulation which shows a rise time of ~2s in plot)
//becaus rise time is independent of inertia
//seems like greater inertia makes the ankle swing faster
//greater inertia --> greater kp, cp, Ir 	


/* This function calls the low-level hip controller. */
void controller_hip( struct ControllerData * C ) {
  
	float Ir;  // reference current, passed to the motor controller

	//C->kp = param_hip_joint_inertia * (C->wn) * (C->wn);
	//C->kd = 2.0 * param_hip_joint_inertia * (C->wn) * (C->xi);

	/*C->Cp = (C->kp - param_hip_spring_const) / param_hip_motor_const;
	C->Cd = C->kd / param_hip_motor_const;

	Ir = (
	         C->uRef + C->kp * (C->xRef) + C->kd * (C->vRef)
	         - param_hip_spring_const * param_hip_spring_ref
	     ) / param_hip_motor_const;
	*/

	float x = get_in_angle();
	float v	= get_in_ang_rate();
	Ir = RangerHipControl(C, x, v);

	mb_io_set_float(ID_MCH_COMMAND_CURRENT, Ir);
	mb_io_set_float(ID_MCH_STIFFNESS, C->Cp);
	mb_io_set_float(ID_MCH_DAMPNESS, C->Cd);
}

#define uMAX_ANK 4
#define uMAX_HIP 8  //2*uMAX_ANK

/* Computes the current for hip using saturation*/
float RangerHipControl(struct ControllerData * C, float x, float v){
	float ir;
	float uMax = uMAX_HIP;
	float uRaw = C->uRef + C->kp*(C->xRef-x) + C->kd*(C->vRef-v);
	float uSmooth = uMax*tanh(uRaw/uMax);
	float S = 1 - tanh(uRaw/uMax) * tanh(uRaw/uMax);
	float Ux = S*C->kp;
	float Uv = S*C->kd;
	float uLin = uSmooth + Ux*x + Uv*v;  
	
	float uStar = uLin -  param_hip_spring_const * param_hip_spring_ref; //float uStar = uLin - kSpring*xSpring;
	float kpStar = Ux - param_hip_spring_const;	//float kpStar = Ux - kSpring;
	float kdStar = Uv; //float kdStar = Uv; 
	
	C->Cp = kpStar / param_hip_motor_const; 	//float cp = kpStar/kMotor;
	C->Cd = kdStar / param_hip_motor_const; 	//float cd = kdStar/kMotor;
	ir = uStar / param_hip_motor_const;	//float ir = uStar/kMotor;
	
	/*mb_io_set_float(ID_CTRL_TEST_W2, C->Cp);
	mb_io_set_float(ID_CTRL_TEST_W3, C->Cd);
	mb_io_set_float(ID_CTRL_TEST_W4, ir);
	*/
	return ir;
}


/* Computes the current for ankle using saturation*/
float RangerAnkleControl(struct ControllerData * C, float x, float v){
	float ir;
	float uMax = uMAX_ANK;
	float uRaw = C->uRef + C->kp*(C->xRef-x) + C->kd*(C->vRef-v);
	float uSmooth = uMax*tanh(uRaw/uMax);
	float S = 1 - tanh(uRaw/uMax) * tanh(uRaw/uMax);
	float Ux = S*C->kp;
	float Uv = S*C->kd;
	float uLin = uSmooth + Ux*x + Uv*v;  
	
	float uStar = uLin -  param_ank_spring_const * param_ank_spring_ref; //float uStar = uLin - kSpring*xSpring;
	float kpStar = Ux - param_ank_spring_const;	//float kpStar = Ux - kSpring;
	float kdStar = Uv; //float kdStar = Uv; 
	
	C->Cp = kpStar / param_ank_motor_const; 	//float cp = kpStar/kMotor;
	C->Cd = kdStar / param_ank_motor_const; 	//float cd = kdStar/kMotor;
	ir = uStar / param_ank_motor_const;	//float ir = uStar/kMotor;
	
	/*mb_io_set_float(ID_CTRL_TEST_W2, C->Cp);
	mb_io_set_float(ID_CTRL_TEST_W3, C->Cd);
	mb_io_set_float(ID_CTRL_TEST_W4, ir);
	*/
	return ir;
}




/* Computes the current to send to the ankle controller 
 */
float getAnkleControllerCurrent( struct ControllerData * C ){
	//float Cp;  // proportional gain, current, passed to the motor controller
	//float Cd;  // derivative gain, current, passed to the motor controller
	float Ir;  // reference current, passed to the motor controller

	//C->kp = param_ank_joint_inertia * (C->wn) * (C->wn);
	//C->kd = 2.0 * param_ank_joint_inertia * (C->wn) * (C->xi);

	C->Cp = (C->kp - param_ank_spring_const) / param_ank_motor_const;
	C->Cd = C->kd / param_ank_motor_const;

	// Check to make sure ankle joint doesn't go out of bound
	if(C->xRef > param_joint_ankle_push){
		C->xRef = param_joint_ankle_push;
	}else if(C->xRef < param_joint_ankle_flip){
		C->xRef = param_joint_ankle_flip;
	}

	mb_io_set_float(ID_CTRL_TEST_W0, C->uRef + C->kp * (C->xRef) + C->kd * (C->vRef));

	Ir = (
	         C->uRef + C->kp * (C->xRef) + C->kd * (C->vRef)
	         - param_ank_spring_const * param_ank_spring_ref
	     ) / param_ank_motor_const;
	return Ir;
}




/* This function calls the low-level ankle (outer) controller. 
 */
void controller_ankleOuter( struct ControllerData * C ) {
	float current;
	//current = getAnkleControllerCurrent(C);
	float x = mb_io_get_float(ID_E_MCFO_RIGHT_ANKLE_ANGLE);
	float v = mb_io_get_float(ID_E_MCFO_RIGHT_ANKLE_RATE);	
	current = RangerAnkleControl(C, x, v);

	mb_io_set_float(ID_MCFO_COMMAND_CURRENT, current);
	mb_io_set_float(ID_MCFO_STIFFNESS, C->Cp);
	mb_io_set_float(ID_MCFO_DAMPNESS, C->Cd);
}


/* This function calls the low-level ankle (inner) controller. 
 */
void controller_ankleInner( struct ControllerData * C ) {
	float current;
	//current = getAnkleControllerCurrent(C);
	float x = mb_io_get_float(ID_E_MCFI_MID_ANKLE_ANGLE);
	float v	= mb_io_get_float(ID_E_MCFI_ANKLE_RATE);
	current = RangerAnkleControl(C, x, v);

	mb_io_set_float(ID_MCFI_COMMAND_CURRENT, current);
	mb_io_set_float(ID_MCFI_STIFFNESS, C->Cp);
	mb_io_set_float(ID_MCFI_DAMPNESS, C->Cd);
}


/* Turns off motors.
 */
 void disable_motors(){

	mb_io_set_float(ID_MCFI_COMMAND_CURRENT, 0.0);
	mb_io_set_float(ID_MCFI_STIFFNESS, 0.0);
	mb_io_set_float(ID_MCFI_DAMPNESS, 0.0);

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
		ctrlAnkInn.wn = mb_io_get_float(ID_CTRL_TEST_R0); //7
 		ctrlAnkInn.xi = mb_io_get_float(ID_CTRL_TEST_R1); //0.7
		ctrlAnkInn.vRef = 0;
		ctrlAnkInn.uRef = 0;
		controller_ankleInner(&ctrlAnkInn);
 }

 /* Runs a simple test of the frequency controllers, connecting them to LabVIEW */
 void test_freq_control(void) {
 		struct ControllerData ctrlHip;
		struct ControllerData ctrlAnkOut;
		struct ControllerData ctrlAnkInn;

		// Run a PD-controller on the inner foot angles:
		ctrlAnkOut.wn = mb_io_get_float(ID_CTRL_TEST_R0);
		ctrlAnkOut.xi = mb_io_get_float(ID_CTRL_TEST_R1);
		ctrlAnkOut.xRef = mb_io_get_float(ID_CTRL_TEST_R2);
		ctrlAnkOut.vRef = mb_io_get_float(ID_CTRL_TEST_R3);
		ctrlAnkOut.uRef = mb_io_get_float(ID_CTRL_TEST_R4);
		controller_ankleOuter(&ctrlAnkOut);	

		// Run a PD-controller on the inner foot angles:
		ctrlAnkInn.wn = mb_io_get_float(ID_CTRL_TEST_R0);
		ctrlAnkInn.xi = mb_io_get_float(ID_CTRL_TEST_R1);
		ctrlAnkInn.xRef = mb_io_get_float(ID_CTRL_TEST_R2);
		ctrlAnkInn.vRef = mb_io_get_float(ID_CTRL_TEST_R3);
		ctrlAnkInn.uRef = mb_io_get_float(ID_CTRL_TEST_R4);
		controller_ankleInner(&ctrlAnkInn);
	
		// Run a PD-controller on the inner foot angles:
		ctrlHip.wn = mb_io_get_float(ID_CTRL_TEST_R5);
		ctrlHip.xi = mb_io_get_float(ID_CTRL_TEST_R6);
		ctrlHip.xRef = mb_io_get_float(ID_CTRL_TEST_R7);
		ctrlHip.vRef = mb_io_get_float(ID_CTRL_TEST_R8);
		ctrlHip.uRef = mb_io_get_float(ID_CTRL_TEST_R9);
		controller_hip(&ctrlHip);
		
 }

 /* Generates data for fitting a motor model */
 void test_ankle_motor_model(void) {
 		struct ControllerData ctrlHip;
		struct ControllerData ctrlAnkOut;
		struct ControllerData ctrlAnkInn;

		// Open loop tracking test (random values between 0.2 and 2.0)
		float refAngle[60] = {1.735,  1.320,  0.832,  1.124,  0.923,  0.337,  0.632,  0.422,  0.531,  0.632,  0.951,  0.289,  1.825,  1.901,  1.084,  1.081,  0.808,  1.820,  0.865,  0.400,  1.604,  0.902,  0.635,  0.927,  0.374,  0.438,  1.896,  1.921,  1.235,  0.308,  0.623,  0.836,  1.678,  0.228,  0.277,  0.504,  1.368,  1.517,  1.366,  1.012,  1.185,  0.733,  1.540,  0.540,  1.436,  0.530,  0.863,  1.326,  1.604,  0.346,  1.873,  1.596,  1.076,  0.985,  1.004,  0.751,  1.115,  1.119,  1.672,  1.631};  		

		// Update value with a period of:
		float updatePeriod = 750.0;   // in milliseconds

		// Select the correct reference angle
		float time = mb_io_get_float(ID_TIMESTAMP);
		int nPeriods = (int) (time/updatePeriod);
		int index = nPeriods % 60;

		// Run a PD-controller on the inner foot angles:
		ctrlAnkOut.wn = 8.0;
		ctrlAnkOut.xi = 0.7;
		ctrlAnkOut.xRef = refAngle[index];
		ctrlAnkOut.vRef = 0.0;
		ctrlAnkOut.uRef = 0.0;
		controller_ankleOuter(&ctrlAnkOut);	

		// Run a PD-controller on the inner foot angles:
		ctrlAnkInn.wn = 8.0;
		ctrlAnkInn.xi = 0.7;
		ctrlAnkInn.xRef = refAngle[index];
		ctrlAnkInn.vRef = 0.0;
		ctrlAnkInn.uRef = 0.0;
		controller_ankleInner(&ctrlAnkInn);
	
		// Run a PD-controller on the inner foot angles:
		ctrlHip.wn = 0.0;
		ctrlHip.xi = 0.0;
		ctrlHip.xRef = 0.0;
		ctrlHip.vRef = 0.0;
		ctrlHip.uRef = 0.0;
		controller_hip(&ctrlHip);
	
 }


 /* Runs a simple test that makes the inner leg traces a sin wave 
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
		float t = fmod(sys_t, max_t);
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
		ctrlHip.wn = 3.5;
		ctrlHip.xi = 0.8;
		
		ctrlHip.xRef = y;
		//mb_io_set_float(ID_CTRL_TEST_W0, ctrlHip.xRef);
		ctrlHip.vRef = yd;
		ctrlHip.uRef = torque;
		controller_hip(&ctrlHip);

	// Run a PD-controller on the outer foot angles: make the feet stay flat wrt the ground
		ctrlAnkOut.wn = 7;
		ctrlAnkOut.xi = 0.8;
		ctrlAnkOut.xRef = FO_flat_angle();
		ctrlAnkOut.vRef = 0.0;
		ctrlAnkOut.uRef = 0.0;
		controller_ankleOuter(&ctrlAnkOut);

	
	// Run a PD-controller on the inner foot angles: make the feet stay flat wrt the ground
		ctrlAnkInn.wn = 7;
		ctrlAnkInn.xi = 0.8;
		ctrlAnkInn.xRef = FI_flat_angle(); 	
		ctrlAnkInn.vRef = 0.0;
		ctrlAnkInn.uRef = 0.0;
		controller_ankleInner(&ctrlAnkInn);
 }

 /* A test that makes the inner leg track a sin wave generated using Sin function from RangerMath. */ 
 void track_sin(void){
 	 	struct ControllerData ctrlHip;
		float sys_t = mb_io_get_float(ID_TIMESTAMP)/1000;//converts the system_time from ms to s
		float t = fmod(sys_t, 2*PI);

	// Run a PD-controller on the hip angle:
		ctrlHip.wn = 3.5;
		ctrlHip.xi = 0.8;
				
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

	// Run a PD-controller on the outer foot angles: make the feet stay flat wrt the ground
		ctrlAnkOut.wn = 7;
		ctrlAnkOut.xi = 0.8;
		ctrlAnkOut.xRef = param_joint_ankle_flip;
		ctrlAnkOut.vRef = 0.0;
		ctrlAnkOut.uRef = 0.0;
		controller_ankleOuter(&ctrlAnkOut);
	// Run a PD-controller on the inner foot angles: make the feet stay flat wrt the ground
		ctrlAnkInn.wn = 7;
		ctrlAnkInn.xi = 0.8;
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

 /* Returns FO_angle_rate = -out_angle_rate = -gyro_rate */
 float FO_flat_rate(void){
 		return -get_out_ang_rate();
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

 /* Returns FI_angle_rate = -out_angle_rate + in_angle_rate = -gyro_rate + differentiate_hip_angle */
 float FI_flat_rate(void){
 		return -get_out_ang_rate() + get_in_ang_rate();	
 }


 /* Check if the angle b/t two legs is 30+/-5 degrees, inner leg is in the back. 
  *	Turn the 3rd LED red if the angle is in this range. 
  */
 void check_30(void){
 		float in_angle, out_angle;
		float target = thirty/2; //check for 15 degrees now!
		float offset = 0.087;  //5 dgrees
		out_angle = get_out_angle();
		in_angle = get_in_angle();
 		if(out_angle>=(-target-offset) && out_angle<=(-target+offset) && in_angle>=(-target-offset) && in_angle<=(-target+offset)){
			set_UI_LED(3, 'r');			
		}else{
			set_io_ul(ID_UI_SET_LED_3, 0x000000);
		}
		return;
 }

 /*	Makes the robot move forward one step. */
 void step(void){
	   	struct ControllerData ctrlHip;
		struct ControllerData ctrlAnkOut;
		struct ControllerData ctrlAnkInn;
		
		float out_angle, in_angle, torque, feetInn_angle; 		
		feetInn_angle = mb_io_get_float(ID_MCFI_MID_ANKLE_ANGLE);


		out_angle = get_out_angle();
	// Set up a PD-controller on the hip angle:
		ctrlHip.wn = 3.5;
		ctrlHip.xi = 0.8;
		ctrlHip.vRef = 0.0;	   	
		// Calculate the toque needed to compensate for gravity pull
		in_angle = mb_io_get_float(ID_MCH_ANGLE);  // gets the hip angle (angle b/t inner&outer legs; pos when inner leg is in front)
		torque = leg_m * g * leg_r * Sin(in_angle);

	// Set up PD-controller on the outer foot angles
		ctrlAnkOut.wn = 7;
		ctrlAnkOut.xi = 0.8;
		ctrlAnkOut.vRef = 0.0;
		ctrlAnkOut.uRef = 0.0;

	// Set up PD-controller on the inner foot angles
		ctrlAnkInn.wn = 7;
		ctrlAnkInn.xi = 0.8;
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

