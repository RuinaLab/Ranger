#include <mb_includes.h> 
#include <motorControl.h>


#include "Trajectory.h"
#include "TrajData.h"
#include "math.h" //for fmod()
#include "RangerMath.h"	//for Sin()

#define DATA TRAJ_DATA_Test0
static const float leg_m = 2.5; //4.95
static const float leg_r = 0.15; //length to the center of mass 
static const float g = 9.8;

/* joint limits */
static const float param_joint_ankle_flip = 0.3; // Hard stop at 0.0. Foot flips up to this angle to clear ground.
static const float param_joint_ankle_push = 2.5; // Hard stop at 3.0. Foot pushes off to inject energy, this is maximum bound.
static const float param_joint_ankle_hold = 1.662;

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

	C->kp = param_hip_joint_inertia * (C->wn) * (C->wn);
	C->kd = 2.0 * param_hip_joint_inertia * (C->wn) * (C->xi);

	C->Cp = (C->kp - param_hip_spring_const) / param_hip_motor_const;
	C->Cd = C->kd / param_hip_motor_const;

	Ir = (
	         C->uRef + C->kp * (C->xRef) + C->kd * (C->vRef)
	         - param_hip_spring_const * param_hip_spring_ref
	     ) / param_hip_motor_const;

	mb_io_set_float(ID_MCH_COMMAND_CURRENT, Ir);
	mb_io_set_float(ID_MCH_STIFFNESS, C->Cp);
	mb_io_set_float(ID_MCH_DAMPNESS, C->Cd);
}




/* Computes the current to send to the ankle controller 
 */
float getAnkleControllerCurrent( struct ControllerData * C ){
	float Cp;  // proportional gain, current, passed to the motor controller
	float Cd;  // derivative gain, current, passed to the motor controller
	float Ir;  // reference current, passed to the motor controller

	C->kp = param_ank_joint_inertia * (C->wn) * (C->wn);
	C->kd = 2.0 * param_ank_joint_inertia * (C->wn) * (C->xi);

	C->Cp = (C->kp - param_ank_spring_const) / param_ank_motor_const;
	C->Cd = C->kd / param_ank_motor_const;

	// Check to make sure ankle joint doesn't go out of bound
	if(C->xRef > param_joint_ankle_push){
		C->xRef = param_joint_ankle_push;
	}else if(C->xRef < param_joint_ankle_flip){
		C->xRef = param_joint_ankle_flip;
	}

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
	current = getAnkleControllerCurrent(C);	
	mb_io_set_float(ID_MCFO_COMMAND_CURRENT, current);
	mb_io_set_float(ID_MCFO_STIFFNESS, C->Cp);
	mb_io_set_float(ID_MCFO_DAMPNESS, C->Cd);
}


/* This function calls the low-level ankle (inner) controller. 
 */
void controller_ankleInner( struct ControllerData * C ) {
	float current;
	current = getAnkleControllerCurrent(C);
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


 int counter = 0; 

 void test_inner_foot() { 
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

 /* Runs a simple test of the frequency controllers */
 void test_freq_control() {
 		struct ControllerData ctrlHip;
		struct ControllerData ctrlAnkOut;
		struct ControllerData ctrlAnkInn;

		
		float hipRefAmp;  // square wave reference amplitude
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
	
	
	// Set up the angle reference, so that outer&inner foot trace a step-function
	// (swing periodically between 0.5 and 1.5 rad)  			
	/*	if(counter <= 1000){
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
		ctrlAnkOut.wn = 5;
		ctrlAnkOut.xi = 1;
		ctrlAnkOut.vRef = 0;
		ctrlAnkOut.uRef = 0;
		controller_ankleOuter(&ctrlAnkOut);

	// Run a PD-controller on the inner foot angles:
		ctrlAnkInn.wn = 3.5;
		ctrlAnkInn.xi = 1;
		ctrlAnkInn.vRef = 0;
		ctrlAnkInn.uRef = 0;
		controller_ankleInner(&ctrlAnkInn);
		*/
 }

 /* Runs a simple test of tracing a trajectory
 */
 void test_trajectory() {
 		struct ControllerData ctrlHip;
		struct ControllerData ctrlAnkOut;
		struct ControllerData ctrlAnkInn;
		float hip_angle;
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

		mb_io_set_float(ID_CTRL_TEST_W0, y);	
	
	// Calculate the toque needed to compensate for gravity pull
		hip_angle = mb_io_get_float(ID_MCH_ANGLE); 
		torque = leg_m * g * leg_r * Sin(hip_angle); 

	// Run a PD-controller on the hip angle:
		ctrlHip.wn = 3.5;
		ctrlHip.xi = 1;
		
		ctrlHip.xRef = y;
		ctrlHip.vRef = yd;
		ctrlHip.uRef = torque;
		controller_hip(&ctrlHip);

	// Run a PD-controller on the outer foot angles:
	/*	ctrlAnkOut.xRef = 1.0;
		ctrlAnkOut.vRef = 0.0;
		ctrlAnkOut.uRef = 0.0;
		controller_ankleOuter(&ctrlAnkOut);

	// Run a PD-controller on the inner foot angles:
		//ctrlAnkInn.kp = 0.0;
		//ctrlAnkInn.kd = 0.0;
		ctrlAnkInn.xRef = 1.0;
		ctrlAnkInn.vRef = 0.0;
		ctrlAnkInn.uRef = 0.0;
		controller_ankleInner(&ctrlAnkInn);
		*/
 }

