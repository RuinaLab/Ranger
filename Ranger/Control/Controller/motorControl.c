#include <mb_includes.h> 
#include <motorControl.h>


#include "fsm.h"


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

	// Compute cp,cd,Ir without saturation
	/*C->Cp = (C->kp - param_hip_spring_const) / param_hip_motor_const;
	C->Cd = C->kd / param_hip_motor_const;

	Ir = (
	         C->uRef + C->kp * (C->xRef) + C->kd * (C->vRef)
	         - param_hip_spring_const * param_hip_spring_ref
	     ) / param_hip_motor_const;
	*/

	// Compute cp,cd,Ir with saturation
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
	
	return ir;
}


/* This function calls the low-level ankle (outer) controller. 
 */
void controller_ankleOuter( struct ControllerData * C ) {
	float current;
	float x = mb_io_get_float(ID_E_MCFO_RIGHT_ANKLE_ANGLE);
	float v = mb_io_get_float(ID_E_MCFO_RIGHT_ANKLE_RATE);
	//calls RangerAnkleControl function to get the current for the outer ankle	
	current = RangerAnkleControl(C, x, v);	   

	mb_io_set_float(ID_MCFO_COMMAND_CURRENT, current);
	mb_io_set_float(ID_MCFO_STIFFNESS, C->Cp);
	mb_io_set_float(ID_MCFO_DAMPNESS, C->Cd);
}


/* This function calls the low-level ankle (inner) controller. 
 */
void controller_ankleInner( struct ControllerData * C ) {
	float current;
	float x = mb_io_get_float(ID_E_MCFI_MID_ANKLE_ANGLE);
	float v	= mb_io_get_float(ID_E_MCFI_ANKLE_RATE);
	//calls RangerAnkleControl function to get the current for the inner ankle	
	current = RangerAnkleControl(C, x, v);

	mb_io_set_float(ID_MCFI_COMMAND_CURRENT, current);
	mb_io_set_float(ID_MCFI_STIFFNESS, C->Cp);
	mb_io_set_float(ID_MCFI_DAMPNESS, C->Cd);
}


/* Computes the current for ankle using saturation */
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

	return ir;
}

/* Computes the current to send to the ankle controller without saturation */
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
