#include <mb_includes.h> 
#include <motorControl.h>
#include "fsm.h"
#include "RangerMath.h"

/* Parameters needed for saturation */
#define uMAX_ANK 4
#define uMAX_HIP 8  //2*uMAX_ANK

/* A flag used to indicate whether to use saturation PD controllers */
static int saturation = 0;

/* Leg constants for gravity compensation */
float leg_m = 2.5;  ////TODO - This should be bigger... 4.96 I believe
float leg_r = 0.15;
float g = 9.8;

/* Relative/absolute angle parameters (refer to RangerNamingConventions in docs) 
 * shared by motorController.c, fsm.c & test.c */
float qr, qh, dqr, dqh, q0, q1, dq0, dq1; //relative
float th0, th1, dth0, dth1; //absolute

/* Relative reference of ankle joint angles */ 
////TODO - These are bounds... name them as such
float param_joint_ankle_flip = 0.3;
float param_joint_ankle_push = 2.5;
float param_joint_ankle_hold = 1.662;		////TODO - delete if unused

/* PD controller constants */ 
static const float param_hip_motor_const = 1.188;  // (Nm/Amp) Motor Constant, including gear box
static const float param_hip_spring_const = 8.045;  // (Nm/rad) Hip spring constant
static const float param_hip_spring_ref = 0.00;  // (rad) Hip spring reference angle
static const float param_hip_joint_inertia = 0.5616; // (kg-m^2) Swing leg moment of inertia about the hip joint

static const float param_ank_motor_const = 0.612;  // (Nm/Amp) Motor Constant, including gear box

////TODO - delete the following?
static const float param_ank_spring_const = 0.0;   ////HACK////  0.134;  // (Nm/rad) Ankle spring constant
static const float param_ank_spring_ref = 0.0; //// HACK //// 1.662;  // (rad) Ankle spring reference angle
static const float param_ank_joint_inertia = 0.07;//0.01; // (kg-m^2) Ankle moment of inertia about ankle joint
//the rise time stays roughly the same for ankle joint intertia in the range of 0.04-0.15 
//(~2s and matches the matlab simulation which shows a rise time of ~2s in plot)
//becaus rise time is independent of inertia
//seems like greater inertia makes the ankle swing faster
//greater inertia --> greater kp, cp, Ir 	


/* Updates the absolute/relative angle parameters in Ranger . 
////TODO - what calls this / how is it used?
*/
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


/* This function calls the low-level hip controller. 
 * The following fields of the input struct need to be set before calling this function:
 * 	{uRef, xRef, vRef, kp, kd} 
 * The following fields of the input struct are being set in the function:
 * 	{Cp, Cd}
 */
void controller_hip( struct ControllerData * C ) {
	float Ir;  // reference current, passed to the motor controller
	float gravity_fraction;

	if(!saturation){
		//calcuates the reference current, Cp, Cd without saturation
		C->Cp = (C->kp - param_hip_spring_const) / param_hip_motor_const;
		C->Cd = C->kd / param_hip_motor_const;
		Ir = (
		         C->uRef + C->kp * (C->xRef) + C->kd * (C->vRef)
		         - param_hip_spring_const * param_hip_spring_ref
		     ) / param_hip_motor_const;
		gravity_fraction =  C->uRef/( C->uRef + C->kp * (C->xRef) + C->kd * (C->vRef) - param_hip_spring_const * param_hip_spring_ref);
	}else{
		//calcuates the reference current using saturation
		float x = get_in_angle();
		float v	= get_in_ang_rate();
		Ir = get_hip_control_current_saturated(C, x, v);
	}

	mb_io_set_float(ID_MCH_COMMAND_CURRENT, Ir);
	mb_io_set_float(ID_MCH_STIFFNESS, C->Cp);
	mb_io_set_float(ID_MCH_DAMPNESS, C->Cd);
}


/* Helper function that computes the SATURATED current for hip 
 * The following fields of the input struct need to be set before calling this function:
 * 	{uRef, xRef, vRef, kp, kd} 
 * The following filds of the input struct are being set in the function:
 * 	{Cp, Cd}
 */
float get_hip_control_current_saturated(struct ControllerData * C, float x, float v){
	float ir;
	float uMax = uMAX_HIP;
	float uRaw = C->uRef + C->kp*(C->xRef-x) + C->kd*(C->vRef-v);
	float uSmooth = uMax*Tanh(uRaw/uMax);
	float S = 1 - Tanh(uRaw/uMax) * Tanh(uRaw/uMax);
	float Ux = S*C->kp;
	float Uv = S*C->kd;
	float uLin = uSmooth + Ux*x + Uv*v;  
	
	float uStar = uLin -  param_hip_spring_const * param_hip_spring_ref; 
	float kpStar = Ux - param_hip_spring_const;	
	float kdStar = Uv; 
	
	C->Cp = kpStar / param_hip_motor_const; 
	C->Cd = kdStar / param_hip_motor_const; 
	ir = uStar / param_hip_motor_const;
	
	return ir;
}


/* This function calls the low-level OUTER ankle controller. 
 * The following fields of the input struct need to be set before calling this function:
 * 	{uRef, xRef, vRef, kp, kd} 
 * The following fields of the input struct are being set in the function:
 * 	{Cp, Cd}
 */
void controller_ankleOuter( struct ControllerData * C ) {
	float current;
	if(!saturation){
		//Calcuates the reference current, Cp, Cd without saturation
		current = getAnkleControllerCurrent(C);
	}else{
		//Calcuates the reference current, Cp, Cd using saturation
		float x = mb_io_get_float(ID_E_MCFO_RIGHT_ANKLE_ANGLE);
		float v = mb_io_get_float(ID_E_MCFO_RIGHT_ANKLE_RATE);	
		current = get_ank_control_current_saturated(C, x, v);
	}
	mb_io_set_float(ID_MCFO_COMMAND_CURRENT, current);
	mb_io_set_float(ID_MCFO_STIFFNESS, C->Cp);
	mb_io_set_float(ID_MCFO_DAMPNESS, C->Cd);
}


/* This function calls the low-level INNER ankle controller. 
 * The following fields of the input struct need to be set before calling this function:
 * 	{uRef, xRef, vRef, kp, kd}
 * The following fields of the input struct are being set in the function:
 * 	{Cp, Cd}
 */
void controller_ankleInner( struct ControllerData * C ) {
	float current;
	if(!saturation){
		current = getAnkleControllerCurrent(C);
	}else{
		float x = mb_io_get_float(ID_E_MCFI_MID_ANKLE_ANGLE);
		float v	= mb_io_get_float(ID_E_MCFI_ANKLE_RATE);
		current = get_ank_control_current_saturated(C, x, v);
	}
	mb_io_set_float(ID_MCFI_COMMAND_CURRENT, current);
	mb_io_set_float(ID_MCFI_STIFFNESS, C->Cp);
	mb_io_set_float(ID_MCFI_DAMPNESS, C->Cd);
}


/* Helper function that computes the current to send to the ankle controller WITHOUT SATURATION 
 * The following fields of the input struct need to be set before calling this function:
 * 	{uRef, xRef, vRef, kp, kd} 
 * The following filds of the input struct are being set in the function:
 * 	{Cp, Cd} 
 */
float getAnkleControllerCurrent( struct ControllerData * C ){
	float Ir;  // reference current, passed to the motor controller

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


/* Helper function that Computes the saturated current for the ankle 
 * The following fields of the input struct need to be set before calling this function:
 * 	{uRef, xRef, vRef, kp, kd} 
 * The following filds of the input struct are being set in the function:
 * 	{Cp, Cd}
 */
float get_ank_control_current_saturated(struct ControllerData * C, float x, float v){
	float ir;
	float uMax = uMAX_ANK;
	float uRaw = C->uRef + C->kp*(C->xRef-x) + C->kd*(C->vRef-v);
	float uSmooth = uMax*Tanh(uRaw/uMax);
	float S = 1 - Tanh(uRaw/uMax) * Tanh(uRaw/uMax);
	float Ux = S*C->kp;
	float Uv = S*C->kd;
	float uLin = uSmooth + Ux*x + Uv*v;  

	float uStar = uLin -  param_ank_spring_const * param_ank_spring_ref; 
	float kpStar = Ux - param_ank_spring_const;
	float kdStar = Uv;
	
	// Check to make sure ankle joint doesn't go out of bound
	if(C->xRef > param_joint_ankle_push){
		C->xRef = param_joint_ankle_push;
	}else if(C->xRef < param_joint_ankle_flip){
		C->xRef = param_joint_ankle_flip;
	} 
	
	C->Cp = kpStar / param_ank_motor_const; 
	C->Cd = kdStar / param_ank_motor_const; 
	ir = uStar / param_ank_motor_const;	
	
	return ir;
}


/* Turns off motors. */
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



/* Returns the torque needed to compensate for gravity pull on the legs. */
float hip_gravity_compensation(void){
	float u = leg_m * g * leg_r;

	if(FI_on_ground() && !FO_on_ground()){
		//track outer, only inner feet on ground
		return -u * Sin(th0); 
	}else if(!FI_on_ground() && FO_on_ground()){
		//track inner, only outer feet on ground 	
		return  u * Sin(th1);	
	}
	return 0.0;
}


/* Computes the controller set-points for tracking a RELATIVE angle in the hip. 
 * Sets the following fields of the input struct:
 * 	{xRef, vRef, uRef, kp, kd}
 */
void hip_track_rel(struct ControllerData * ctrlData, float qh_ref, float dqh_ref, float KP, float KD){
	ctrlData->xRef = qh_ref;
	ctrlData->vRef = dqh_ref;
	ctrlData->uRef = hip_gravity_compensation();
	//ctrlData->uRef = 0.0;
	ctrlData->kp = KP;
	ctrlData->kd = KD;
	return;
}


/* Computes the controller set-points for the hip when OUTER FEET are on ground and inner feet in the air.
 * Sets the following fields of the input struct:
 * 	{xRef, vRef, uRef, kp, kd}
 */
void hip_scissor_track_outer(struct ControllerData * ctrlData, float offset, float rate, float KP, float KD){
	float th1Ref = offset - rate * th0;
	float dth1Ref = -rate*dth0; 
	
	ctrlData->xRef = th1Ref - th0;
	ctrlData->vRef = dth1Ref - dth0; 
	ctrlData->uRef = hip_gravity_compensation();
	//ctrlData->uRef = 0.0;
	ctrlData->kp = KP;
	ctrlData->kd = KD;
}

/* Computes the controller set-points for the hip when INNER FEET are on ground and outer feet in the air.
 * Sets the following fields of the input struct:
 * 	{xRef, vRef, uRef, kp, kd}
 */
void hip_scissor_track_inner(struct ControllerData * ctrlData, float offset, float rate, float KP, float KD){
	float th0Ref = offset - rate * th1;
	float dth0Ref = -rate*dth1; 
	
	ctrlData->xRef = th1 - th0Ref;
	ctrlData->vRef = dth1 - dth0Ref;
	ctrlData->uRef = hip_gravity_compensation();
	//ctrlData->uRef = 0.0;
 	ctrlData->kp = KP;
	ctrlData->kd = KD;
}

/* Computes the OUTER ankle controller set-points to track the desired ABSOLUTE angle and rate. 
 * Sets the following fields of the input struct:
 * {xRef, vRef, uRef, kp, kd}
 */
void out_ank_track_abs(struct ControllerData * ctrlData, float phi0_ref, float dphi0_ref, float u_ref, float KP, float KD){
	//convert absolute to relative
	//q0 = ZERO_POS_OUT - phi0 + th0
	//th0 = -qr
	ctrlData->xRef = ZERO_POS_OUT - phi0_ref - qr; 
	ctrlData->vRef = -dphi0_ref - dqr;
	ctrlData->uRef = u_ref;

	ctrlData->kp = KP;
	ctrlData->kd = KD;
	return;
}


/* Computes the INNER ankle controller set-points to track the desired ABSOLUTE angle and rate. 
 * Sets the following fields of the input struct:
 * {xRef, vRef, uRef, kp, kd}
 */
void inn_ank_track_abs(struct ControllerData * ctrlData, float phi1_ref, float dphi1_ref, float u_ref, float KP, float KD){
	//convert absolute to relative
	//q1 = ZERO_POS_INN - phi1 + th1 
	//th1 = qh - qr
	ctrlData->xRef = ZERO_POS_INN - phi1_ref + qh - qr;
	ctrlData->vRef = -dphi1_ref + dqh - dqr;
	ctrlData->uRef = u_ref;

	ctrlData->kp = KP;
	ctrlData->kd = KD;
	return;
}

