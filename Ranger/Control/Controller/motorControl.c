#include <mb_includes.h>
#include <motorControl.h>
#include "fsm.h"
#include "RangerMath.h"
#include "robotParameters.h"
#include "mb_estimator"

/* Ankle joint limits (in relative angle) */
static const float param_joint_ankle_min = 0.3;	 // Hard stop at 0.0. Foot flips up to this angle to clear ground.
static const float param_joint_ankle_max = 2.5;	 // Hard stop at 3.0. Foot pushes off to inject energy, this is maximum bound.
static const float param_ank_motor_const = 0.612;  // (Nm/Amp) Motor Constant, including gear box

/* PD controller constants */
static const float param_hip_motor_const = 1.188;  // (Nm/Amp) Motor Constant, including gear box
static const float param_hip_spring_const = 8.045;  // (Nm/rad) Hip spring constant
static const float param_hip_spring_ref = 0.00;  // (rad) Hip spring reference angle


/* An important function that updates the absolute/relative angle vairables used in Ranger code.
 * Needs to be called every clock cycle in order to use functions implemented in motroController.c, fsm.c & unit_test.c,
 * otherwise, these functions would get old sensor readings and output wrong results.
 */
void angles_update(void) {
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

	//calcuates the reference current, Cp, Cd without saturation
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


/* This function calls the low-level OUTER ankle controller.
 * The following fields of the input struct need to be set before calling this function:
 * 	{uRef, xRef, vRef, kp, kd}
 * The following fields of the input struct are being set in the function:
 * 	{Cp, Cd}
 */
void controller_ankleOuter( struct ControllerData * C ) {
	float current;
	//Calcuates the reference current, Cp, Cd without saturation
	current = getAnkleControllerCurrent(C);
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
	current = getAnkleControllerCurrent(C);
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
float getAnkleControllerCurrent( struct ControllerData * C ) {
	float Ir;  // reference current, passed to the motor controller

	C->Cp = C->kp / param_ank_motor_const;
	C->Cd = C->kd / param_ank_motor_const;

	// Check to make sure ankle joint doesn't go out of bound
	if (C->xRef > param_joint_ankle_max) {
		C->xRef = param_joint_ankle_max;
	} else if (C->xRef < param_joint_ankle_min) {
		C->xRef = param_joint_ankle_min;
	}

	Ir = (  C->uRef + C->kp * (C->xRef) + C->kd * (C->vRef) ) / param_ank_motor_const;
	return Ir;
}


/* Turns off motors. */
void disable_motors() {
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


/* Turns a motor off at a high levelby setting all fields of the input instruct to zero */
void motor_off(struct ControllerData * C) {
	C->kp = 0.0;
	C->kd = 0.0;
	C->xRef = 0.0;
	C->vRef = 0.0;
	C->uRef = 0.0;
	C->GC = 0;
}


/* Returns the torque needed to compensate for gravity. Returns 0.0 if the robot 
 * is in either flight or double stance. */
float hip_gravity_compensation(void) {
	float uGravity = PARAM_m * PARAM_g * PARAM_c; // Torque scale factor
	bool c0 = getContactOuter(); // Is the outer foot in contact?
	bool c1 = getContactInner(); // Is the inner foot in contact?

	if (c0 && !c1) { // Single stance, outer leg on ground
		return  uGravity * Sin(STATE_th1);
	} else if (!c0 && c1) { // Single stance, inner leg on ground
		return -uGravity * Sin(STATE_th0);
	} else {  // Flight or double stance - no compensation
		return 0.0; 
	}

}


/* Computes the controller set-points for tracking a RELATIVE angle in the hip.
 * The GC field of the input struct needs to be set before calling this function.
 * Sets the following fields of the input struct:
 * 	{xRef, vRef, uRef, kp, kd}
 */
void hip_track_rel(struct ControllerData * ctrlData, float qh_ref, float dqh_ref, float KP, float KD) {
	ctrlData->xRef = qh_ref;
	ctrlData->vRef = dqh_ref;
	if (ctrlData->GC) {
		ctrlData->uRef = hip_gravity_compensation();
	} else {
		ctrlData->uRef = 0.0;
	}
	ctrlData->kp = KP;
	ctrlData->kd = KD;
	return;
}


/* Computes the controller set-points for the hip when OUTER FEET are on ground and inner feet in the air.
 * The GC field of the input struct needs to be set before calling this function.
 * Sets the following fields of the input struct:
 * 	{xRef, vRef, uRef, kp, kd}
 */
void hip_scissor_track_outer(struct ControllerData * ctrlData, float offset, float rate, float KP, float KD) {
	float th1Ref = offset - rate * th0;
	float dth1Ref = -rate * dth0;

	ctrlData->xRef = th1Ref - th0;
	ctrlData->vRef = dth1Ref - dth0;
	if (ctrlData->GC) {
		ctrlData->uRef = hip_gravity_compensation();
	} else {
		ctrlData->uRef = 0.0;
	}
	ctrlData->kp = KP;
	ctrlData->kd = KD;
}

/* Computes the controller set-points for the hip when INNER FEET are on ground and outer feet in the air.
 * The GC field of the input struct needs to be set before calling this function.
 * Sets the following fields of the input struct:
 * 	{xRef, vRef, uRef, kp, kd}
 */
void hip_scissor_track_inner(struct ControllerData * ctrlData, float offset, float rate, float KP, float KD) {
	float th0Ref = offset - rate * th1;
	float dth0Ref = -rate * dth1;

	ctrlData->xRef = th1 - th0Ref;
	ctrlData->vRef = dth1 - dth0Ref;
	if (ctrlData->GC) {
		ctrlData->uRef = hip_gravity_compensation();
	} else {
		ctrlData->uRef = 0.0;
	}
	ctrlData->kp = KP;
	ctrlData->kd = KD;
}

/* Computes the OUTER ankle controller set-points to track the desired ABSOLUTE angle and rate.
 * Sets the following fields of the input struct:
 * {xRef, vRef, uRef, kp, kd}
 */
void out_ank_track_abs(struct ControllerData * ctrlData, float phi0_ref, float dphi0_ref, float u_ref, float KP, float KD) {
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
void inn_ank_track_abs(struct ControllerData * ctrlData, float phi1_ref, float dphi1_ref, float u_ref, float KP, float KD) {
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

