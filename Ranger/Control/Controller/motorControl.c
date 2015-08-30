#include <mb_includes.h>
#include <motorControl.h>
#include "RangerMath.h"
#include "robotParameters.h"
#include "mb_estimator.h"

bool HIP_GRAVITY_COMPENSATION = true;
bool HIP_SPRING_COMPENSATION = true;

/* standardized controller input struct */
typedef struct  {
	float uRef;   // feed-forward torque term
	float xRef; 	// reference joint angle
	float vRef;  	// reference joint angle rate
	float kp;	//stiffness, proportional gain, torque
	float kd;	//dampness, derivative gain, torque
} ControllerData;

static ControllerData ctrlHip;
static ControllerData ctrlAnkOut;
static ControllerData ctrlAnkInn;


/************************************************************************
 *                    Private Methods                                   *
 ************************************************************************/

/* Returns the torque needed to compensate for gravity at the target hip angle.
 * Returns 0.0 if the robot is in double stance. */
float hip_gravity_compensation(float qRefHip) {
	float uGravity = PARAM_m * PARAM_g * PARAM_c; // Torque scale factor
	float qRefSwing;

	switch (STATE_contactMode) {
	case CONTACT_S0:
		qRefSwing = qRefHip + STATE_th0;
		return  uGravity * Sin(qRefSwing);
	case CONTACT_S1:
		qRefSwing = STATE_th1 - qRefHip;
		return -uGravity * Sin(qRefSwing);
	case CONTACT_DS:
		return uGravity * Sin(0.5 * qRefHip);
	default:
		return 0.0;
	}
}


/* This function calls the low-level hip controller.
 * The spring compensation adds a torque equal and opposite
 * to what the spring would produce at the target hip angle */
void run_controller_hip( ControllerData * C ) {
	float uRef = C->uRef; // Nominal torque expected at joint before control
	float kp = C->kp;
	float kd = C->kd;
	float xRef = C->xRef;
	float vRef = C->vRef;

	if (LABVIEW_HIP_SPRING_COMPENSATION) {
		uRef = uRef  + xRef * PARAM_hip_spring_const;
	}

	if (LABVIEW_HIP_GRAVITY_COMPENSATION) {
		uRef = uRef + hip_gravity_compensation(xRef);
	}

	/* Combine all constant terms:                         *
	 * u = uRef + kp*(xRef - x) + kd*(vRef - v)            *
	 * u = (uRef + kp*xRef + kd*vRef) - (kp)*x - (kd)*v    */
	uRef = uRef + kp * xRef + kd * vRef;

	// Convert from torques to currents:
	uRef = uRef * PARAM_inv_hip_motor_const;
	kp = kp * PARAM_inv_hip_motor_const;
	kd = kd * PARAM_inv_hip_motor_const;

	mb_io_set_float(ID_MCH_COMMAND_CURRENT, uRef);
	mb_io_set_float(ID_MCH_STIFFNESS, kp);
	mb_io_set_float(ID_MCH_DAMPNESS, kd);
}


/* This function calls the low-level motor controller for the outer ankle. */
void run_controller_ankOut( ControllerData * C ) {
	float uRef = C->uRef; // Nominal torque expected at joint before control
	float kp = C->kp;
	float kd = C->kd;

	/* Combine all constant terms:                         *
	 * u = uRef + kp*(xRef - x) + kd*(vRef - v)            *
	 * u = (uRef + kp*xRef + kd*vRef) - (kp)*x - (kd)*v    */
	uRef = uRef + kp * C->xRef + kd * C->vRef;

	// Convert from torques to currents:
	uRef = uRef * PARAM_inv_ank_motor_const;
	kp = kp * PARAM_inv_ank_motor_const;
	kd = kd * PARAM_inv_ank_motor_const;

	mb_io_set_float(ID_MCFO_COMMAND_CURRENT, uRef);
	mb_io_set_float(ID_MCFO_STIFFNESS, kp);
	mb_io_set_float(ID_MCFO_DAMPNESS, kd);
}



/* This function calls the low-level motor controller for the inner ankle. */
void run_controller_ankInn( ControllerData * C ) {
	float uRef = C->uRef; // Nominal torque expected at joint before control
	float kp = C->kp;
	float kd = C->kd;

	/* Combine all constant terms:                         *
	 * u = uRef + kp*(xRef - x) + kd*(vRef - v)            *
	 * u = (uRef + kp*xRef + kd*vRef) - (kp)*x - (kd)*v    */
	uRef = uRef + kp * C->xRef + kd * C->vRef;

	// Convert from torques to currents:
	uRef = uRef * PARAM_inv_ank_motor_const;
	kp = kp * PARAM_inv_ank_motor_const;
	kd = kd * PARAM_inv_ank_motor_const;

	mb_io_set_float(ID_MCFI_COMMAND_CURRENT, uRef);
	mb_io_set_float(ID_MCFI_STIFFNESS, kp);
	mb_io_set_float(ID_MCFI_DAMPNESS, kd);
}


/************************************************************************
 *                    Public Methods                                    *
 ************************************************************************/


/* Turns off motors. */
void disable_motors() {
	disable_ankInn();
	disable_ankOut();
	disable_hip();
}

/* Turn off the hip motor */
void disable_hip() {
	mb_io_set_float(ID_MCH_COMMAND_CURRENT, 0.0);
	mb_io_set_float(ID_MCH_STIFFNESS, 0.0);
	mb_io_set_float(ID_MCH_DAMPNESS, 0.0);
}

/* turn off the outer ankle motor */
void disable_ankOut() {
	mb_io_set_float(ID_MCFO_COMMAND_CURRENT, 0.0);
	mb_io_set_float(ID_MCFO_STIFFNESS, 0.0);
	mb_io_set_float(ID_MCFO_DAMPNESS, 0.0);
}

/* turn off the inner ankle motor */
void disable_ankInn() {
	mb_io_set_float(ID_MCFI_COMMAND_CURRENT, 0.0);
	mb_io_set_float(ID_MCFI_STIFFNESS, 0.0);
	mb_io_set_float(ID_MCFI_DAMPNESS, 0.0);
}

/* Computes controller commands such that the outer ankle tracks
 * a constant joint angle (q0). */
void trackRel_ankOut(float q0, float kp, float kd) {
	ctrlAnkOut.uRef = 0.0;
	ctrlAnkOut.xRef = q0;
	ctrlAnkOut.vRef = 0.0;
	ctrlAnkOut.kp = kp;
	ctrlAnkOut.kd = kd;
	run_controller_ankOut( &ctrlAnkOut );
}

/* Computes controller commands such that the inner ankle tracks
 * a constant joint angle (q1). */
void trackRel_ankInn(float q1, float kp, float kd) {
	ctrlAnkInn.uRef = 0.0;
	ctrlAnkInn.xRef = q1;
	ctrlAnkInn.vRef = 0.0;
	ctrlAnkInn.kp = kp;
	ctrlAnkInn.kd = kd;
	run_controller_ankInn( &ctrlAnkInn );
}

/* Computes controller commands such that the hip angle tracks
 * a constant joint angle (qh). */
void trackRel_hip(float qh, float kp, float kd) {
	ctrlHip.uRef = 0.0;
	ctrlHip.xRef = qh;
	ctrlHip.vRef = 0.0;
	ctrlHip.kp = kp;
	ctrlHip.kd = kd;
	run_controller_hip(&ctrlHip);
}

/* Computes controller commands such that the outer feet track
 * a constant absolute orientation (phi0). */
void trackAbs_ankOut(float phi0, float kp, float kd) {
	ctrlAnkOut.uRef = 0.0;
	ctrlAnkOut.xRef = PARAM_Phi - phi0 + STATE_th0;
	ctrlAnkOut.vRef = STATE_dth0;
	ctrlAnkOut.kp = kp;
	ctrlAnkOut.kd = kd;
	run_controller_ankOut( &ctrlAnkOut );
}

/* Computes controller commands such that the inner feet track
 * a constant absolute orientation (phi1). */
void trackAbs_ankInn(float phi1, float kp, float kd) {
	ctrlAnkInn.uRef = 0.0;
	ctrlAnkInn.xRef = PARAM_Phi - phi1 + STATE_th1;
	ctrlAnkInn.vRef = STATE_dth1;
	ctrlAnkInn.kp = kp;
	ctrlAnkInn.kd = kd;
	run_controller_ankInn( &ctrlAnkInn );
}


/* Computes the controller commands such that the swing leg tracks a
 * linear function of the stance leg angle. Disables the hip motor
 * when in double stance or flight.
 * swingAngle -> -rate*stanceAngle + offset
 * positive is defined to be the swing leg in front of the stance leg */
void trackScissor_hip(float rate, float offset, float kp, float kd) {

	ctrlHip.kp = kp;
	ctrlHip.kd = kd;

	switch (STATE_contactMode) {
	case CONTACT_S0:
		ctrlHip.uRef = 0.0;
		ctrlHip.kp = kp;
		ctrlHip.kd = kd;
		ctrlHip.xRef = offset - STATE_th0 * (rate + 1.0);
		ctrlHip.vRef = -STATE_dth0 * (rate + 1.0);
		break;
	case CONTACT_S1:
		ctrlHip.uRef = 0.0;
		ctrlHip.kp = kp;
		ctrlHip.kd = kd;
		ctrlHip.xRef = -offset + STATE_th1 * (rate + 1.0);
		ctrlHip.vRef = STATE_dth1 * (rate + 1.0);
		break;
	default:
		ctrlHip.uRef = 0.0;
		ctrlHip.kp = 0.0;
		ctrlHip.kd = 0.0;
		ctrlHip.xRef = STATE_qh;
		ctrlHip.vRef = STATE_dqh;
	}

	run_controller_hip(&ctrlHip);
}



/* A simple wrapper function that forces the outer foot to track
 * the hold-level target. Designed to be called on the stance foot
 * during walking control. Uses gains from LabVIEW, and setpoints
 * from robotParameters.h */
void holdStance_ankOut(void) {
	trackAbs_ankOut(PARAM_ctrl_ank_holdLevel, LABVIEW_ANK_STANCE_KP, LABVIEW_ANK_STANCE_KD);
}
void holdStance_ankInn(void) {
	trackAbs_ankInn(PARAM_ctrl_ank_holdLevel, LABVIEW_ANK_STANCE_KP, LABVIEW_ANK_STANCE_KD);
}


/* Wrapper Function.
 * Flips the outer feet up and out of the way during swing phase */
void flipUp_ankOut(void) {
	trackRel_ankOut(PARAM_ctrl_ank_flipTarget, LABVIEW_ANK_SWING_KP, LABVIEW_ANK_SWING_KD);
}
void flipUp_ankInn(void) {
	trackRel_ankInn(PARAM_ctrl_ank_flipTarget, LABVIEW_ANK_SWING_KP, LABVIEW_ANK_SWING_KD);
}


/* Wrapper Functions.
 * Flips the outer feet down in preparation for heel-strike */
void flipDown_ankOut(void) {
	trackAbs_ankOut(PARAM_ctrl_ank_holdLevel, LABVIEW_ANK_SWING_KP, LABVIEW_ANK_SWING_KD);
}
void flipDown_ankInn(void) {
	trackAbs_ankInn(PARAM_ctrl_ank_holdLevel, LABVIEW_ANK_SWING_KP, LABVIEW_ANK_SWING_KD);
}

/* Push off the stance feet, using an open-loop current in addition
 * to the feed-back position control.
 * @param push = [0,1] = [none, max] feed-forward current */
void pushOff_ankOut(float push) {
	float qStart;  // Relative joint angle if the absolute orientation of the foot were level.
	float qFinal;  // Relative joint angle when the foot reaches maximum extension
	float phase;   // progress through push-off cycle (0 = start, 1 = at target)
	float torque;  // feed-forward current to send to motor
	qStart = PARAM_Phi - PARAM_ctrl_ank_holdLevel + STATE_th0;
	qFinal = PARAM_ctrl_ank_pushTarget;

	// Calculate the feed-forward current signal
	phase = (qFinal - STATE_q0) / (qFinal - qStart); // (0 = start, 1 = at target)
	phase = Clamp(phase, 0.0, 1.0);
	push = Clamp(push, 0.0, 1.0);
	torque = push * phase * PARAM_ctrl_ank_torqueScale; // Positive torque will extend foot

	// Send the feed-back commands
	ctrlAnkOut.uRef = torque;
	ctrlAnkOut.xRef = PARAM_ctrl_ank_pushTarget;
	ctrlAnkOut.vRef = 0.0;
	ctrlAnkOut.kp = LABVIEW_ANK_STANCE_KP;
	ctrlAnkOut.kd = LABVIEW_ANK_STANCE_KD;
	run_controller_ankOut( &ctrlAnkOut );
}

/* Same as pushOff_ankOut, but with the inner feed instead. */
void pushOff_ankInn(float push) {
	float qStart;  // Relative joint angle if the absolute orientation of the foot were level.
	float qFinal;  // Relative joint angle when the foot reaches maximum extension
	float phase;   // progress through push-off cycle (0 = start, 1 = at target)
	float torque;  // feed-forward current to send to motor
	qStart = PARAM_Phi - PARAM_ctrl_ank_holdLevel + STATE_th1;
	qFinal = PARAM_ctrl_ank_pushTarget;

	// Calculate the feed-forward current signal
	phase = (qFinal - STATE_q1) / (qFinal - qStart); // (0 = start, 1 = at target)
	phase = Clamp(phase, 0.0, 1.0);
	push = Clamp(push, 0.0, 1.0);
	torque = push * phase * PARAM_ctrl_ank_torqueScale; // Positive torque will extend foot

	// Send the feed-back commands
	ctrlAnkInn.uRef = torque;
	ctrlAnkInn.xRef = PARAM_ctrl_ank_pushTarget;
	ctrlAnkInn.vRef = 0.0;
	ctrlAnkInn.kp = LABVIEW_ANK_STANCE_KP;
	ctrlAnkInn.kd = LABVIEW_ANK_STANCE_KD;
	run_controller_ankInn( &ctrlAnkInn );
}


/* Wrapper function.
 * Hip does scissor tracking with gains from LabVIEW */
void hipGlide(float rate, float offset){
	trackScissor_hip(rate, offset, LABVIEW_HIP_KP, LABVIEW_HIP_KD);
}

/* Wrapper function.
 * hip holds a fixed angle using gains from LabVIEW */
void hipHold(float qh){
	trackRel_hip(qh, LABVIEW_HIP_KP, LABVIEW_HIP_KD);
}
