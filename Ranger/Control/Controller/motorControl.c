#include <mb_includes.h>
#include <motorControl.h>
#include "RangerMath.h"
#include "robotParameters.h"
#include "mb_estimator.h"

float MOTOR_qh_trackErr;  // hip tracking error
float MOTOR_q0_trackErr;  // outer ankle tracking error
float MOTOR_q1_trackErr;  // inner ankle tracking error

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

/* Hip Compensation constants. Computed experimentally by
 *  data fitting on Feb 25, 2016
 *  torque = kSpring*(thSwing-thStance) + kGravity*sin(thSwing)  */
static float HIP_OUT_K_GRAVITY = 6.2700 * 1.1880;  // (current gain)*(motor constant)
static float HIP_OUT_K_SPRING = 5.5566 * 1.1880;   // (current gain)*(motor constant)
static float HIP_INN_K_GRAVITY = -5.5260 * 1.1880; // (current gain)*(motor constant)
static float HIP_INN_K_SPRING = -5.4468 * 1.1880;  // (current gain)*(motor constant)

/************************************************************************
 *                    Private Methods                                   *
 ************************************************************************/

/* Returns the torque needed to compensate for gravity and the hip spring.
 * Returns 0.0 if the robot is in double stance.
 * torque = kSpring*(thSwing-thStance) + kGravity*sin(thSwing) */
float hip_compensation(void) {
	switch (STATE_contactMode) {
	case CONTACT_S0:
		return  HIP_OUT_K_SPRING * (STATE_th1 - STATE_th0) + HIP_OUT_K_GRAVITY * Sin(STATE_th1);
	case CONTACT_S1:
		return  HIP_INN_K_SPRING * (STATE_th0 - STATE_th1) + HIP_INN_K_GRAVITY * Sin(STATE_th0);
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

	if (LABVIEW_HIP_COMPENSATION) {
		uRef = uRef + hip_compensation();
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

	MOTOR_qh_trackErr = xRef - STATE_qh;

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

	MOTOR_q0_trackErr = C->xRef - STATE_q0;
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

	MOTOR_q1_trackErr = C->xRef - STATE_q1;
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
void sendCurrent_ankInn(float current) {
	mb_io_set_float(ID_MCFI_COMMAND_CURRENT, current);
	mb_io_set_float(ID_MCFI_STIFFNESS, 0.0);
	mb_io_set_float(ID_MCFI_DAMPNESS, 0.0);
}

/* turn off the outer ankle motor */
void sendCurrent_ankOut(float current) {
	mb_io_set_float(ID_MCFO_COMMAND_CURRENT, current);
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

/* Computes controller commands such that the hip angle tracks
 * a constant joint angle (qh). */
void trackVel_hip(float qh, float dqh, float kp, float kd) {
	ctrlHip.uRef = 0.0;
	ctrlHip.xRef = qh;
	ctrlHip.vRef = dqh;
	ctrlHip.kp = kp;
	ctrlHip.kd = kd;
	run_controller_hip(&ctrlHip);
}

/* Computes controller commands such that the outer feet track
 * a constant absolute orientation (phi0). */
void trackAbs_ankOut(float phi0, float kp, float kd) {
	ctrlAnkOut.uRef = 0.0;
	ctrlAnkOut.xRef = PARAM_Phi0 - phi0 + STATE_th0;
	ctrlAnkOut.vRef = STATE_dth0;
	ctrlAnkOut.kp = kp;
	ctrlAnkOut.kd = kd;
	run_controller_ankOut( &ctrlAnkOut );
}

/* Computes controller commands such that the inner feet track
 * a constant absolute orientation (phi1). */
void trackAbs_ankInn(float phi1, float kp, float kd) {
	ctrlAnkInn.uRef = 0.0;
	ctrlAnkInn.xRef = PARAM_Phi1 - phi1 + STATE_th1;
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
