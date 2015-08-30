#include <mb_includes.h>
#include <motorControl.h>
#include "RangerMath.h"
#include "robotParameters.h"
#include "mb_estimator.h"

bool HIP_GRAVITY_COMPENSATION = true;
bool HIP_SPRING_COMPENSATION = true;

/* standardized controller input struct */
typedef struct  {
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
	float uRef = 0.0; // Nominal torque expected at joint before control
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
	float uRef = 0.0; // Nominal torque expected at joint before control
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
	float uRef = 0.0; // Nominal torque expected at joint before control
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
	ctrlAnkOut.xRef = q0;
	ctrlAnkOut.vRef = 0.0;
	ctrlAnkOut.kp = kp;
	ctrlAnkOut.kd = kd;
	run_controller_ankOut( &ctrlAnkOut );
}

/* Computes controller commands such that the inner ankle tracks
 * a constant joint angle (q1). */
void trackRel_ankInn(float q1, float kp, float kd) {
	ctrlAnkInn.xRef = q1;
	ctrlAnkInn.vRef = 0.0;
	ctrlAnkInn.kp = kp;
	ctrlAnkInn.kd = kd;
	run_controller_ankInn( &ctrlAnkInn );
}

/* Computes controller commands such that the hip angle tracks
 * a constant joint angle (qh). */
void trackRel_hip(float qh, float kp, float kd) {
	ctrlHip.xRef = qh;
	ctrlHip.vRef = 0.0;
	ctrlHip.kp = kp;
	ctrlHip.kd = kd;
	run_controller_hip(&ctrlHip);
}

/* Computes controller commands such that the outer feet track
 * a constant absolute orientation (phi0). */
void trackAbs_ankOut(float phi0, float kp, float kd) {
	ctrlAnkOut.xRef = PARAM_Phi - phi0 + STATE_th0;
	ctrlAnkOut.vRef = STATE_dth0;
	ctrlAnkOut.kp = kp;
	ctrlAnkOut.kd = kd;
	run_controller_ankOut( &ctrlAnkOut );
}

/* Computes controller commands such that the inner feet track
 * a constant absolute orientation (phi1). */
void trackAbs_ankInn(float phi1, float kp, float kd) {
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
		ctrlHip.kp = kp;
		ctrlHip.kd = kd;
		ctrlHip.xRef = offset - STATE_th0 * (rate + 1.0);
		ctrlHip.vRef = -STATE_dth0 * (rate + 1.0);
		break;
	case CONTACT_S1:
		ctrlHip.kp = kp;
		ctrlHip.kd = kd;
		ctrlHip.xRef = -offset + STATE_th1 * (rate + 1.0);
		ctrlHip.vRef = STATE_dth1 * (rate + 1.0);
		break;
	default:
		ctrlHip.kp = 0.0;
		ctrlHip.kd = 0.0;
		ctrlHip.xRef = STATE_qh;
		ctrlHip.vRef = STATE_dqh;
	}

	run_controller_hip(&ctrlHip);
}

