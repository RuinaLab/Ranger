#include <mb_includes.h> 
#include <motorControl.h>


/* parameters for the motors in Ranger */
// static const float param_motor_R = 1.3; // (Ohms) terminal resistance
// static const float param_motor_Vc = 0.7; // (Volts) contact voltage drop
static const float param_motor_K = 0.018; // (Nm/A) torque constant
// static const float param_motor_c1 = 0.0; // (Nms/rad) viscous friction
// static const float param_motor_c0 = 0.01; // (Nm) const friction
// static const float param_motor_mu = 0.1; // (1) current-dep const friction
// static const float param_motor_Jm = 1.6e-6; // (kg-m^2) motor inertia
static const float param_motor_G_hip = 66.0; // (1) gearbox ratio (66 = hip, 34 = ankle)
static const float param_motor_G_ank = 34.0; // (1) gearbox ratio (66 = hip, 34 = ankle)

/* joint limits */
static const float param_joint_ankle_flip = 0.15; // Hard stop at 0.0. Foot flips up to this angle to clear ground.
static const float param_joint_ankle_push = 2.0; // Hard stop at 3.0. Foot pushes off to inject energy, this is maximum bound.

/* passive elements in joints. Experimental data stored in:
 * templates/MotorModel/Test_StaticTorque.m
 */
static const float param_joint_ankle_ref = 1.662; // (rad) Foot angle corresponding to zero torque from spring. 
static const float param_joint_ankle_spring = 0.134; //(Nm/rad) Spring constant for the ankle spring	
static const float param_joint_hip_spring = 8.045; // (Nm/rad) Spring constant for the hip spring	(equilibrium: angle=0)

/*  MotorModel_Current computes the current that should be required to produce
 * 	the desired torque, given the state of the motor.
 * 	@param torque = desired joint torque
 *
 * --> Neglect friction since, terms are either zero (damping) or not easily
 *     invertable (coulomb friction)
 */
float MotorModel_Current(float torque, float G) {
	// Tf = c1*omega + c0*sign(omega) + mu*G*K*abs(I)*sign(omega);	// Frictional torque
	// T = G*(K*I - Jm*G*alpha) - Tf; // Torque out of gear box:
	//
	// --> T = G*K*I + [neglected terms]
	// --> I = T/(G*K)

	return torque / (G * param_motor_K);

}

/* This function calls the low-level hip controller. */
void controller_hip( struct ControllerData * C ) {
	float uSpring; // expected torque exerted by the hip spring
	float uControl; // torque due to PD controller
	float torque;  // feed-forward torque term
	float current;  // target motor current, based on motor model

	uControl =  C->kp * C->xRef + C->kd * C->vRef; // PD controller
	uSpring = -param_joint_hip_spring * C->xRef; // equilibrium at zero
	torque = C->uRef - uSpring + uControl;
	current = MotorModel_Current(torque, param_motor_G_hip);

	mb_io_set_float(ID_MCH_COMMAND_CURRENT, current);
	mb_io_set_float(ID_MCH_STIFFNESS, C->kp);
	mb_io_set_float(ID_MCH_DAMPNESS, C->kd);
}

/* Computes the current to send to the ankle controller 
 */
float getAnkleControllerCurrent( struct ControllerData * C ){
	float uSpring; // expected torque exerted by the ankle spring
	float uControl; // PD controller torque terms
	float torque;  // feed-forward torque term

	uControl = C->kp * C->xRef + C->kd * C->vRef;
	uSpring = -param_joint_ankle_spring * (C->xRef - param_joint_ankle_ref); 
	torque = C->uRef - uSpring + uControl;
	return MotorModel_Current(torque, param_motor_G_ank);  // Compute expected current
}

/* This function calls the low-level ankle (outer) controller. 
 */
void controller_ankleOuter( struct ControllerData * C ) {
	float current;
	current = getAnkleControllerCurrent(C);
	mb_io_set_float(ID_MCFO_COMMAND_CURRENT, current);
	mb_io_set_float(ID_MCFO_STIFFNESS, C->kp);
	mb_io_set_float(ID_MCFO_DAMPNESS, C->kd);
}


/* This function calls the low-level ankle (inner) controller. 
 */
void controller_ankleInner( struct ControllerData * C ) {
	float current;
	current = getAnkleControllerCurrent(C);
	mb_io_set_float(ID_MCFI_COMMAND_CURRENT, current);
	mb_io_set_float(ID_MCFI_STIFFNESS, C->kp);
	mb_io_set_float(ID_MCFI_DAMPNESS, C->kd);
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


#include "Trajectory.h"
#include "TrajData.h"
#include "math.h" //for fmod()
#include "RangerMath.h"	//for Sin()

#define DATA TRAJ_DATA_Test0
static const float leg_m = 2.5; //4.95
static const float leg_r = 0.15;
static const float g = 9.8;

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
		ctrlHip.kp = mb_io_get_float(ID_CTRL_TEST_R0);
		ctrlHip.kd = mb_io_get_float(ID_CTRL_TEST_R1);
		ctrlHip.xRef = y;
		ctrlHip.vRef = yd;
		ctrlHip.uRef = torque;
		controller_hip(&ctrlHip);

	// Run a PD-controller on the outer foot angles:
		ctrlAnkOut.kp = 0.0;
		ctrlAnkOut.kd = 0.0;
		ctrlAnkOut.xRef = 1.0;
		ctrlAnkOut.vRef = 0.0;
		ctrlAnkOut.uRef = 0.0;
		controller_ankleOuter(&ctrlAnkOut);

	// Run a PD-controller on the inner foot angles:
		ctrlAnkInn.kp = 0.0;
		ctrlAnkInn.kd = 0.0;
		ctrlAnkInn.xRef = 1.0;
		ctrlAnkInn.vRef = 0.0;
		ctrlAnkInn.uRef = 0.0;
		controller_ankleInner(&ctrlAnkInn);
 }

