#include <mb_includes.h>
#include <led_lcd_ouput.h>

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

enum ControlMode {
	M0_StandBy,
	M1_Active
};

/* standardized controller input struct */
struct ControllerData {
	float kp;	 	// proportional gain
	float kd;		// derivative gain
	float xRef; 	// reference joint angle
	float vRef;  	// reference joint angle rate
	float uRef;	// reference (nominal) torque required to achieve xRef and vRef
};


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
void controller_hip( struct ControllerData C ) {
	float uSpring; // expected torque exerted by the hip spring
	float uControl; // torque due to PD controller
	float torque;  // feed-forward torque term
	float current;  // target motor current, based on motor model

	uControl =  C.kp * C.xRef + C.kd * C.vRef; // PD controller
	uSpring = -param_joint_hip_spring * C.xRef; // equilibrium at zero
	torque = C.uRef - uSpring + uControl;
	current = MotorModel_Current(torque, param_motor_G_hip);

	mb_io_set_float(ID_MCH_COMMAND_CURRENT, current);
	mb_io_set_float(ID_MCH_STIFFNESS, C.kp);
	mb_io_set_float(ID_MCH_DAMPNESS, C.kd);
}


/* This function calls the low-level ankle (outer) controller. */
void controller_ankleOuter( struct ControllerData C ) {
	float uSpring; // expected torque exerted by the ankle spring
	float uControl; // PD controller torque terms
	float torque;  // feed-forward torque term
	float current;  // target motor current, based on motor model

	uControl = C.kp * C.xRef + C.kd * C.vRef;
	uSpring = -param_joint_ankle_spring * (C.xRef - param_joint_ankle_ref); 
	torque = C.uRef - uSpring + uControl;
	current = MotorModel_Current(torque, param_motor_G_ank);

	mb_io_set_float(ID_MCFO_COMMAND_CURRENT, current);
	mb_io_set_float(ID_MCFO_STIFFNESS, C.kp);
	mb_io_set_float(ID_MCFO_DAMPNESS, C.kd);
}


/* This function calls the low-level ankle (inner) controller. */
void controller_ankleInner( struct ControllerData C ) {
	float uSpring; // expected torque exerted by the ankle spring
	float uControl; // PD controller torque terms
	float torque;  // feed-forward torque term
	float current;  // target motor current, based on motor model

	uControl = C.kp * C.xRef + C.kd * C.vRef;
	uSpring = -param_joint_ankle_spring * (C.xRef - param_joint_ankle_ref); 
	torque = C.uRef - uSpring + uControl;
	current = MotorModel_Current(torque, param_motor_G_ank);

	mb_io_set_float(ID_MCFI_COMMAND_CURRENT, current);
	mb_io_set_float(ID_MCFI_STIFFNESS, C.kp);
	mb_io_set_float(ID_MCFI_DAMPNESS, C.kd);
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
		controller_hip(ctrlHip);

	// Run a PD-controller on the outer foot angles:
		ctrlAnkOut.kp = mb_io_get_float(ID_CTRL_TEST_R5);
		ctrlAnkOut.kd = mb_io_get_float(ID_CTRL_TEST_R6);
		ctrlAnkOut.xRef = mb_io_get_float(ID_CTRL_TEST_R7);
		ctrlAnkOut.vRef = mb_io_get_float(ID_CTRL_TEST_R8);
		ctrlAnkOut.uRef = mb_io_get_float(ID_CTRL_TEST_R9);
		controller_ankleOuter(ctrlAnkOut);

	// Run a PD-controller on the inner foot angles:
		ctrlAnkInn.kp = mb_io_get_float(ID_CTRL_TEST_R5);
		ctrlAnkInn.kd = mb_io_get_float(ID_CTRL_TEST_R6);
		ctrlAnkInn.xRef = mb_io_get_float(ID_CTRL_TEST_R7);
		ctrlAnkInn.vRef = mb_io_get_float(ID_CTRL_TEST_R8);
		ctrlAnkInn.uRef = mb_io_get_float(ID_CTRL_TEST_R9);
		controller_ankleInner(ctrlAnkInn);
 }


/*  ENTRY-POINT FUNCTION FOR ALL CONTROL CODE */
void mb_controller_update(void) {

	// Enter stand-by mode when robot first boots
	static enum ControlMode controlMode = M0_StandBy;

	// Check UI buttons to update control mode
	if (detect_UI_button_input(1)) controlMode = M1_Active;
	if (detect_UI_button_input(0)) controlMode = M0_StandBy; // Stand-by always goes last (highest priority)

	// Run the desired control mode
	switch (controlMode) {
	case M0_StandBy:
		set_UI_LED(5, 'g');
		disable_motors();
		break;
	case M1_Active:
		set_UI_LED(5, 'b');
		test_motor_control() ;
		break;
	}

} // mb_controller_update()
