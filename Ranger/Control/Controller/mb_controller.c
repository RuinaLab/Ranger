#include <mb_includes.h> 
#include <led_lcd_ouput.h>

/* parameters for the motors in Ranger */
static const float param_motor_R = 1.3; // (Ohms) terminal resistance
static const float param_motor_Vc = 0.7; // (Volts) contact voltage drop
static const float param_motor_K = 0.018; // (Nm/A) torque constant
static const float param_motor_c1 = 0.0; // (Nms/rad) viscous friction
static const float param_motor_c0 = 0.01; // (Nm) const friction
static const float param_motor_mu = 0.1; // (1) current-dep const friction
static const float param_motor_Jm = 1.6e-6; // (kg-m^2) motor inertia
static const float param_motor_G_hip = 66.0; // (1) gearbox ratio (66 = hip, 34 = ankle)
static const float param_motor_G_ank = 34.0; // (1) gearbox ratio (66 = hip, 34 = ankle)

/* joint limits */
static const float param_joint_ankle_flip = 0.15; // Hard stop at 0.0. Foot flips up to this angle to clear ground.
static const float param_joint_ankle_hold = 1.65; // Angle such that ankle joint is closest to ground when foot in contact.
static const float param_joint_ankle_push = 2.0; // Hard stop at 3.0. Foot pushes off to inject energy, this is maximum bound.

/* passive elements in joints */
static const float param_joint_ankle_spring = 14.0; //(Nm/rad) Spring constant for the ankle spring	(equilibrium: angle = param_joint_ankle_hold)
static const float param_joint_hip_spring = 7.6; // (Nm/rad) Spring constant for the hip spring	(equilibrium: angle=0)



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
 * 	@param omega = motor angular rate
 * 	@param torque = desired joint torque
 *  @param G = gearbox ratio
 *  
 *  --> Assume that motor is in steady-state (shaft acceleration == zero)
 */
float MotorModel_Current(float omega, float torque, float G) {
	// Tf = c1*omega + c0*sign(omega) + mu*G*K*abs(I)*sign(omega);	// Frictional torque
	// T = G*(K*I - Jm*G*alpha) - Tf; // Torque out of gear box:

	float direction = 0.0;  // Which direction is the motor spinning
	float num;  // current = num / den
	float denPos, denNeg;  // two possible values for den, solve by guess + check below
	float Ipos, Ineg; // two possible options for current, solve by guess + check below

	omega = omega*G; // convert motor shaft speed to joint speed

	////TODO////
	/* Replace the following the arctangent smoothing */
	if (omega > 0.0) {
		direction = 1.0;
	} else if (omega < 0.0) {
		direction = -1.0;
	}

	////BUG////
	/* The following line has a static friction term that does not make sense near the origin
	 * This is basically a result of trying to invert a vertical line. Replacing the direction 
	 * function with arctangent smoothing should help the problem */
	num = torque + param_motor_c1 * omega + param_motor_c0 * direction;


	/* Need to invert an abs() function... denPos assumes that the current is
	 * positive, denNeg does the opposite. Later we check the sign of the
	 * current to know which assumption was good.
	 */
    denPos = (G * param_motor_K - param_motor_mu * G * param_motor_K * direction);
    denNeg = (G * param_motor_K + param_motor_mu * G * param_motor_K * direction);

    Ipos = num / denPos; // %Current, assuming that current > 0
	Ineg = num / denNeg; // %Current, assuming that current <= 0

	if (Ipos > 0 && Ineg > 0) {
		return Ipos; // Then I > 0 assumption was correct!
	} else if (Ipos <= 0 && Ineg <= 0) {
		return Ineg;  // Then I <= 0 assumption was correct!
	} else {
		// Should never reach here
		return num / (G * param_motor_K); // Just drop the coulomb friction term
	}
}

/* This function calls the low-level hip controller. */
void controller_hip( struct ControllerData C ){
 	float omega; // ankle motor shaft speed
	float uSpring; // expected torque exerted by the hip spring
	float iRef;  //	feed-forward current to match desired torque
	float iTarget;  // feed-forward current with terms to match reference	
	omega = mb_io_get_float(ID_MCH_MOTOR_VELOCITY); 
	uSpring = -param_joint_hip_spring*mb_io_get_float(ID_MCH_ANGLE); // equilibrium at zero
	iRef = MotorModel_Current(omega, C.uRef-uSpring, param_motor_G_hip);  
	iTarget = iRef + C.kp*C.xRef + C.kd*C.vRef;	// Flip sign convention on ref angle  
	mb_io_set_float(ID_MCH_COMMAND_CURRENT, iTarget);	
	mb_io_set_float(ID_MCH_STIFFNESS, C.kp);
	mb_io_set_float(ID_MCH_DAMPNESS, C.kd);
}


/* This function calls the low-level ankle (outer) controller. */
void controller_ankleOuter( struct ControllerData C ){
 	float omega; // ankle motor shaft speed
	float iRef;  //	feed-forward current to match desired torque
	float iTarget;  // feed-forward current with terms to match reference	
	omega = mb_io_get_float(ID_MCFO_MOTOR_VELOCITY); 
	iRef = MotorModel_Current(omega, C.uRef, param_motor_G_ank);  
	iTarget = iRef + C.kp*C.xRef + C.kd*C.vRef;	// Flip sign convention on ref angle  
	mb_io_set_float(ID_MCFO_COMMAND_CURRENT, iTarget);	
	mb_io_set_float(ID_MCFO_STIFFNESS, C.kp);
	mb_io_set_float(ID_MCFO_DAMPNESS, C.kd);
}


/* This function calls the low-level ankle (inner) controller. */
void controller_ankleInner( struct ControllerData C ){
 	float omega; // ankle motor shaft speed
	float iRef;  //	feed-forward current to match desired torque
	float iTarget;  // feed-forward current with terms to match reference	
	omega = mb_io_get_float(ID_MCFI_MOTOR_VELOCITY); 
	iRef = MotorModel_Current(omega, C.uRef, param_motor_G_ank);  
	iTarget = iRef + C.kp*C.xRef + C.kd*C.vRef;	// Flip sign convention on ref angle  
	mb_io_set_float(ID_MCFI_COMMAND_CURRENT, iTarget);	
	mb_io_set_float(ID_MCFI_STIFFNESS, C.kp);
	mb_io_set_float(ID_MCFI_DAMPNESS, C.kd);
}


/*  ENTRY-POINT FUNCTION FOR ALL CONTROL CODE */
void mb_controller_update(void){

//test_led_lcd();  // Simple test of the LED and LCD outputs

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

} // mb_controller_update()
