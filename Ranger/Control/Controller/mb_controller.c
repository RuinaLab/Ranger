#include <mb_includes.h> 
#include <led_lcd_ouput.h>

/* physical parameters for the motors in Ranger */



/* joint limits */
static const float param_joint_ankle_flip = 0.15; // Hard stop at 0.0. Foot flips up to this angle to clear ground.
static const float param_joint_ankle_hold = 1.65; // Angle such that ankle joint is closest to ground when foot in contact.
static const float param_joint_ankle_push = 2.0; // Hard stop at 3.0. Foot pushes off to inject energy, this is maximum bound.



/* standardized controller input struct */
struct ControllerData {
	float kp;	 	// proportional gain
	float kd;		// derivative gain
	float xRef; 	// reference joint angle
	float vRef;  	// reference joint angle rate
	float uRef;	// reference (nominal) torque required to achieve xRef and vRef							  
};


/* This function calls the low-level hip controller. */
void controller_hip( struct ControllerData C ){
 	float iTarget;	
	float iRef;
	iRef = C.uRef;  ////hack assume torque = current
	iTarget = iRef - C.kp*C.xRef - C.kd*C.vRef;
	mb_io_set_float(ID_MCH_COMMAND_CURRENT, iTarget);
	mb_io_set_float(ID_MCH_STIFFNESS, C.kp);
	mb_io_set_float(ID_MCH_DAMPNESS, C.kd);
}


/* This function calls the low-level ankle (outer) controller. */
void controller_ankleOuter( struct ControllerData C ){
 	float iTarget;	
	float iRef;
	iRef = C.uRef;  ////hack assume torque = current
	iTarget = iRef + C.kp*C.xRef + C.kd*C.vRef;	// Flip sign convention on ref angle  
	mb_io_set_float(ID_MCFO_COMMAND_CURRENT, iTarget);	
	mb_io_set_float(ID_MCFO_STIFFNESS, C.kp);
	mb_io_set_float(ID_MCFO_DAMPNESS, C.kd);
}


/* This function calls the low-level ankle (inner) controller. */
void controller_ankleInner( struct ControllerData C ){
 	float iTarget;	
	float iRef;
	iRef = C.uRef;  ////hack assume torque = current
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
