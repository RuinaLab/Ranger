#include <mb_includes.h> 
#include <led_lcd_ouput.h>

//struct controllerData {
//	float wn;	 	// natural frequency of controller (kp = m*wn^2)
//	float xi;		// damping ratio of the controller (kd = 2*m*xi*wn)
//	float xRef; 	// reference joint angle
//	float vRef;  	// reference joint angle rate
//	float uRef;	// reference (nominal) torque required to achieve xRef and vRef							  
//};

/*  This function is the entry-point for all controller stuff. It is called by the scheduler.
 */
void mb_controller_update(void){

//test_led_lcd();

// Run a PD-controller on the hip angle:
float kp;
float kd;
float xRef;  // Hip angle reference
float vRef;  // Hip angle rate reference
float iRef;	 // Reference motor current
float iTarget; 

kp = mb_io_get_float(ID_CTRL_TEST_R0);
kd = mb_io_get_float(ID_CTRL_TEST_R1);
xRef = mb_io_get_float(ID_CTRL_TEST_R2);
vRef = mb_io_get_float(ID_CTRL_TEST_R3);
iRef = mb_io_get_float(ID_CTRL_TEST_R4);

iTarget = iRef - kp*xRef - kd*vRef;
mb_io_set_float(ID_MCH_COMMAND_CURRENT, iTarget);
mb_io_set_float(ID_MCH_STIFFNESS, kp);
mb_io_set_float(ID_MCH_DAMPNESS, kd);

} // mb_controller_update()
