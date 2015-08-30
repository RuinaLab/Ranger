#include <mb_includes.h>
#include <unit_test.h>
#include <motorControl.h>
#include <RangerMath.h>


/*Simple function to return the robot time in seconds */
float getTime(){
return 0.001*mb_io_get_float(ID_TIMESTAMP);
}


/* Check that the wave functions in Ranger math are performing correctly.
 * Creates a sine wave, triangle wave, and a square wave, and then sends 
 * them over the CAN bus to LabVIEW, on channels:
 * 	ID_CTRL_TEST_W0, *_W1, *_W2 */
void test_waveFunctions(){
	float period = 1.0;
	float min = -0.5;
	float max = 2.2;
	float time = getTime();

	mb_io_set_float(ID_CTRL_TEST_W0, SquareWave(time,period,min,max));
	mb_io_set_float(ID_CTRL_TEST_W1, TriangleWave(time,period,min,max));
	mb_io_set_float(ID_CTRL_TEST_W2, SineWave(time,period,min,max));
}


/* Hold both ankles at zero absolute angle */
void test_trackAbs_ankle() {
	float kp = 7.0;
	float kd = 1.0;
	float phi = 0.0;  // Absolute tracking reference angle
	trackAbs_ankOut(phi, kp, kd);
	trackAbs_ankInn(phi, kp, kd);
	disable_hip();
}


/* Have the outer ankles track a square wave and the inner ankles track 
 * a triangle wave. The max and min values are set based on reasonable values
 * for flip-up and push-off target angles. The reference angles are sent out
 * on ID_CTRL_TEST_W0 and ID_CTRL_TEST_W1. */
 void test_trackRel_ankle(){
 	float kp = 7.0;
	float kd = 1.0;
	float max = 2.6; // Push-off
	float min = 0.2; // Flip-up
	float period = 2.0;
	float q0, q1;  // Target angles
	float time = getTime();

	q0 = SquareWave(time,period,min,max);
	q1 = TriangleWave(time,period,min,max);
	mb_io_set_float(ID_CTRL_TEST_W0, q0);
	mb_io_set_float(ID_CTRL_TEST_W1, q1); 

	trackRel_ankOut(q0,kp,kd);
	trackRel_ankInn(q1,kp,kd);
	disable_hip();
 }


 /* Hang the robot from the ceiling. The hip motor should 
  * attempt to cancel the effect of the hip spring. */
 void test_hipSpringCompensation(){
 	disable_ankOut();
 	disable_ankInn();
 	trackRel_hip(0.0, 0.0, 0.0); // 
 }


/* Robot is standing in single stance on the outer feet.
 * The inner legs should track a slow sine curve, with 
 * minimal stead-state error thanks to gravity and spring
 * compensation. The target hip angle is on ID_CTRL_TEST_W0. */
void test_hipCompensation_outer(){
	float kp_hip = 14.0;
	float kd_hip = 2.0;
	float kp_ank = 7.0;
	float kd_ank = 1.0;
	float min = -0.2;
	float max = 0.2;
	float period = 5.0;
	float qTarget;
	float time = getTime();

	qTarget = SineWave(time,period,min,max);

	trackAbs_ankOut(0.0, kp_ank, kd_ank);
	trackRel_ankInn(0.2, kp_ank, kd_ank);
	trackRel_hip(qTarget, kp_hip, kd_hip);
	mb_io_set_float(ID_CTRL_TEST_W0, qTarget);
}


/* Gives the user direct control over motor command current
 * from LabVIEW, bypassing high-level motor control code.
 * Use Carefully!
 * ID_CTRL_TEST_R0 == outer ankle command current
 * ID_CTRL_TEST_R1 == inner ankle command current
 * ID_CTRL_TEST_R2 == hip joint command current          */
void debug_directCurrentControl(){
	mb_io_set_float(ID_MCFO_COMMAND_CURRENT, mb_io_get_float(ID_CTRL_TEST_R0));
	mb_io_set_float(ID_MCFO_STIFFNESS, 0.0);
	mb_io_set_float(ID_MCFO_DAMPNESS, 0.0);
	mb_io_set_float(ID_MCFI_COMMAND_CURRENT, mb_io_get_float(ID_CTRL_TEST_R1));
	mb_io_set_float(ID_MCFI_STIFFNESS, 0.0);
	mb_io_set_float(ID_MCFI_DAMPNESS, 0.0);
	mb_io_set_float(ID_MCH_COMMAND_CURRENT, mb_io_get_float(ID_CTRL_TEST_R2));
	mb_io_set_float(ID_MCH_STIFFNESS, 0.0);
	mb_io_set_float(ID_MCH_DAMPNESS, 0.0);
}


/* Entry-point function for all unit tests */
void runUnitTest(void) {

	/***** Ranger Math ****/
	//test_waveFunctions();

	/***** Motor Control ****/
	//test_trackAbs_ankle();
	//test_trackRel_ankle();
	//test_hipSpringCompensation();  // --RE-CHECK--
	test_hipCompensation_outer();  // --FAILED--

	/**** Debugging ****/
	//debug_directCurrentControl();
}

/***************************************************************
 *                        NOTES                                *
 ***************************************************************

Known Bugs:

1) Something is wrong with the hip controller. The spring compensation
works fine when all inputs to the controller are set to zero, but it makes
things worse (or maybe it's gravity compensation?) when tracking.

2) When the robot turns on, the absolute tracking on the outer feet 
sometimes fails spectacularily. Calling calibrate by hitting button 0 
seems to fix the problem. Maybe it's something related to an uninitialized
variable?

 ***************************************************************/



