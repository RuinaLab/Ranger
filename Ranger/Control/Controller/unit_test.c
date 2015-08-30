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


 /* Hang the robot from the ceiling. Send a desired hip
  * angle to the robot on ID_CTRL_TEST_R0, and the hip motor
  * will produce the torque to cancel out the effect of the
  * spring at that angle. */
 void test_hipCompensation_flight(){
 	disable_ankOut();
 	disable_ankInn();
 	trackRel_hip(mb_io_get_float(ID_CTRL_TEST_R0), 0.0, 0.0);  
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


/* Robot is standing in single stance on the inner feet.
 * The outer legs should track a slow sine curve, with 
 * minimal stead-state error thanks to gravity and spring
 * compensation. The target hip angle is on ID_CTRL_TEST_W0. */
void test_hipCompensation_inner(){
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

	trackRel_ankOut(0.2, kp_ank, kd_ank);
	trackAbs_ankInn(0.0, kp_ank, kd_ank);
	trackRel_hip(qTarget, kp_hip, kd_hip);
	mb_io_set_float(ID_CTRL_TEST_W0, qTarget);
}


/* Set the motor controllers to hold the robot in a good double-stance 
 * configuration. The user can then rock the robot up onto the inner
 * or outer feet to check that the contact sensors are working well */
 void test_doubleStanceContact(void){
 	float kp_hip = 14.0;
	float kd_hip = 2.0;
	float kp_ank = 7.0;
	float kd_ank = 1.0;

 	trackAbs_ankOut(0.0, kp_ank, kd_ank);
	trackAbs_ankInn(0.0, kp_ank, kd_ank);
	trackRel_hip(0.2, kp_hip, kd_hip);
 }


/* Gives the user direct control over motor command current
 * from LabVIEW, bypassing high-level motor control code.
 * Use Carefully!
 * ID_CTRL_TEST_R0 == outer ankle command current
 * ID_CTRL_TEST_R1 == inner ankle command current
 * ID_CTRL_TEST_R2 == hip joint command current          */
void debug_directCurrentControl(void){
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
	//test_hipCompensation_flight();  
	//test_hipCompensation_outer(); 
	//test_hipCompensation_inner();

	/***** Estimation ****/
	test_doubleStanceContact();

	/**** Debugging ****/
	//debug_directCurrentControl();
}

/***************************************************************
 *                        NOTES                                *
 ***************************************************************

--> All unit tests pass


--> Known Issue: The estimator initializes the stance leg angle 
to zero when the robot boots. This is really bad if you turn the 
robot on when it is balanced in double stance, with the feet spread
apart. It would be better to assume that the angle between the legs 
is vertical, since this will be true in double stance and when the
robot is hanging, or balancing with legs together.



 ***************************************************************/



