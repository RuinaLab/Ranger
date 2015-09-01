#include <mb_includes.h>
#include <unit_test.h>
#include <motorControl.h>
#include <RangerMath.h>
#include <walkControl.h>


/*Simple function to return the robot time in seconds */
float getTime() {
	return 0.001 * mb_io_get_float(ID_TIMESTAMP);
}


/* Check that the wave functions in Ranger math are performing correctly.
 * Creates a sine wave, triangle wave, and a square wave, and then sends
 * them over the CAN bus to LabVIEW, on channels:
 * 	ID_CTRL_TEST_W0, *_W1, *_W2 */
void test_waveFunctions() {
	float period = 1.0;
	float min = -0.5;
	float max = 2.2;
	float time = getTime();

	mb_io_set_float(ID_CTRL_TEST_W0, SquareWave(time, period, min, max));
	mb_io_set_float(ID_CTRL_TEST_W1, TriangleWave(time, period, min, max));
	mb_io_set_float(ID_CTRL_TEST_W2, SineWave(time, period, min, max));
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
void test_trackRel_ankle() {
	float kp = 7.0;
	float kd = 1.0;
	float max = 2.6; // Push-off
	float min = 0.2; // Flip-up
	float period = 2.0;
	float q0, q1;  // Target angles
	float time = getTime();

	q0 = SquareWave(time, period, min, max);
	q1 = TriangleWave(time, period, min, max);
	mb_io_set_float(ID_CTRL_TEST_W0, q0);
	mb_io_set_float(ID_CTRL_TEST_W1, q1);

	trackRel_ankOut(q0, kp, kd);
	trackRel_ankInn(q1, kp, kd);
	disable_hip();
}


/* Hang the robot from the ceiling. Send a desired hip
 * angle to the robot on ID_CTRL_TEST_R0, and the hip motor
 * will produce the torque to cancel out the effect of the
 * spring at that angle. */
void test_hipCompensation_flight() {
	disable_ankOut();
	disable_ankInn();
	trackRel_hip(mb_io_get_float(ID_CTRL_TEST_R0), 0.0, 0.0);
}


/* Robot is standing in single stance on the outer feet.
 * The inner legs should track a slow sine curve, with
 * minimal stead-state error thanks to gravity and spring
 * compensation. The target hip angle is on ID_CTRL_TEST_W0. */
void test_hipCompensation_outer() {
	float kp_hip = 14.0;
	float kd_hip = 2.0;
	float kp_ank = 7.0;
	float kd_ank = 1.0;
	float min = -0.2;
	float max = 0.2;
	float period = 5.0;
	float qTarget;
	float time = getTime();

	qTarget = SineWave(time, period, min, max);

	trackAbs_ankOut(0.0, kp_ank, kd_ank);
	trackRel_ankInn(0.2, kp_ank, kd_ank);
	trackRel_hip(qTarget, kp_hip, kd_hip);
	mb_io_set_float(ID_CTRL_TEST_W0, qTarget);
}

/* Robot is standing in single stance on the inner feet.
 * The outer legs should track a slow sine curve, with
 * minimal stead-state error thanks to gravity and spring
 * compensation. The target hip angle is on ID_CTRL_TEST_W0. */
void test_hipCompensation_inner() {
	float kp_hip = 14.0;
	float kd_hip = 2.0;
	float kp_ank = 7.0;
	float kd_ank = 1.0;
	float min = -0.2;
	float max = 0.2;
	float period = 5.0;
	float qTarget;
	float time = getTime();

	qTarget = SineWave(time, period, min, max);

	trackRel_ankOut(0.2, kp_ank, kd_ank);
	trackAbs_ankInn(0.0, kp_ank, kd_ank);
	trackRel_hip(qTarget, kp_hip, kd_hip);
	mb_io_set_float(ID_CTRL_TEST_W0, qTarget);
}


/* Robot is standing in single stance on the outer feet.
 * The inner legs should track a linear function of the outer
 * leg angle, with the constant offset being set by R0 and the
 * linear gain being set by R1. */
void test_hipScissorTrack_outer() {
	float kp_hip = 14.0;
	float kd_hip = 2.0;
	float kp_ank = 7.0;
	float kd_ank = 1.0;
	float offset, rate;

	offset = mb_io_get_float(ID_CTRL_TEST_R0);
	rate = mb_io_get_float(ID_CTRL_TEST_R1);

	trackAbs_ankOut(0.0, kp_ank, kd_ank);
	trackRel_ankInn(0.2, kp_ank, kd_ank);
	trackScissor_hip(rate, offset, kp_hip, kd_hip);
}



/* Robot is standing in single stance on the inner feet.
 * The outer legs should track a linear function of the inner
 * leg angle, with the constant offset being set by R0 and the
 * linear gain being set by R1. */
void test_hipScissorTrack_inner() {
	float kp_hip = 14.0;
	float kd_hip = 2.0;
	float kp_ank = 7.0;
	float kd_ank = 1.0;
	float offset, rate;

	offset = mb_io_get_float(ID_CTRL_TEST_R0);
	rate = mb_io_get_float(ID_CTRL_TEST_R1);

	trackRel_ankOut(0.2, kp_ank, kd_ank);
	trackAbs_ankInn(0.0, kp_ank, kd_ank);
	trackScissor_hip(rate, offset, kp_hip, kd_hip);
}

/* The robot holds the outer feet while the inner feet alternate
 * between flip-up and flip-down to hold. Used to test the basic
 * wrapper functions on the ankles */
void test_flipUpDownHold_outer(void) {
	float period = 2.0;
	float upDownTest;
	float time = getTime();
	upDownTest = SquareWave(time, period, -1.0, 1.0);
	if (upDownTest > 0.0) {
		flipUp_ankInn();
	} else {
		flipDown_ankInn();
	}
	disable_hip();
	holdStance_ankOut();
}

/* The robot holds the inner feet while the outer feet alternate
 * between flip-up and flip-down to hold. Used to test the basic
 * wrapper functions on the ankles */
void test_flipUpDownHold_inner(void) {
	float period = 2.0;
	float upDownTest;
	float time = getTime();
	upDownTest = SquareWave(time, period, -1.0, 1.0);
	if (upDownTest > 0.0) {
		flipUp_ankOut();
	} else {
		flipDown_ankOut();
	}
	disable_hip();
	holdStance_ankInn();
}

/* Runs hip-scissor tracking, but using gains and set-points
 * from labview and the constant parameters header file for walking */
void test_hipGlide_outer() {
	holdStance_ankOut();
	flipUp_ankInn();
	hipGlide(LABVIEW_WALK_HIP_RATE, LABVIEW_WALK_HIP_OFFSET);
}

/* Runs hip-scissor tracking, but using gains and set-points
 * from labview and the constant parameters header file for walking */
void test_hipGlide_inner() {
	holdStance_ankInn();
	flipUp_ankOut();
	hipGlide(LABVIEW_WALK_HIP_RATE, LABVIEW_WALK_HIP_OFFSET);
}


/* Put the robot in double stance, and set the ID_CTRL_TEST_R0 to be a 
 * value between 0.0 (no push-off) and 1.0 (maximum push-off). The robot 
 * will then alternate between push-off and stance hold on the inner feet,
 * waiting 4.0 seconds between each change. The outer feet remain in stance
 * hold the entire time, and the hip just (weakly) holds a constant angle */
 void test_pushOff_outer(void){
 	float kp_hip = 10;  // weak gains on the hip
 	float kd_hip = 0.5;
 	float qh_hold = -0.3;  // hold this angle
 	float period = 8.0; // 4 seconds push, 4 seconds hold
 	float modeSignal;  // -1.0 for push, 1.0 for hold
 	float push;  // Amplitude of the push-off feed-forward term, from labVIEW
 	float time = getTime();

 	push = mb_io_get_float(ID_CTRL_TEST_R0);
 	modeSignal = SquareWave(time,period,-1.0, 1.0);
 	if (modeSignal < 0.0){
 		pushOff_ankInn(push);
 	} else {
 		holdStance_ankInn();
 	}

 	holdStance_ankOut();
	trackRel_hip(qh_hold,kp_hip,kd_hip);
 }


/* Put the robot in double stance, and set the ID_CTRL_TEST_R0 to be a 
 * value between 0.0 (no push-off) and 1.0 (maximum push-off). The robot 
 * will then alternate between push-off and stance hold on the outer feet,
 * waiting 4.0 seconds between each change. The inner feet remain in stance
 * hold the entire time, and the hip just (weakly) holds a constant angle */
 void test_pushOff_inner(void){
 	float kp_hip = 10;  // weak gains on the hip
 	float kd_hip = 0.5;
 	float qh_hold = 0.3;  // hold this angle
 	float period = 8.0; // 4 seconds push, 4 seconds hold
 	float modeSignal;  // -1.0 for push, 1.0 for hold
 	float push;  // Amplitude of the push-off feed-forward term, from labVIEW
 	float time = getTime();

 	push = mb_io_get_float(ID_CTRL_TEST_R0);
 	modeSignal = SquareWave(time,period,-1.0, 1.0);
 	if (modeSignal < 0.0){
 		pushOff_ankOut(push);
 	} else {
 		holdStance_ankOut();
 	}

 	holdStance_ankInn();
	trackRel_hip(qh_hold,kp_hip,kd_hip);
 }


/* Holds both feet in stance. The behavior of the hip:
 * double stance = do nothing
 * flight = hold zero
 * single stance outer = hold pos
 * single stance inner = hold neg    */
 void test_hipHold(){
 	holdStance_ankOut();
 	holdStance_ankInn();
 	hipHold(0.3);
 }


/* Set the motor controllers to hold the feet of the robot in the correct
 * configuration for double stance standing, while turning off the hip motor.
 * The user can then rock the robot up onto the inner
 * or outer feet to check that the contact sensors are working well */
void test_doubleStanceContact(void) {
	float kp_ank = 7.0;
	float kd_ank = 1.0;

	trackAbs_ankOut(0.0, kp_ank, kd_ank);
	trackAbs_ankInn(0.0, kp_ank, kd_ank);
	disable_hip();
}


/* Gives the user direct control over motor command current
 * from LabVIEW, bypassing high-level motor control code.
 * Use Carefully!
 * ID_CTRL_TEST_R0 == outer ankle command current
 * ID_CTRL_TEST_R1 == inner ankle command current
 * ID_CTRL_TEST_R2 == hip joint command current          */
void debug_directCurrentControl(void) {
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

/* Holds the outer feet in stance and the inner feet are
 * flipped up. The hip motor is disabled */
void debug_singleStanceOuter(void) {
	float kp_ank = 7.0;
	float kd_ank = 1.0;
	trackAbs_ankOut(0.0, kp_ank, kd_ank);
	trackRel_ankInn(0.2, kp_ank, kd_ank);
	disable_hip();
}


/* Entry-point function for all unit tests */
void runUnitTest(void) {

	/**** Ranger Math ****/
	//test_waveFunctions();

	/**** Low-Level Motor Control ****/
	//test_trackAbs_ankle();
	//test_trackRel_ankle();
	//test_hipCompensation_flight();
	//test_hipCompensation_outer();
	//test_hipCompensation_inner();
	//test_hipScissorTrack_outer();
	//test_hipScissorTrack_inner();

	/**** High-Level Motor Control ****/
	//test_flipUpDownHold_outer();
	//test_flipUpDownHold_inner();
	//test_hipGlide_outer();
	//test_hipGlide_inner();
	//test_pushOff_outer();
	//test_pushOff_inner();
	//test_hipHold();

	/**** Walking Controller ****/
	//walkControl_test();

	/**** Estimation ****/
	//test_doubleStanceContact();

	/**** Debugging ****/
	//debug_directCurrentControl();
	//debug_singleStanceOuter();
}

/***************************************************************
 *                        NOTES                                *
 ***************************************************************


Debugging...


************************


Known Bugs:
1) If you send a single command to the motors, such as disable,
then the inner ankle motor sometimes does not receive the command.
This can be addressed by resending the command, but should be fixed 
eventually on the low-level control board.


 ***************************************************************/



