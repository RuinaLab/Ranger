#include <mb_includes.h>
#include "unit_test.h"
#include "motorControl.h"
#include "RangerMath.h"
#include "walkControl.h"
#include "PSO.h"
#include "objectiveFunction.h"
#include <optimizeGait.h>
#include "bufferDataOut.h"

/*Simple function to return the robot time in seconds */
float getTime() {
	return 0.001 * mb_io_get_float(ID_TIMESTAMP);
}


/* --1-- Check that the wave functions in Ranger math are performing correctly.
 * Creates a sine wave, triangle wave, and a square wave, and then sends
 * them over the CAN bus to LabVIEW, on channels:
 * 	ID_CTRL_TEST_W0, *_W1, *_W2 */
void test_waveFunctions() {
	float period = 2.0;
	float min = -0.5;
	float max = 2.2;
	float time = getTime();

	mb_io_set_float(ID_CTRL_TEST_W0, SquareWave(time, period, min, max));
	mb_io_set_float(ID_CTRL_TEST_W1, TriangleWave(time, period, min, max));
	mb_io_set_float(ID_CTRL_TEST_W2, SineWave(time, period, min, max));
	mb_io_set_float(ID_CTRL_TEST_W3, SawToothWave(time, period, min, max));
}


/* --2-- Hold both ankles at zero absolute angle */
void test_trackAbs_ankle() {
	float kp = 7.0;
	float kd = 1.0;
	float phi = 0.0;  // Absolute tracking reference angle
	trackAbs_ankOut(phi, kp, kd);
	trackAbs_ankInn(phi, kp, kd);
	disable_hip();
}


/* --3-- Have the outer ankles track a square wave and the inner ankles track
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


/* --4-- Hang the robot from the ceiling. Send a desired hip
 * angle to the robot on ID_CTRL_TEST_R0, and the hip motor
 * will produce the torque to cancel out the effect of the
 * spring at that angle. If the hip is set to track cancel the
 * measured state (not the target state), then you can use
 * R1 and R2 to send kp and kd gains respectively.
 *
 * --> In flight mode, gravity compensation is disabled, since
 * it is not clear where the reaction torque is coming from.   */
void test_hipCompensation_flight() {
	float target = mb_io_get_float(ID_CTRL_TEST_R0);
	float kp = mb_io_get_float(ID_CTRL_TEST_R1);
	float kd = mb_io_get_float(ID_CTRL_TEST_R2);
	disable_ankOut();
	disable_ankInn();
	trackRel_hip(target, kp, kd);
}


/* --5-- Robot is standing in single stance on the outer feet.
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

/* --6-- Robot is standing in single stance on the inner feet.
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


/* --7-- Robot is standing in single stance on the outer feet.
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



/* --8-- Robot is standing in single stance on the inner feet.
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

/* --9-- The robot holds the outer feet while the inner feet alternate
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

/* --10-- The robot holds the inner feet while the outer feet alternate
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

/* --11-- Runs hip-scissor tracking, but using gains and set-points
 * from labview and the constant parameters header file for walking */
void test_hipGlide_outer() {
	holdStance_ankOut();
	flipUp_ankInn();
	hipGlide();
}

/* --12-- Runs hip-scissor tracking, but using gains and set-points
 * from labview and the constant parameters header file for walking */
void test_hipGlide_inner() {
	holdStance_ankInn();
	flipUp_ankOut();
	hipGlide();
}


/* --13-- Put the robot in double stance, and set the ID_CTRL_TEST_R0 to be a
 * value between 0.0 (no push-off) and 1.0 (maximum push-off). The robot
 * will then alternate between push-off and stance hold on the inner feet,
 * waiting 4.0 seconds between each change. The outer feet remain in stance
 * hold the entire time, and the hip just (weakly) holds a constant angle */
void test_pushOff_outer(void) {
	float kp_hip = 10;  // weak gains on the hip
	float kd_hip = 0.5;
	float qh_hold = -0.3;  // hold this angle
	float period = 8.0; // 4 seconds push, 4 seconds hold
	float modeSignal;  // -1.0 for push, 1.0 for hold
	float push;  // Amplitude of the push-off feed-forward term, from labVIEW
	float time = getTime();

	push = mb_io_get_float(ID_CTRL_TEST_R0);
	modeSignal = SquareWave(time, period, -1.0, 1.0);
	if (modeSignal < 0.0) {
		pushOff_ankInn(push);
	} else {
		holdStance_ankInn();
	}

	holdStance_ankOut();
	trackRel_hip(qh_hold, kp_hip, kd_hip);
}


/* --14-- Put the robot in double stance, and set the ID_CTRL_TEST_R0 to be a
 * value between 0.0 (no push-off) and 1.0 (maximum push-off). The robot
 * will then alternate between push-off and stance hold on the outer feet,
 * waiting 4.0 seconds between each change. The inner feet remain in stance
 * hold the entire time, and the hip just (weakly) holds a constant angle */
void test_pushOff_inner(void) {
	float kp_hip = 10;  // weak gains on the hip
	float kd_hip = 0.5;
	float qh_hold = 0.3;  // hold this angle
	float period = 8.0; // 4 seconds push, 4 seconds hold
	float modeSignal;  // -1.0 for push, 1.0 for hold
	float push;  // Amplitude of the push-off feed-forward term, from labVIEW
	float time = getTime();

	push = mb_io_get_float(ID_CTRL_TEST_R0);
	modeSignal = SquareWave(time, period, -1.0, 1.0);
	if (modeSignal < 0.0) {
		pushOff_ankOut(push);
	} else {
		holdStance_ankOut();
	}

	holdStance_ankInn();
	trackRel_hip(qh_hold, kp_hip, kd_hip);
}


/* --15-- Holds both feet in stance. The behavior of the hip:
 * double stance = do nothing
 * flight = hold zero
 * single stance outer = hold pos
 * single stance inner = hold neg    */
void test_hipHold() {
	holdStance_ankOut();
	holdStance_ankInn();
	hipHold(0.3);
}


/* --17-- Set the motor controllers to hold the feet of the robot in the correct
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


/* --18-- Gives the user direct control over motor command current
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

/* --19-- Holds the outer feet in stance and the inner feet are
 * flipped up. The hip motor is disabled */
void debug_singleStanceOuter(void) {
	float kp_ank = 7.0;
	float kd_ank = 1.0;
	trackAbs_ankOut(0.0, kp_ank, kd_ank);
	trackRel_ankInn(0.2, kp_ank, kd_ank);
	disable_hip();
}

/* --20-- Lets the user directly control the inputs to the steering
 * motor controller at a low level */
void debug_steeringMotors(void) {
	mb_io_set_float(ID_MCSI_COMMAND_CURRENT, mb_io_get_float(ID_CTRL_TEST_R0));
	mb_io_set_float(ID_MCSI_STIFFNESS, mb_io_get_float(ID_CTRL_TEST_R1));
	mb_io_set_float(ID_MCSI_DAMPNESS, mb_io_get_float(ID_CTRL_TEST_R2));
}

/* --21-- Tests the variable-grid linear interpolation on Ranger by
 * sending a periodic waveform to be plotted in labview. W0 plots a
 * saw-tooth wave and w1 plots the test function. The result should be
 * a sine wave */
void test_linInterpVar(void) {
	static float T[] = {0.0, 0.1,  0.3,  0.4,  0.45, 0.5, 0.9,   0.98,  1.0};  // Time vector
	static float Y[] = {0.0, 0.59, 0.95, 0.59, 0.31, 0.0, -0.59, -0.13, 0.0};  // Function
	static int nGrid = 9;   // <-- BE VERY CAREFUL WITH THIS - don't want to get a segfault...

	float period = 2.0;
	float min = 0.0;
	float max = 1.0;
	float time = getTime();
	float timeWrap = SawToothWave(time, period, min, max);
	float func = LinInterpVar(timeWrap, T, Y, nGrid);

	mb_io_set_float(ID_CTRL_TEST_W0, timeWrap);
	mb_io_set_float(ID_CTRL_TEST_W1, func);
}


/* --22-- Tests the pseudo-random number sampler on Ranger
 * by simply writing random numbers to w0 as fast as possible */
void test_fastRand(void) {
	mb_io_set_float(ID_CTRL_TEST_W0, FastRand());
}


/* --23-- Tests particle swarm optimization by running on a N-dimensional
 * Elliptic bowl.
 * 	R0 = 0 to run optimization = 1 to reset optimization
 *  R1 = parameter omega
 * 	R2 = parameter alpha
 * 	R3 = parameter beta
 * 	W0 = Global best obj fun
 * 	W1 = Local best obj fun
 *  W2 = Local obj fun
 * 	W3 = Index of the currently selected particle  */
void test_pso_quadBowl(void) {

	static bool initComplete = false;

	bool flagRun = mb_io_get_float(ID_CTRL_TEST_R0) < 0.5;
	float omega = mb_io_get_float(ID_CTRL_TEST_R1);
	float alpha = mb_io_get_float(ID_CTRL_TEST_R2);
	float beta = mb_io_get_float(ID_CTRL_TEST_R3);

	/// Send problem statement on start-up
	if (!initComplete) {
		objFun_set_quadraticBowl();
		initComplete = true;
	}

	// Let user adjust parameters in optimization if desired
	if (omega > 0) {
		PSO_OMEGA = omega;
	}
	if (alpha > 0) {
		PSO_ALPHA = alpha;
	}
	if (beta > 0) {
		PSO_BETA = beta;
	}

	/// This is the key block - runs optimization
	if (flagRun) {
		pso_send_point();
		pso_eval_point();
	} else {
		psoReset();
	}

	mb_io_set_float(ID_CTRL_TEST_W0, psoGetGlobalBest());
	mb_io_set_float(ID_CTRL_TEST_W1, psoGetSelectBest());
	mb_io_set_float(ID_CTRL_TEST_W2, psoGetSelectObjVal());

}


/* --24-- Tests particle swarm optimization by running on a N-dimensional
 * Elliptic bowl. Puts a delay between query to the objective function
 * and the actual evaluation of the objective function.
 * 	R0 = 0 to run optimization = 1 to reset optimization
 *  R1 = parameter omega
 * 	R2 = parameter alpha
 * 	R3 = parameter beta
 * 	W0 = Global best obj fun
 * 	W1 = Local best obj fun
 *  W2 = Local obj fun
 * 	W3 = Index of the currently selected particle  */
void test_pso_quadBowlAsync(void) {

	static bool initComplete = false;
	static int callCounter = 0;

	bool flagRun = mb_io_get_float(ID_CTRL_TEST_R0) < 0.5;
	float omega = mb_io_get_float(ID_CTRL_TEST_R1);
	float alpha = mb_io_get_float(ID_CTRL_TEST_R2);
	float beta = mb_io_get_float(ID_CTRL_TEST_R3);

	/// Send problem statement on start-up
	if (!initComplete) {
		objFun_set_quadraticBowl();
		initComplete = true;
	}

	// Let user adjust parameters in optimization if desired
	if (omega > 0) {
		PSO_OMEGA = omega;
	}
	if (alpha > 0) {
		PSO_ALPHA = alpha;
	}
	if (beta > 0) {
		PSO_BETA = beta;
	}

	/// This is the key block - runs optimization
	if (flagRun) {
		if (callCounter == 0) {
			pso_send_point();   // This must always be called first!
			callCounter++;
		} else if (callCounter > 10) {
			// Simulate objective function that takes 10 clock cycles
			pso_eval_point();  // This must always be called second!
			callCounter = 0;
		} else {
			callCounter++;
		}
	} else {
		psoReset();
	}

	mb_io_set_float(ID_CTRL_TEST_W0, psoGetGlobalBest());
	mb_io_set_float(ID_CTRL_TEST_W1, psoGetSelectBest());
	mb_io_set_float(ID_CTRL_TEST_W2, psoGetSelectObjVal());

}



/* --25-- Tests particle swarm optimization by searching for
 * a controller that tracks a sine-wave with the robot's feet.
 * 	R0 = 0 to run optimization = 1 to reset optimization
 *  R1 = parameter omega
 * 	R2 = parameter alpha
 * 	R3 = parameter beta
 * 	W0 = Global best obj fun
 * 	W1 = Local best obj fun
 *  W2 = Local obj fun
 * 	W3 = Index of the currently selected particle  */
void test_pso_sineTrack(void) {

	float qRef;
	bool flagRun = mb_io_get_float(ID_CTRL_TEST_R0) < 0.5;
	float omega = mb_io_get_float(ID_CTRL_TEST_R1);
	float alpha = mb_io_get_float(ID_CTRL_TEST_R2);
	float beta = mb_io_get_float(ID_CTRL_TEST_R3);


	// Let user adjust parameters in optimization if desired
	if (omega > 0) {
		PSO_OMEGA = omega;
	}
	if (alpha > 0) {
		PSO_ALPHA = alpha;
	}
	if (beta > 0) {
		PSO_BETA = beta;
	}
	/// This is the key line:
	qRef = sineTrack_run();

	// Send info back through LabVIEW
	mb_io_set_float(ID_CTRL_TEST_W0, psoGetGlobalBest());
	mb_io_set_float(ID_CTRL_TEST_W1, psoGetSelectBest());
	mb_io_set_float(ID_CTRL_TEST_W2, psoGetSelectObjVal());
	mb_io_set_float(ID_CTRL_TEST_W3, qRef);

}


/* --26--  TO DO:  how to run test and what it should do if working properly */
void test_stepEventData(void) {
	static ContactMode lastMode = CONTACT_FL;

	/// Both feet held level, hip free.
	float kp_ank = 7.0;
	float kd_ank = 1.0;
	trackAbs_ankOut(0.0, kp_ank, kd_ank);
	trackAbs_ankInn(0.0, kp_ank, kd_ank);
	disable_hip();

	/// Figure out contact mode stuff
	if (lastMode == CONTACT_S0 || lastMode == CONTACT_S1) { // if we were in single stance
		if (STATE_contactMode != lastMode) { // and now are not
			triggerHeelStrikeUpdate();
		}
	}
	lastMode = STATE_contactMode;
}


/* --27-- Make the buzzer play a song on the UI board */
void test_uiPlaySong(void) {
	mb_io_set_float(ID_UI_SET_BUZZER_AMPL, 1); // Plays a song...
}


/* --28-- Force the robot to enter safe mode. This will exit debug mode,
 * but that is ok in this case. */
void test_safeMode(void) {
	enterSafeMode();
}

/* --29--  Tests pulse wave by using it to increment a counter which
 * is then published to labview. */
void test_pulseWave(void) {
	static int counter = 0;
	float period = 1.2;
	bool pulse = PulseWave(getTime(), period);
	if (pulse){
		counter++;
		mb_io_set_float(ID_CTRL_TEST_W0, counter);
	}
}


/* --30--  Test dumping a lot of data really fast. Will any get lost? Will it always come in order?
	Should send 1-10 in order to LabView 5 times. */
void test_sendFastData(void){ // UPDATE: NO, this is bad. Data does not get buffered. Just gets lost.
	int i;
	int j;

	for ( i = 0; i < 5; i++ ){
		for ( j = 0; j< 10; j++ ){
			mb_io_set_float(ID_CTRL_TEST_W0, (float)(j+1));
		}
	} 
}

/* --31--  Test dumping a lot of data really fast. Will any get lost? Will it always come in order?
	Should send 1-10 in order to LabView 5 times. This tests the bufferDataOut functions. */
void test_sendFastDataBuffered(void){
	static bool done = false;
	int i;
	int j;

	if (!done){ // Only send in one burst and then stop.
		for ( i = 0; i < 5; i++ ){
			for ( j = 0; j< 10; j++ ){

				addToQueue( ID_CTRL_TEST_W0, (float)(j+1) ); // Try 2 channels, see if the order is preserved.
				addToQueue( ID_CTRL_TEST_W1, (float)(j+1) );
			}
		} 
		done = true;
	}
}



/* --33--  Send the internal time over a test ID to labview. See if it matches the timestamp being sent already.
 * Are data timestamps with mb_io_send when it's sent or when it's called? */
void test_timestampSend(void){
	mb_io_set_float(ID_CTRL_TEST_W0, mb_io_get_float(ID_TIMESTAMP));
}

/* --34--  Make sure we understand the difference between MOTOR_CURRENT, COMMAND_CURRENT, and TARGET_CURRENT.
 * Theory: MOTOR_CURRENT is actually what's sensed at the motor. COMMAND_CURRENT is the feedforward term of
 * current control loop, and TARGET_CURRENT is the current command sent to the low level controller after 
 * feedback is added.
 * Test: Give COMMAND_CURRENT a value.. Alternate between 0 and nonzero gains. See what changes. 
 * Result: MOTOR_CURRENT seems to track COMMAND_CURRENT. TARGET_CURRENT doesn't seem to do anything. 
 * If TARGET_CURRENT is set directly, it affects neither of the other two. */

void test_motorCurrentCommandTypes(void){

	float period = 1.5;
	bool pulse = PulseWave(getTime(), period);

	mb_io_set_float(ID_MCFI_COMMAND_CURRENT, 0.01);
	// mb_io_set_float(ID_MCFI_MOTOR_TARGET_CURRENT, 0.05);
	if (pulse){

		mb_io_set_float(ID_MCFI_STIFFNESS, 1);
		mb_io_set_float(ID_MCFI_DAMPNESS, 1);
	}else{

		mb_io_set_float(ID_MCFI_STIFFNESS, 0.0);
		mb_io_set_float(ID_MCFI_DAMPNESS, 0.0);
	}

}


/* Entry-point function for all unit tests */
void runUnitTest(void) {

	int testId = (int) (mb_io_get_float(ID_CTRL_UNIT_TEST_ID));

	switch (testId) {   // Default (on initialization == 0 == do nothing)

	/**** Ranger Math ****/
	case 1: test_waveFunctions(); break;
	case 21: test_linInterpVar(); break;
	case 22: test_fastRand(); break;
	case 29: test_pulseWave(); break;

	/**** Low-Level Motor Control ****/
	case 2: test_trackAbs_ankle(); break;
	case 3: test_trackRel_ankle(); break;
	case 4: test_hipCompensation_flight(); break;
	case 5: test_hipCompensation_outer(); break;
	case 6: test_hipCompensation_inner(); break;
	case 7: test_hipScissorTrack_outer(); break;
	case 8: test_hipScissorTrack_inner(); break;
	case 34: test_motorCurrentCommandTypes(); break;

	/**** High-Level Motor Control ****/
	case 9: test_flipUpDownHold_outer(); break;
	case 10: test_flipUpDownHold_inner(); break;
	case 11: test_hipGlide_outer(); break;
	case 12: test_hipGlide_inner(); break;
	case 13: test_pushOff_outer(); break;
	case 14: test_pushOff_inner(); break;
	case 15: test_hipHold(); break;
	case 28: test_safeMode(); break;

	/**** Walking Controller ****/
	case 16: walkControl_test(); break; // Calls a function in walkControl.c

	/**** Estimation ****/
	case 17: test_doubleStanceContact();  break;
	case 26: test_stepEventData(); break;

	/**** Particle Swarm Optimization:  ****/
	case 23: test_pso_quadBowl(); break;
	case 24: test_pso_quadBowlAsync(); break;
	case 25: test_pso_sineTrack(); break;

	/**** Debugging ****/
	case 18: debug_directCurrentControl();  break;
	case 19: debug_singleStanceOuter();  break;
	case 20: debug_steeringMotors(); break;

	/**** UI Board ****/
	case 27: test_uiPlaySong(); break;

	/**** LabView IO ****/
	case 30: test_sendFastData(); break;
	case 31: test_sendFastDataBuffered(); break;
	case 33: test_timestampSend(); break;

	}
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

2) Sometimes when the robot boots there is a timing problem and the IMU
does not receive the command to start sending data. As a result the robot
orientation estimate goes crazy. We may have fixed this by increasing the
lenght of a wait on the UI board before it sends the command to the IMU to
start sending data.

 ***************************************************************/



