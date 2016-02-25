#include <mb_includes.h>
#include <mb_estimator.h>
#include <mb_controller.h>
#include <input_output.h>
#include <motorControl.h>
#include <walkControl.h>
#include <robotParameters.h>
#include <gaitControl.h>


/* Current and previous finite state machine modes. Initialized in
 * walkControl_entry()  */
WalkFsmMode WALK_FSM_MODE;  // What to run now
WalkFsmMode WALK_FSM_MODE_PREV;  // What we ran last time

/* The current set of gait parameters */
static float CtrlWalk_ankPush;
static float CtrlWalk_critStepLen;
static float CtrlWalk_hipGain;
static float CtrlWalk_hipOffset;
static float CtrlWalk_pushDelay;

/* Time since last state change: */
static float WalkFsm_switchTime;

/* Current Integral Stuff */
static float WalkFsm_ankOutPushInt = 0.0;
static float WalkFsm_ankInnPushInt = 0.0;

/* This function computes the distance between the ankle joints of the robot,
 * projected on to the ground */
float getAnkleJointRelDistance(float thStance, float thSwing) {
	return PARAM_l * (Sin(thSwing) - Sin(thStance));
}

/* This function computes the height between the ankle joints of the robot,
 * projected on to the ground */
float getAnkleJointRelHeight(float thStance, float thSwing) {
	return PARAM_l * (Cos(thStance) - Cos(thSwing));
}

/* Computes abs(targetHipAngle) for the hip, such that the swing ankle is virtually constrained
 * to move only in the vertical direction, with the distance locked at the target step length */
float getTargetHipAngle(float thStance, float thSwing, float stepLen) {
	float d = stepLen;   // target step length
	float l = PARAM_l;   // robot leg length
	float h = getAnkleJointRelHeight(thStance, thSwing);
	return Acos( 1 - ( d * d + h * h ) / ( 2 * l * l ) );
}

/* Accumulate (integrate) current signals to the motors for push-off
 * exit conditions */
void accumulatePushOffCurrent(void) {
	WalkFsm_ankOutPushInt += CLOCK_CYCLE_DURATION * Abs(mb_io_get_float(ID_MCFO_MOTOR_CURRENT));
	WalkFsm_ankInnPushInt += CLOCK_CYCLE_DURATION * Abs(mb_io_get_float(ID_MCFI_MOTOR_CURRENT));
	mb_io_set_float(ID_EST_OUT_PUSH_ACCUMULATED, WalkFsm_ankOutPushInt);
	mb_io_set_float(ID_EST_INN_PUSH_ACCUMULATED, WalkFsm_ankInnPushInt);
}

/* Reset the accumulators when the FSM changes */
void resetPushOffAccumulators(void) {
	WalkFsm_ankOutPushInt = 0.0;
	WalkFsm_ankInnPushInt = 0.0;
}

/* This function is called once per loop, and checks the
 * sensors that are used to trigger transitions between
 * modes of the finite state machine */
void updateWalkFsm(void) {
	float stepLen;
	WALK_FSM_MODE_PREV = WALK_FSM_MODE;
	accumulatePushOffCurrent();  // Compute the current integrals

	if (STATE_contactMode == CONTACT_FL) { // Then robot is in the air
		WALK_FSM_MODE = Flight;  // Give up on walking and enter flight mode
		WalkFsm_switchTime = STATE_t;
		resetPushOffAccumulators();
	} else {  // Run the normal walking finite state machine
		switch (WALK_FSM_MODE_PREV) {
		case Glide_Out:
			stepLen = getAnkleJointRelDistance(STATE_th0, STATE_th1);
			if (stepLen > CtrlWalk_critStepLen) {
				WALK_FSM_MODE = Push1_Out;
				WalkFsm_switchTime = STATE_t;
				resetPushOffAccumulators();
			} break;
		case Push1_Out:
			if (STATE_c1) {  // If inner feet hit the ground
				WALK_FSM_MODE = Push2_Out;
				WalkFsm_switchTime = STATE_t;
			} break;
		case Push2_Out:
			if (WalkFsm_ankOutPushInt > 0.6 || !STATE_c0) {   ////HACK////
				// if (STATE_t - WalkFsm_switchTime > CtrlWalk_pushDelay) {
				WALK_FSM_MODE = Glide_Inn;
				WalkFsm_switchTime = STATE_t;
				resetPushOffAccumulators();
			}

		case Glide_Inn:
			stepLen = getAnkleJointRelDistance(STATE_th1, STATE_th0);
			if (stepLen > CtrlWalk_critStepLen) {
				WALK_FSM_MODE = Push1_Inn;
				WalkFsm_switchTime = STATE_t;
				resetPushOffAccumulators();
			} break;
		case Push1_Inn:
			if (STATE_c0) {  // If outer feet hit the ground
				WALK_FSM_MODE = Push2_Inn;
				WalkFsm_switchTime = STATE_t;
			} break;
		case Push2_Inn:
			if (WalkFsm_ankInnPushInt > 0.6 || !STATE_c1) {  ////HACK////
				// if  (STATE_t - WalkFsm_switchTime > CtrlWalk_pushDelay) { // If outer feet hit the ground
				WALK_FSM_MODE = Glide_Out;
				WalkFsm_switchTime = STATE_t;
				resetPushOffAccumulators();
			} break;

		case Flight:
			if (STATE_c0) {  // If outer feet hit the ground
				WALK_FSM_MODE = Glide_Out;
				WalkFsm_switchTime = STATE_t;
				resetPushOffAccumulators();
			} break;
		}

	}
}


/* Turns on a specific led for each state of the walking FSM.
 * No LED indicates that the mode is flight. */
void setWalkFsmLed(void) {
	if (FSM_LED_FLAG) {
		switch (WALK_FSM_MODE_PREV) {
		case Glide_Out:
			set_UI_LED(LED_WALK_FSM , 'r');
			break;
		case Push1_Out:
		case Push2_Out:
			set_UI_LED(LED_WALK_FSM , 'o');
			break;
		case Glide_Inn:
			set_UI_LED(LED_WALK_FSM , 'b');
			break;
		case Push1_Inn:
		case Push2_Inn:
			set_UI_LED(LED_WALK_FSM , 'p');
			break;
		}
	}
}

/* Updates the data that are used by the walking controller,
 * and determines whether to read from LabVIEW or from the
 * gait controller */
void readGaitData(void) {

	if (LABVIEW_GAIT_USE_MDP_DATA) {
		CtrlWalk_ankPush = GAIT_WALK_ANK_PUSH;
		CtrlWalk_critStepLen = GAIT_WALK_CRIT_STEP_LENGTH;
		CtrlWalk_hipGain = GAIT_WALK_SCISSOR_GAIN;
		CtrlWalk_hipOffset = GAIT_WALK_SCISSOR_OFFSET;
		CtrlWalk_pushDelay = GAIT_WALK_DS_DELAY;
	} else {
		CtrlWalk_ankPush = LABVIEW_WALK_ANK_PUSH;
		CtrlWalk_critStepLen = LABVIEW_WALK_CRIT_STEP_LENGTH;
		CtrlWalk_hipGain = LABVIEW_WALK_SCISSOR_GAIN;
		CtrlWalk_hipOffset = LABVIEW_WALK_SCISSOR_OFFSET;
		CtrlWalk_pushDelay = LABVIEW_WALK_DS_DELAY;
	}
}


/* Sends commands to the motors based on the current state of the
 * walking finite state machine */
void sendMotorCommands(void) {
	float hipTargetAngle;
	float push = CtrlWalk_ankPush;

	switch (WALK_FSM_MODE_PREV) {
	case Glide_Out:
		holdStance_ankOut();
		flipUp_ankInn();
		hipGlide();
		break;
	case Push1_Out:
	case Push2_Out:
		flipDown_ankInn();
		pushOff_ankOut(push);
		hipTargetAngle = getTargetHipAngle(STATE_th0, STATE_th1, CtrlWalk_critStepLen);
		hipHold(hipTargetAngle);
		break;
	case Glide_Inn:
		holdStance_ankInn();
		flipUp_ankOut();
		hipGlide();
		break;
	case Push1_Inn:
	case Push2_Inn:
		flipDown_ankOut();
		pushOff_ankInn(push);
		hipTargetAngle = getTargetHipAngle(STATE_th1, STATE_th0, CtrlWalk_critStepLen);
		hipHold(hipTargetAngle);
		break;
	case Flight:  // In the air, get ready for Glide_Out
		holdStance_ankOut();
		flipUp_ankInn();
		disable_hip();
		break;
	}

}


/*******************************************************************************
 *                           Motor Control                                     *
 *******************************************************************************/

/* A simple wrapper function that forces the outer foot to track
 * the hold-level target. Designed to be called on the stance foot
 * during walking control. Uses gains from LabVIEW, and setpoints
 * from robotParameters.h */
void holdStance_ankOut(void) {
	trackAbs_ankOut(PARAM_ctrl_ank_holdLevel, LABVIEW_ANK_STANCE_KP, LABVIEW_ANK_STANCE_KD);
}
void holdStance_ankInn(void) {
	trackAbs_ankInn(PARAM_ctrl_ank_holdLevel, LABVIEW_ANK_STANCE_KP, LABVIEW_ANK_STANCE_KD);
}


/* Wrapper Function.
 * Flips the outer feet up and out of the way during swing phase */
void flipUp_ankOut(void) {
	trackRel_ankOut(PARAM_ctrl_ank_flipTarget, LABVIEW_ANK_SWING_KP, LABVIEW_ANK_SWING_KD);
}
void flipUp_ankInn(void) {
	trackRel_ankInn(PARAM_ctrl_ank_flipTarget, LABVIEW_ANK_SWING_KP, LABVIEW_ANK_SWING_KD);
}

/* Wrapper Functions.
 * Flips the outer feet down in preparation for heel-strike */
void flipDown_ankOut(void) {
	trackAbs_ankOut(PARAM_ctrl_ank_holdLevel, LABVIEW_ANK_SWING_KP, LABVIEW_ANK_SWING_KD);
}
void flipDown_ankInn(void) {
	trackAbs_ankInn(PARAM_ctrl_ank_holdLevel, LABVIEW_ANK_SWING_KP, LABVIEW_ANK_SWING_KD);
}

/* Wrapper Function.
 * Push-off with the outer ankles*/
void pushOff_ankOut(float push) {
	float target = push * (PARAM_ctrl_ank_pushTarget_0) + (1.0 - push) * PARAM_ctrl_ank_holdLevel;
	trackAbs_ankOut(target, LABVIEW_ANK_PUSH_KP, LABVIEW_ANK_PUSH_KD);
}
void pushOff_ankInn(float push) {
	float target = push * (PARAM_ctrl_ank_pushTarget_1) + (1.0 - push) * PARAM_ctrl_ank_holdLevel;
	trackAbs_ankInn(target, LABVIEW_ANK_PUSH_KP, LABVIEW_ANK_PUSH_KD);
}


/* Wrapper function.
 * Does scissor tracking on the hip, using labview gains.  */
void hipGlide(void) {
	trackScissor_hip(
	    CtrlWalk_hipGain, CtrlWalk_hipOffset,
	    LABVIEW_HIP_KP, LABVIEW_HIP_KD);
}


/* Wrapper function.
 * The hip holds various angles based on the contact configuration.
 * @param qh = magnitude (positive) of the hip angle during single stance.
 * double stance = do nothing
 * flight = hold zero
 * single stance outer = hold pos
 * single stance inner = hold neg    */
void hipHold(float qh) {
	switch (STATE_contactMode) {
	case CONTACT_S0:
		trackRel_hip(qh, LABVIEW_HIP_KP, LABVIEW_HIP_KD);
		break;
	case CONTACT_S1:
		trackRel_hip(-qh, LABVIEW_HIP_KP, LABVIEW_HIP_KD);
		break;
	case CONTACT_DS:
		disable_hip();
		break;
	case CONTACT_FL:
		trackRel_hip(0.0, LABVIEW_HIP_KP, LABVIEW_HIP_KD);
		break;
	}
}






/*******************************************************************************
 *                      TESTING FUNCTIONS                                      *
 *******************************************************************************/


/* Hold both feet in stance and disable the hip. Run the walking finite state
 * machine, and check transitions based on LED sequence.  */
void test_walkFsmTransitions(void) {
	updateWalkFsm();
	setWalkFsmLed();
	disable_hip();
	holdStance_ankOut();
	holdStance_ankInn();
}


/* This function is used for testing various aspects of the walking code,
 * designed to be called by unitTest     */
void walkControl_test(void) {

	/* Simple test for manually stepping through the finite state machine */
	test_walkFsmTransitions();
}



/*******************************************************************************
 *                     MAIN ENTRY-POINT FUNCTIONS                              *
 *******************************************************************************/

/* This function is called once, as soon as the button is pressed
 * for the robot to begin walking. It is used for initialization. */
void walkControl_entry(void) {

	disable_motors(); // Clear old motor commands;

	// Always start with the outer feet in stance, and the inner feet tracking a scissor gait
	WALK_FSM_MODE = Glide_Out;
	WALK_FSM_MODE_PREV = Glide_Out;

}


/* This is the main function for walking. All walking controls
 * are called from here */
void walkControl_main(void) {
	updateWalkFsm();   // Figures out the walking FSM mode
	setWalkFsmLed();   // Sets the LEDs for the user
	readGaitData();  // Read gait parameters from the high-level gait controller
	sendMotorCommands();  // Run the controller - send commands to motors
}

