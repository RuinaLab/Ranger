#include <mb_includes.h>
#include <mb_estimator.h>
#include <mb_controller.h>
#include <input_output.h>
#include <motorControl.h>
#include <walkControl.h>
#include <robotParameters.h>
#include <gaitControl.h>

typedef enum {
	Glide_Out,   // Outer feet on the ground, inner feet swing through with scissor gait
	Push1_Out,    // Outer feet on the ground,
	Push2_Out,
	Glide_Inn,  // Inner feet on the ground, inner feet swing through with scissor gait
	Push1_Inn,
	Push2_Inn,
	Flight
} WalkFsmMode;

/* Current and previous finite state machine modes. Initialized in
 * walkControl_entry()  */
static WalkFsmMode WALK_FSM_MODE;  // What to run now
static WalkFsmMode WALK_FSM_MODE_PREV;  // What we ran last time

/* The current set of gait parameters */
static float CtrlWalk_ankPush;
static float CtrlWalk_critAngle;
static float CtrlWalk_hipHold;
static float CtrlWalk_hipGain;
static float CtrlWalk_hipOffset;
static float CtrlWalk_pushTime;

/* Time since last state change: */
static float WalkFsm_switchTime;

/* This function is called once per loop, and checks the
 * sensors that are used to trigger transitions between
 * modes of the finite state machine */
void updateWalkFsm(void) {
	WALK_FSM_MODE_PREV = WALK_FSM_MODE;

	if (STATE_contactMode == CONTACT_FL) { // Then robot is in the air
		WALK_FSM_MODE = Flight;  // Give up on walking and enter flight mode
		WalkFsm_switchTime = STATE_t;
	} else {  // Run the normal walking finite state machine
		switch (WALK_FSM_MODE_PREV) {

		case Glide_Out:
			if (STATE_th0 < CtrlWalk_critAngle) {
				WALK_FSM_MODE = Push1_Out;
				WalkFsm_switchTime = STATE_t;
			} break;
		case Push1_Out:
			if (STATE_c1) {  // If inner feet hit the ground
				WALK_FSM_MODE = Push2_Out;
				WalkFsm_switchTime = STATE_t;
			} break;
		case Push2_Out:
			if (STATE_t - WalkFsm_switchTime > CtrlWalk_pushTime) {
				WALK_FSM_MODE = Glide_Inn;
				WalkFsm_switchTime = STATE_t;
			}

		case Glide_Inn:
			if (STATE_th1 < CtrlWalk_critAngle) {
				WALK_FSM_MODE = Push1_Inn;
				WalkFsm_switchTime = STATE_t;
			} break;
		case Push1_Inn:
			if (STATE_c0) {  // If outer feet hit the ground
				WALK_FSM_MODE = Push2_Inn;
				WalkFsm_switchTime = STATE_t;
			} break;
		case Push2_Inn:
			if  (STATE_t - WalkFsm_switchTime > CtrlWalk_pushTime) { // If outer feet hit the ground
				WALK_FSM_MODE = Glide_Out;
				WalkFsm_switchTime = STATE_t;
			} break;

		case Flight:
			if (STATE_c0) {  // If outer feet hit the ground
				WALK_FSM_MODE = Glide_Out;
				WalkFsm_switchTime = STATE_t;
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
 * and determines whether to read from LabVIEW or from the MDP
 * gait controller */
void readGaitData(void) {

	if (LABVIEW_GAIT_USE_MDP_DATA) {
		CtrlWalk_ankPush = GAIT_WALK_ANK_PUSH;
		CtrlWalk_critAngle = GAIT_WALK_CRIT_STANCE_ANGLE;
		CtrlWalk_hipHold = GAIT_WALK_HIP_STEP_ANGLE;
		CtrlWalk_hipGain = GAIT_WALK_SCISSOR_GAIN;
		CtrlWalk_hipOffset = GAIT_WALK_SCISSOR_OFFSET;
		CtrlWalk_pushTime = GAIT_WALK_PUSH_TIME;
	} else {
		CtrlWalk_ankPush = LABVIEW_WALK_ANK_PUSH;
		CtrlWalk_critAngle = LABVIEW_WALK_CRIT_STANCE_ANGLE;
		CtrlWalk_hipHold = LABVIEW_WALK_HIP_STEP_ANGLE;
		CtrlWalk_hipGain = LABVIEW_WALK_SCISSOR_GAIN;
		CtrlWalk_hipOffset = LABVIEW_WALK_SCISSOR_OFFSET;
		CtrlWalk_pushTime = LABVIEW_WALK_PUSH_TIME;
	}
}


/* Sends commands to the motors based on the current state of the
 * walking finite state machine */
void sendMotorCommands(void) {

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
		hipHold(CtrlWalk_hipHold);
		break;
	case Glide_Inn:
		flipUp_ankOut();
		holdStance_ankInn();
		hipGlide();
		break;
	case Push1_Inn:
	case Push2_Inn:
		flipDown_ankOut();
		pushOff_ankInn(push);
		hipHold(CtrlWalk_hipHold);
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
	float target = push * (PARAM_ctrl_ank_pushTarget) + (1.0 - push) * PARAM_ctrl_ank_holdLevel;
	trackAbs_ankOut(target, LABVIEW_ANK_PUSH_KP, LABVIEW_ANK_PUSH_KD);
}
void pushOff_ankInn(float push) {
	float target = push * (PARAM_ctrl_ank_pushTarget) + (1.0 - push) * PARAM_ctrl_ank_holdLevel;
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

