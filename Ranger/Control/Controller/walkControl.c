#include <mb_includes.h>
#include <mb_estimator.h>
#include <mb_controller.h>
#include <input_output.h>
#include <motorControl.h>
#include <walkControl.h>
#include "robotParameters.h"

typedef enum {
	Glide_Out,   // Outer feet on the ground, inner feet swing through with scissor gait
	Push_Out,    // Outer feet on the ground,
	Glide_Inn,  // Inner feet on the ground, inner feet swing through with scissor gait
	Push_Inn,
	Flight
} WalkFsmMode;

static WalkFsmMode WALK_FSM_MODE = Glide_Out;  // What to run now
static WalkFsmMode WALK_FSM_MODE_PREV = Glide_Out;  // What we ran last time

/* These are made available in the header file to use with unit testing */
float PHASE_HIP_ANGLE_START;    // The angle of the hip at the start of the glide phase
float PHASE_HIP_ANGLE_FINAL;    // The angle of the hip at the end of the glide phase
float PHASE_STANCE_ANGLE_START;    // The angle of the stance leg at the start of the glide phase
float PHASE_STANCE_ANGLE_FINAL;    // The angle of the stance leg at the end of the glide phase


/* This function should be called when the walking finite state machine
 * transitions into the Glide_Out mode. It sets the constants that determine
 * how the swing leg tracks the stance leg.    */
void setPhaseConstantsOuter(void) {
	PHASE_HIP_ANGLE_START = STATE_qh;
	PHASE_HIP_ANGLE_FINAL = LABVIEW_WALK_HIP_STEP_ANGLE;
	PHASE_STANCE_ANGLE_START = STATE_th0;
	PHASE_STANCE_ANGLE_FINAL = LABVIEW_WALK_CRIT_STANCE_ANGLE;
}

/* This function should be called when the walking finite state machine
 * transitions into the Glide_Inn mode. It sets the constants that determine
 * how the swing leg tracks the stance leg.    */
void setPhaseConstantsInner(void) {
	PHASE_HIP_ANGLE_START = STATE_qh;
	PHASE_HIP_ANGLE_FINAL = -LABVIEW_WALK_HIP_STEP_ANGLE;
	PHASE_STANCE_ANGLE_START = STATE_th1;
	PHASE_STANCE_ANGLE_FINAL = LABVIEW_WALK_CRIT_STANCE_ANGLE;
}

/* This function applies any reset maps that are necessary for transitions
 * into a given state of the finite state machine.   */
void applyResetMaps(void) {
	switch (WALK_FSM_MODE) {
	case Glide_Out:
		if (WALK_FSM_MODE != WALK_FSM_MODE_PREV) {
			setPhaseConstantsOuter();
		} break;
	case Glide_Inn:
		if (WALK_FSM_MODE != WALK_FSM_MODE_PREV) {
			setPhaseConstantsInner();
		} break;
	}
}

/* This function is called once per loop, and checks the
 * sensors that are used to trigger transitions between
 * modes of the finite state machine */
void updateWalkFsm(void) {
	WALK_FSM_MODE_PREV = WALK_FSM_MODE;

	if (STATE_contactMode == CONTACT_FL) { // Then robot is in the air
		WALK_FSM_MODE = Flight;  // Give up on walking and enter flight mode
	} else {  // Run the normal walking finite state machine
		switch (WALK_FSM_MODE_PREV) {
		case Glide_Out:
			if (STATE_th0 < LABVIEW_WALK_CRIT_STANCE_ANGLE) {
				WALK_FSM_MODE = Push_Out;
			} break;
		case Push_Out:
			if (STATE_c1) {  // If inner feet hit the ground
				WALK_FSM_MODE = Glide_Inn;
			} break;
		case Glide_Inn:
			if (STATE_th1 < LABVIEW_WALK_CRIT_STANCE_ANGLE) {
				WALK_FSM_MODE = Push_Inn;
			} break;
		case Push_Inn:
			if (STATE_c0) {  // If outer feet hit the ground
				WALK_FSM_MODE = Glide_Out;
			} break;
		case Flight:
			if (STATE_c0) {  // If outer feet hit the ground
				WALK_FSM_MODE = Glide_Out;
			} else if (STATE_c1) { // If inner feet hit the ground
				WALK_FSM_MODE = Glide_Inn;
			} break;
		}
	}
}


/* Turns on a specific led for each state of the walking FSM.
 * No LED indicates that the mode is flight. */
void setWalkFsmLed(void) {
	switch (WALK_FSM_MODE_PREV) {
	case Glide_Out:
		set_UI_LED(LED_WALK_FSM , 'r');
		break;
	case Push_Out:
		set_UI_LED(LED_WALK_FSM , 'o');
		break;
	case Glide_Inn:
		set_UI_LED(LED_WALK_FSM , 'b');
		break;
	case Push_Inn:
		set_UI_LED(LED_WALK_FSM , 'p');
		break;
	}
}


/* Sends commands to the motors based on the current state of the
 * walking finite state machine */
void sendMotorCommands(void) {

	/* Read controller parameters directly from labview for now */
	float push = LABVIEW_WALK_ANK_PUSH;  //magnitude of the push-off during walking, normalized to be on the range [0,1]
	float angle = LABVIEW_WALK_HIP_STEP_ANGLE ;  //Target angle for the hip to hold during push-off

	switch (WALK_FSM_MODE_PREV) {
	case Glide_Out:
		holdStance_ankOut();
		flipUp_ankInn();
		hipGlide();
		break;
	case Push_Out:
		flipDown_ankInn();
		pushOff_ankOut(push);
		hipHold(angle);
		break;
	case Glide_Inn:
		flipUp_ankOut();
		holdStance_ankInn();
		hipGlide();
		break;
	case Push_Inn:
		flipDown_ankOut();
		pushOff_ankInn(push);
		hipHold(angle);
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
	float holdLevelRel = PARAM_Phi - PARAM_ctrl_ank_holdLevel + STATE_th0;
	float target = push * (PARAM_ctrl_ank_pushTarget) + (1.0 - push) * holdLevelRel;
	trackRel_ankOut(target, LABVIEW_ANK_PUSH_KP, LABVIEW_ANK_PUSH_KD);
}
void pushOff_ankInn(float push) {
	float holdLevelRel = PARAM_Phi - PARAM_ctrl_ank_holdLevel + STATE_th1;
	float target = push * (PARAM_ctrl_ank_pushTarget) + (1.0 - push) * holdLevelRel;
	trackRel_ankInn(target, LABVIEW_ANK_PUSH_KP, LABVIEW_ANK_PUSH_KD);
}


/* Wrapper function.
 * Computes the scissor tracking gains such that the hip angle
 * tracks a linear function of the stance leg angle. The coefficients
 * are set by the boundary conditions:
 * Start: Hip angle measured at start of glide phase
 * FInal: Hip angle is at target angle when push-off begins */
void hipGlide(void) {
	float qStart = PHASE_HIP_ANGLE_START;  // Initial hip angle
	float qFinal = PHASE_HIP_ANGLE_FINAL;  // Final hip angle
	float thStart = PHASE_STANCE_ANGLE_START;  // Initial stance angle
	float thFinal = PHASE_STANCE_ANGLE_FINAL;   // Final stance angle
	float rate, offset; // Gains to pass through to scissor tracking.

	rate = (qFinal - qStart) / (thFinal - thStart);
	offset = qStart - rate * thStart;

	trackScissor_hip(rate, offset, LABVIEW_HIP_KP, LABVIEW_HIP_KD);
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
	updateWalkFsm();
	applyResetMaps();
	setWalkFsmLed();
	sendMotorCommands();
}

