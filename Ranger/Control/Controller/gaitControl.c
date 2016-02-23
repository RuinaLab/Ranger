#include <mb_includes.h>
#include <mb_estimator.h>
#include <mb_controller.h>
#include <input_output.h>
#include <gaitControl.h>
#include <gaitControlData.h>
#include <optimizeGait.h>

typedef enum {
	PreMid_Out,   // Outer feet on the ground, inner feet swing through with scissor gait
	PostMid_Out,    // Outer feet on the ground,
	PreMid_Inn,  // Inner feet on the ground, inner feet swing through with scissor gait
	PostMid_Inn
} GaitFsmMode;

static GaitFsmMode GAIT_FSM_MODE = PreMid_Out;

static bool flagFirstStep = true;  // True if the robot is taking its first step

/* Parameters that are updated once per step and read by the estimator,
 * which then passes them to the walking controller. They are initialized
 * in gaitControl_entry() */
float GAIT_WALK_ANK_PUSH;
float GAIT_WALK_CRIT_STEP_LENGTH;
float GAIT_WALK_SCISSOR_GAIN;
float GAIT_WALK_SCISSOR_OFFSET;
float GAIT_WALK_DS_DELAY;
float GAIT_WALK_IDX;   // used for debugging - says which gait parameters are being used

/* This function computes the average walking speed, using the step length and the time since
 * the last mid-stance. We assume that this is called at each mid-stance!! */
float getMeanWalkSpeed(void) {
	static float lastCallTime = 0.0;
	float stepDuration = STATE_t - lastCallTime;
	float speed = STATE_lastStepLength / stepDuration;
	if (flagFirstStep) {
		return 0.0;
	} else {
		lastCallTime = STATE_t;
		return speed;
	}
}

/* This function is called once per walking step at mid-stance
 * It computes the new set of gait data that is used by the
 * walking controller  */
void updateGaitData() {
	float speed = getMeanWalkSpeed();
	// Linear interpolation over data
	GAIT_WALK_ANK_PUSH = LinInterpVar(speed, GAITDATA_SPEED_KNOT_POINTS, GAITDATA_WALK_ANK_PUSH, GAITDATA_NGRID);
	GAIT_WALK_CRIT_STEP_LENGTH = LinInterpVar(speed, GAITDATA_SPEED_KNOT_POINTS, GAITDATA_WALK_CRIT_STEP_LENGTH, GAITDATA_NGRID);
	GAIT_WALK_SCISSOR_GAIN = LinInterpVar(speed, GAITDATA_SPEED_KNOT_POINTS, GAITDATA_WALK_SCISSOR_GAIN, GAITDATA_NGRID);
	GAIT_WALK_SCISSOR_OFFSET = LinInterpVar(speed, GAITDATA_SPEED_KNOT_POINTS, GAITDATA_WALK_SCISSOR_OFFSET, GAITDATA_NGRID);
	GAIT_WALK_DS_DELAY = LinInterpVar(speed, GAITDATA_SPEED_KNOT_POINTS, GAITDATA_WALK_DS_DELAY, GAITDATA_NGRID);
	flagFirstStep = false;
}


/* This function is called once per loop, and checks the
 * sensors that are used to trigger transitions between
 * modes of the finite state machine */
void updateGaitFsm(void) {

	if (STATE_contactMode == CONTACT_FL) { // Then robot is in the air
		GAIT_FSM_MODE = PreMid_Out;  // Reset the finite state machine to initial state
		flagFirstStep = true;
	} else {  // Run the normal mid-stance finite state machine
		switch (GAIT_FSM_MODE) {
		case PreMid_Out:
			if (STATE_th0 < 0.0) {
				GAIT_FSM_MODE = PostMid_Out;
				updateGaitData();      // Update the controller at mid-stance on the outer legs
			}
			break;
		case PostMid_Out:
			if (STATE_c1) { // Inner feet hit ground
				triggerHeelStrikeUpdate();  // Tell estimtor that we've reached heel-strike
				GAIT_FSM_MODE = PreMid_Inn;
			}
			break;
		case PreMid_Inn:
			if (STATE_th1 < 0.0) {
				GAIT_FSM_MODE = PostMid_Inn;
				updateGaitData();      // Update the controller at mid-stance on the outer legs
			}
			break;
		case PostMid_Inn:
			if (STATE_c0) { // Outer feet hit ground
				triggerHeelStrikeUpdate();  // Tell estimtor that we've reached heel-strike
				GAIT_FSM_MODE = PreMid_Out;
			}
			break;
		}
	}
}


/* Turns on a specific led for each state of the walking FSM.
 * No LED indicates that the mode is flight. */
void setGaitFsmLed(void) {
	if (FSM_LED_FLAG) {
		if (STATE_contactMode != CONTACT_FL) { // Only update lights when feet on ground
			switch (GAIT_FSM_MODE) {
			case PreMid_Out:
				set_UI_LED(LED_GAIT_FSM, 'r');
				break;
			case PostMid_Out:
				set_UI_LED(LED_GAIT_FSM, 'o');
				break;
			case PreMid_Inn:
				set_UI_LED(LED_GAIT_FSM, 'b');
				break;
			case PostMid_Inn:
				set_UI_LED(LED_GAIT_FSM, 'p');
				break;
			}
		}
	}
}


/*******************************************************************************
 *                     MAIN ENTRY-POINT FUNCTIONS                              *
 *******************************************************************************/

/* This function is called once, as soon as the button is pressed
 * for the robot to begin walking. It is used for initialization. */
void gaitControl_entry(void) {

	updateGaitData(); // Read gait parameters from file

	// Always start with the outer feet in stance, and the inner feet tracking a scissor gait
	GAIT_FSM_MODE = PreMid_Out;
}


/* This is the main function for walking. All walking controls
 * are called from here */
void gaitControl_main(void) {
	updateGaitFsm();
	setGaitFsmLed();
}

