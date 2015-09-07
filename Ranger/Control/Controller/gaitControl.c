#include <mb_includes.h>
#include <mb_estimator.h>
#include <mb_controller.h>
#include <input_output.h>
#include <gaitControl.h>
#include <gaitControlData.h>

typedef enum {
	PreMid_Out,   // Outer feet on the ground, inner feet swing through with scissor gait
	PostMid_Out,    // Outer feet on the ground,
	PreMid_Inn,  // Inner feet on the ground, inner feet swing through with scissor gait
	PostMid_Inn
} GaitFsmMode;

static GaitFsmMode GAIT_FSM_MODE = PreMid_Out;

/* Parameters that are updated once per step and read by the estimator,
 * which then passes them to the walking controller. They are initialized
 * in gaitControl_entry() */
float GAIT_WALK_ANK_PUSH;
float GAIT_WALK_CRIT_STANCE_ANGLE;
float GAIT_WALK_HIP_STEP_ANGLE;
float GAIT_WALK_SCISSOR_GAIN;
float GAIT_WALK_SCISSOR_OFFSET;
float GAIT_WALK_IDX;   // used for debugging - says which gait parameters are being used


/* This function is called once per walking step at mid-stance
 * It computes the new set of gait data that is used by the
 * walking controller */
void updateGaitData(float w) {
	int binIdx = (int)(w * GAITDATA_SLOPE + GAITDATA_CONST);
	if (binIdx < 0) binIdx = 0;
	if (binIdx > (GAITDATA_NBINS - 1)) binIdx = GAITDATA_NBINS;

	GAIT_WALK_ANK_PUSH = GAITDATA_WALK_ANK_PUSH[binIdx];
	GAIT_WALK_CRIT_STANCE_ANGLE = GAITDATA_WALK_CRIT_STANCE_ANGLE[binIdx];
	GAIT_WALK_HIP_STEP_ANGLE = GAITDATA_WALK_HIP_STEP_ANGLE[binIdx];
	GAIT_WALK_SCISSOR_GAIN = GAITDATA_WALK_SCISSOR_GAIN[binIdx];
	GAIT_WALK_SCISSOR_OFFSET = GAITDATA_WALK_SCISSOR_OFFSET[binIdx];

	mb_io_set_float(ID_GAIT_WALK_IDX, binIdx);   // Tell labview which gait parameter set we're using
}


/* This function is called once per loop, and checks the
 * sensors that are used to trigger transitions between
 * modes of the finite state machine */
void updateGaitFsm(void) {

	if (STATE_contactMode == CONTACT_FL) { // Then robot is in the air
		GAIT_FSM_MODE = PreMid_Out;  // Reset the finite state machine to initial state
	} else {  // Run the normal mid-stance finite state machine
		switch (GAIT_FSM_MODE) {
		case PreMid_Out:
			if (STATE_th0 < 0.0) {
				GAIT_FSM_MODE = PostMid_Out;
				updateGaitData(STATE_dth0);
			} break;
		case PostMid_Out:
			if (STATE_c1) { // Inner feet hit ground
				GAIT_FSM_MODE = PreMid_Inn;
			} break;
		case PreMid_Inn:
			if (STATE_th1 < 0.0) {
				GAIT_FSM_MODE = PostMid_Inn;
				updateGaitData(STATE_dth1);
			} break;
		case PostMid_Inn:
			if (STATE_c0) { // Inner feet hit ground
				GAIT_FSM_MODE = PreMid_Out;
			} break;
		}
	}
}


/* Turns on a specific led for each state of the walking FSM.
 * No LED indicates that the mode is flight. */
void setGaitFsmLed(void) {
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



/*******************************************************************************
 *                     MAIN ENTRY-POINT FUNCTIONS                              *
 *******************************************************************************/

/* This function is called once, as soon as the button is pressed
 * for the robot to begin walking. It is used for initialization. */
void gaitControl_entry(void) {

	updateGaitData(0.0);  // initialize the gait data

	// Always start with the outer feet in stance, and the inner feet tracking a scissor gait
	GAIT_FSM_MODE = PreMid_Out;
}


/* This is the main function for walking. All walking controls
 * are called from here */
void gaitControl_main(void) {
	updateGaitFsm();
	setGaitFsmLed();
}

