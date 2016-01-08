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

/* Parameters that are updated once per step and read by the estimator,
 * which then passes them to the walking controller. They are initialized
 * in gaitControl_entry() */
float GAIT_WALK_ANK_PUSH;
float GAIT_WALK_CRIT_STANCE_ANGLE;
float GAIT_WALK_HIP_STEP_ANGLE;
float GAIT_WALK_SCISSOR_GAIN;
float GAIT_WALK_SCISSOR_OFFSET;
float GAIT_WALK_IDX;   // used for debugging - says which gait parameters are being used

/* The current working set of data, either from the optimization or from defaults */
int GD_NGRID;
float GD_KNOTS[GAITDATA_NGRID];
float GD_ANK_PUSH[GAITDATA_NGRID];
float GD_CRIT_ANGLE[GAITDATA_NGRID];
float GD_HIP_ANGLE[GAITDATA_NGRID];
float GD_SCISSOR_GAIN[GAITDATA_NGRID];
float GD_SCISSOR_OFFSET[GAITDATA_NGRID];

/* Load the default data set into the working data set */
void resetGaitData(void) {
	int i;  // counter
	GD_NGRID = (int)( GAITDATA_NGRID );
	for (i = 0; i < GAITDATA_NGRID; i++) {
		GD_KNOTS[i] = (float)( GAITDATA_SPEED_KNOT_POINTS[i] );
		GD_ANK_PUSH[i] =  (float)( GAITDATA_WALK_ANK_PUSH[i] );
		GD_CRIT_ANGLE[i] =  (float)( GAITDATA_WALK_CRIT_STANCE_ANGLE[i] );
		GD_HIP_ANGLE[i] =  (float)( GAITDATA_WALK_HIP_STEP_ANGLE[i] );
		GD_SCISSOR_GAIN[i] =  (float)( GAITDATA_WALK_SCISSOR_GAIN[i] );
		GD_SCISSOR_OFFSET[i] =  (float)( GAITDATA_WALK_SCISSOR_OFFSET[i] );
	}
}


/* This function is called once per walking step at mid-stance
 * It computes the new set of gait data that is used by the
 * walking controller  */
void updateGaitParam() {
	// Linear interpolation over data
	GAIT_WALK_ANK_PUSH = LinInterpVar(STATE_velCom, GD_KNOTS, GD_ANK_PUSH, GD_NGRID);
	GAIT_WALK_CRIT_STANCE_ANGLE = LinInterpVar(STATE_velCom, GD_KNOTS, GD_CRIT_ANGLE, GD_NGRID);
	GAIT_WALK_HIP_STEP_ANGLE = LinInterpVar(STATE_velCom, GD_KNOTS, GD_HIP_ANGLE, GD_NGRID);
	GAIT_WALK_SCISSOR_GAIN = LinInterpVar(STATE_velCom, GD_KNOTS, GD_SCISSOR_GAIN, GD_NGRID);
	GAIT_WALK_SCISSOR_OFFSET = LinInterpVar(STATE_velCom, GD_KNOTS, GD_SCISSOR_OFFSET, GAITDATA_NGRID);
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
				updateGaitParam();      // Update the controller at mid-stance on the outer legs
			} break;
		case PostMid_Out:
			if (STATE_c1) { // Inner feet hit ground
				triggerHeelStrikeUpdate();  // Tell estimtor that we've reached heel-strike
				GAIT_FSM_MODE = PreMid_Inn;
			} break;
		case PreMid_Inn:
			if (STATE_th1 < 0.0) {
				GAIT_FSM_MODE = PostMid_Inn;
				updateGaitParam();      // Update the controller at mid-stance on the outer legs
			} break;
		case PostMid_Inn:
			if (STATE_c0) { // Outer feet hit ground
				triggerHeelStrikeUpdate();  // Tell estimtor that we've reached heel-strike
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

	resetGaitData();  // Load default data 
	updateGaitParam(); // Interpolate gait parameters 
	optimizeGait_entry();  // Initialize optimization algorithm

	// Always start with the outer feet in stance, and the inner feet tracking a scissor gait
	GAIT_FSM_MODE = PreMid_Out;
}


/* This is the main function for walking. All walking controls
 * are called from here */
void gaitControl_main(void) {
	updateGaitFsm();
	setGaitFsmLed();
	optimizeGait_main();  // Call optimization on the gait (analysis)
}

