#include <mb_includes.h>
#include <mb_estimator.h>
#include <mb_controller.h>
#include <input_output.h>
#include <optimizeGait.h>
#include <walkControl.h>
#include <gaitControl.h>
#include <gaitControlData.h>

static int STEP_COUNT = 0;
static const int N_STEP_TRANSIENT = 2;  // Ignore the first few steps to reject transients
static const int N_STEP_TRIAL = 10; // Include this many steps in objective function

static float SPEED[N_STEP_TRIAL];


//TODO:  Add mechanics for objective function

//TODO:  Function handles and stuff for interfacing with PSO

//TODO:  Unit Testing!!


/*******************************************************************************
 *                    PRIVATE UTILITY FUNCTIONS                                *
 *******************************************************************************/

/* This function is called at the start of a new trial. This occurs whenever the
 * robot transitions from flight phase to single stance.  */
void resetObjective(void) {
	int stepCountIdx;  // counter
	STEP_COUNT = -N_STEP_TRANSIENT;
	for (stepCountIdx = 0;  stepCountIdx < N_STEP_TRIAL; stepCountIdx++) {
		SPEED[stepCountIdx] = 0.0;
	}
}


/*******************************************************************************
 *                    PUBLIC INTERFACE FUNCTIONS                               *
 *******************************************************************************/

/* This function is triggered by a button press on the top level UI, designed to
 * be called by mb_controller.c. It tells the optimization to accept the current
 * trial, if valid. */
void acceptTrial(void) {
	if (STEP_COUNT >= N_STEP_TRIAL) {  // Make sure that we've completed enough steps
		
		//TODO:  Tally up the objective function   J = sum((SPEED-target).^2);

	} else {
		mb_error_occurred(ERROR_EST_INCOMPLETE_TRIAL); // Send off an error message
	}
}

/* This function is called by the estimator each time that a step occurs */
void logStepData(double duration, double length) {
	if (STEP_COUNT >= 0) { // Then we've passed out of the transient period
		if (STEP_COUNT < N_STEP_TRIAL) { // Then we have not yet completed the trial
			if (duration != 0) {  // No divide by zero errors!  (keep default=0 if so)
				SPEED[STEP_COUNT] = length / duration;   // Log the speed data
			}
		}
	}
	STEP_COUNT++;
}

/*******************************************************************************
 *                     MAIN ENTRY-POINT FUNCTIONS                              *
 *******************************************************************************/

/* This function is called once, as soon as the button is pressed
 * for the robot to begin walking. It is used for initialization. */
void optimizeGait_entry(void) {

	// TODO:  Log the default controller value (for resets and initialization)

}


/* This is the main function for walking. All walking controls
 * are called from here */
void optimizeGait_main(void) {

	// Does anything else go here? 

	// TODO: Integral cost function ?

}

