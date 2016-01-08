#include <mb_includes.h>
#include <mb_estimator.h>
#include <mb_controller.h>
#include <input_output.h>
#include <optimizeGait.h>
#include <walkControl.h>
#include <gaitControl.h>
#include <gaitControlData.h>

/* MECHANICS OF OPTIMIZATION TRIALS 
 *
 * Whenever the robot is walking it is quietly evaluating the gait in the background. 
 * A trial begins when the robot transitions from flight phase (in walk mode) to single
 * stance, since this is how walking is initiated. The objective function then logs 
 * data for a preset number of steps, and then ignores all subsequent steps. A trial 
 * is completed when the robot either falls down or is picked up. At this point, the
 * user has a choice. If he/she does nothing, then the objective function will be 
 * cleared when the robot shuts down, or starts walking again. On the other hand, if the
 * user presses the ACCEPT button (currently button 0), then the optimization will log
 * the trial, sending the data to the optimization algorithm. The optimization will then 
 * sample a new point, and then update the controller parameters to match this new point.
 * This way, when a new trial starts it will be evaluating the next point in the search.
 *
 * When the robot transitions back to "standby" mode, the current set of walk parameters 
 * are returned to default, but the optimization data remains, and returning to a walk 
 * mode will continue the optimization where it left off.
 *
 */

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
		
		//TODO:  Tally up the objective function   J = sum((SPEED-target).^2);

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

