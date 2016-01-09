#include <mb_includes.h>
#include <mb_estimator.h>
#include <mb_controller.h>
#include <input_output.h>
#include <optimizeGait.h>
#include <walkControl.h>
#include <gaitControl.h>
#include <gaitControlData.h>
#include "PSO.h"

/* MECHANICS OF OPTIMIZATION TRIALS
 *
 * Before starting a new trial, press the Accept button once to start optimization.
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

static const int N_POPULATION = 11;  // population to use in optimization

static float SPEED[N_STEP_TRIAL];
static float COST[N_STEP_TRIAL];

float OBJ_FUN_RUNNING_AVG = 0;

typedef enum {
	INIT,        // Use has not yet activated optimization. Do nothing
	PRE_TRIAL,   // Just started walking, still in transient. Dump data.
	TRIAL,       // Main body of the walking trial. Log data.
	POST_TRIAL,  // Trial has been successfully completed. Dump data, keep walking.
	FLYING       // User picked up the robot.
} OptimizeFsmMode;


static OptimizeFsmMode OPTIMIZE_FSM_MODE = INIT;




/*******************************************************************************
 *                      PSO INTERFACE FUNCTIONS                                *
 *******************************************************************************/


/* Objective function for a quadratic bowl. Evaluates the query point
 * the is sent by the optimizer and saves it to memory. */
void objFun_send(float* x, int nDim) {
	/// Update the middle value on each data channel. Assume that the other values [0] and [2] are good enough
	GAITDATA_WALK_SCISSOR_OFFSET[1] = x[0];
	GAITDATA_WALK_SCISSOR_GAIN[1] = x[1];
	GAITDATA_WALK_ANK_PUSH[1] = x[2];
	GAITDATA_WALK_CRIT_STANCE_ANGLE[1] = x[3];
	GAITDATA_WALK_HIP_STEP_ANGLE[1] = x[4];

	/// Send out over CAN network
	mb_io_set_float(ID_OPTIM_WALK_SCISSOR_OFFSET, x[0]);
	mb_io_set_float(ID_OPTIM_WALK_SCISSOR_GAIN, x[1]);
	mb_io_set_float(ID_OPTIM_WALK_ANK_PUSH, x[2]);
	mb_io_set_float(ID_OPTIM_WALK_CRIT_STANCE_ANGLE, x[3]);
	mb_io_set_float(ID_OPTIM_WALK_HIP_STEP_ANGLE, x[4]);
}


/* Returns the value of the objective function at the last called
 * query point. Accumulates the cost function*/
float objFun_eval(void) {
	int stepCountIdx;  // counter
	float objVal = 0;
	for (stepCountIdx = 0;  stepCountIdx < N_STEP_TRIAL; stepCountIdx++) {
		objVal += COST[stepCountIdx];
	}
	objVal = objVal / ( (float) N_STEP_TRIAL );

	mb_io_set_float(ID_OPTIM_OBJ_FUN_RUNNING_AVG, objVal ); // Report to LabView.

	                return objVal;
}


/* This function should be called once to create the optimization
 * problem, and then pass this information along to the optimization
 * method. */
void objFun_set_optimizeGait(void) {

	int dim;
	int nDim = 5;   ////HACK////   GAITDATA_NBOUND
	int nPop = N_POPULATION;
	void (*objFunSend)(float * x, int nDim);
	float (*objFunEval)();
	float xLow[5];
	float xUpp[5];
	objFunSend = &objFun_send;
	objFunEval = &objFun_eval;
	for (dim = 0; dim < nDim; dim++) {
		xLow[dim] = GAITDATA_LOW_BOUND[dim];
		xUpp[dim] = GAITDATA_UPP_BOUND[dim];
	}
	setObjFunInfo(xLow, xUpp, nDim, nPop, objFunSend, objFunEval); // Send problem to PSO

}



/*******************************************************************************
 *                    PRIVATE UTILITY FUNCTIONS                                *
 *******************************************************************************/

/* This function is called at the start of a new trial. This occurs whenever the
 * robot transitions from flight phase to single stance.  */
void resetObjective(void) {
	int stepCountIdx;  // counter
	STEP_COUNT = -N_STEP_TRANSIENT;

	mb_io_set_float(ID_OPTIM_STEP_COUNT, STEP_COUNT);

	for (stepCountIdx = 0;  stepCountIdx < N_STEP_TRIAL; stepCountIdx++) {
		SPEED[stepCountIdx] = 0.0;
	}
}

/* Running average of the cost of the steps so far. If it's past the recorded steps,
 * just return the final average of the trial. */
void objFun_runningAvg(void) {
	int stepCountIdx;  // counter
	static float objAvg = 0;
	if ( STEP_COUNT < 0) { // If we're in the "transient" steps, return 0.
		OBJ_FUN_RUNNING_AVG = 0;
	} else if ( STEP_COUNT >= N_STEP_TRIAL) { // If we're past the trial steps, return the final average of the trial.
		OBJ_FUN_RUNNING_AVG = objAvg;
	} else { // Return the running average so far.
		objAvg = 0;
		for (stepCountIdx = 0;  stepCountIdx < STEP_COUNT; stepCountIdx++) {
			objAvg += COST[stepCountIdx];
		}
		objAvg = objAvg / ( (float)STEP_COUNT );
		OBJ_FUN_RUNNING_AVG = objAvg;
	}

	mb_io_set_float(ID_OPTIM_OBJ_FUN_RUNNING_AVG, OBJ_FUN_RUNNING_AVG); // Report to LabView.
}



/*******************************************************************************
 *                         FINITE STATE MACHINE                                *
 *******************************************************************************/

void updateOptimizeFsm(void) {
	static bool DataSent = false;

	switch (OPTIMIZE_FSM_MODE) {

	case INIT:
		if (detect_UI_button_input(BUTTON_ACCEPT_TRIAL)) {
			objFun_set_optimizeGait();    // Set up optimization problem
			resetObjective();	// Clears objective and sets up for new trial
			DataSent = false;
			pso_send_point();   // Tell PSO to send a new point (Updates controller)
			OPTIMIZE_FSM_MODE = PRE_TRIAL;
		}
		break;

	case PRE_TRIAL:
		if (STEP_COUNT >= 0) { // Then start logging data for the trial!
			OPTIMIZE_FSM_MODE = TRIAL;
		} else if (STATE_contactMode == CONTACT_FL) {
			OPTIMIZE_FSM_MODE = FLYING;
		}
		break;

	case TRIAL:
		if (STEP_COUNT >= N_STEP_TRIAL) {
			OPTIMIZE_FSM_MODE = POST_TRIAL;
		} else if (STATE_contactMode == CONTACT_FL) {
			OPTIMIZE_FSM_MODE = FLYING;
		}
		break;

	case POST_TRIAL:
		if (STATE_contactMode == CONTACT_FL) {
			OPTIMIZE_FSM_MODE = FLYING;
		}
		break;

	case FLYING:
		if (detect_UI_button_input(BUTTON_ACCEPT_TRIAL) && !DataSent) {
			pso_eval_point();   // Evaluate objective function (send to PSO)
			pso_send_point();   // Tell PSO to send a new point (Updates controller)
			DataSent = true;
		}
		if ( STATE_contactMode != CONTACT_FL ) { // Trial failed due to environment (person walked in front of robot)
			resetObjective();	// Clears objective and sets up for new trial
			DataSent = false;
			OPTIMIZE_FSM_MODE = PRE_TRIAL;
		}
		break;
	}
}


/* Sets the color for the LED indicator on the board.
 'g' = green = standby mode
 'p' = purple = debug mode
 */
void updateOptimizeLed(void) {
	switch (OPTIMIZE_FSM_MODE) {

	case INIT:
		set_UI_LED(LED_UI_FSM, 'b');
		break;

	case PRE_TRIAL:
		set_UI_LED(LED_UI_FSM, 'c');
		break;

	case TRIAL:
		set_UI_LED(LED_UI_FSM, 'y');
		break;

	case POST_TRIAL:
		set_UI_LED(LED_UI_FSM, 'b');
		break;

	case FLYING:
		set_UI_LED(LED_UI_FSM, 'r');
		break;
	}
}




/*******************************************************************************
 *                    PUBLIC INTERFACE FUNCTIONS                               *
 *******************************************************************************/

/* This function is called by the estimator each time that a step occurs */
void logStepData(double duration, double length) {
	float err;

	if (OPTIMIZE_FSM_MODE == TRIAL) {
		if (duration != 0) {  // No divide by zero errors!  (keep default=0 if so)
			SPEED[STEP_COUNT] = length / duration;   // Log the speed data
			err = SPEED[STEP_COUNT] - GAITDATA_TARGET_SPEED; //Also evaluate the cost of this step.
			COST[STEP_COUNT] = err * err;
		}
	}

	objFun_runningAvg(); // Update the cost function's running average.

	STEP_COUNT++;

	mb_io_set_float(ID_OPTIM_STEP_COUNT, STEP_COUNT);
}


/* Call once per tick during walking. Checks for the start of a new walking trial. */
void optimizeGait_main(void) {
	updateOptimizeFsm();
	updateOptimizeLed();
}
