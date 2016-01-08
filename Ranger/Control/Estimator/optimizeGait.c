#include <mb_includes.h>
#include <mb_estimator.h>
#include <mb_controller.h>
#include <input_output.h>
#include <optimizeGait.h>
#include <walkControl.h>
#include <gaitControl.h>
#include <gaitControlData.h>

static float STEP_COUNT = 0;
static float STEP_COUNT_LOW = 2;  // Start logging data at this step count
static float STEP_COUNT_UPP = 10; // Stop logging data after this step count

/*******************************************************************************
 *                    PRIVATE UTILITY FUNCTIONS                                *
 *******************************************************************************/

/* This function is called at the start of a new trial. This occurs whenever the 
 * robot transitions from flight phase to single stance.  */
void resetObjective(void){

}



/*******************************************************************************
 *                    PUBLIC INTERFACE FUNCTIONS                               *
 *******************************************************************************/

/* This function is triggered by a button press on the top level UI, designed to
 * be called by mb_controller.c. It tells the optimization to accept the current 
 * trial, if valid. */
void acceptTrial(void){

}

/* This function is called by the estimator each time that a step occurs */
void logStepData(double duration, double length){
	
}

/*******************************************************************************
 *                     MAIN ENTRY-POINT FUNCTIONS                              *
 *******************************************************************************/

/* This function is called once, as soon as the button is pressed
 * for the robot to begin walking. It is used for initialization. */
void optimizeGait_entry(void) {

}


/* This is the main function for walking. All walking controls
 * are called from here */
void optimizeGait_main(void) {

}

