#ifndef __OPTIMIZEGAIT_H__
#define __OPTIMIZEGAIT_H__

typedef enum {
	INIT,        // Wakes up here.
	PRE_1,       // Ready to start!
	PRE_2, 		 // MORE ready to start
	TRANS, 		 // Walking, but still in transient
	TRIAL, 	     // WOO!  Logging data
	POST 		 // Walking, but stop logging more data
} OptimizeFsmMode;

extern OptimizeFsmMode OPTIMIZE_FSM_MODE;  // mode for the finite state machine in the optimization

void logStepData(double duration, double length); // Called once per step by estimator

void optimizeGait_main(void);  // Called every tick to check for the start of a new trial.

#endif // __OPTIMIZEGAIT_H__
