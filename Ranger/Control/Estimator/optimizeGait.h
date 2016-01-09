#ifndef __OPTIMIZEGAIT_H__
#define __OPTIMIZEGAIT_H__

typedef enum {
	INIT,        // Use has not yet activated optimization. Do nothing
	PRE_TRIAL,   // Just started walking, still in transient. Dump data.
	TRIAL,       // Main body of the walking trial. Log data.
	FLYING       // User picked up the robot.
} OptimizeFsmMode;

extern OptimizeFsmMode OPTIMIZE_FSM_MODE;  // mode for the finite state machine in the optimization

extern float OBJ_FUN_RUNNING_AVG; // Running average of the cost function for all the logged steps taken so far.

void logStepData(double duration, double length); // Called once per step by estimator

void optimizeGait_main(void);  // Called every tick to check for the start of a new trial.

#endif // __OPTIMIZEGAIT_H__
