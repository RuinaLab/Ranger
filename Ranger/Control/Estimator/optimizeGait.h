#ifndef __OPTIMIZEGAIT_H__
#define __OPTIMIZEGAIT_H__

extern float OBJ_FUN_RUNNING_AVG; // Running average of the cost function for all the logged steps taken so far.

void logStepData(double duration, double length); // Called once per step by estimator

void optimizeGait_main(void);  // Called every tick to check for the start of a new trial.

#endif // __OPTIMIZEGAIT_H__
