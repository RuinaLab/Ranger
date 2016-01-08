#ifndef __OPTIMIZEGAIT_H__
#define __OPTIMIZEGAIT_H__

void acceptTrial(void);  // Accept the results of the current trial if valid

void logStepData(double duration, double length); // Called once per step by estimator

void optimizeGait_entry(void);  // Call once, to set up optimization
void optimizeGait_main(void);   // Call on every tick

#endif // __OPTIMIZEGAIT_H__
