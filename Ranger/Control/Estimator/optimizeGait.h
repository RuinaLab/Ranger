#ifndef __OPTIMIZEGAIT_H__
#define __OPTIMIZEGAIT_H__

void acceptTrial(void);  // Accept the results of the current trial if valid

void logStepData(double duration, double length); // Called once per step by estimator

#endif // __OPTIMIZEGAIT_H__
