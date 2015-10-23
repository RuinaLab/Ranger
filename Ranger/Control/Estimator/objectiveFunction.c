//============================================================================
// Name        : objectiveFunction.c
// Authors     : Matthew Kelly
//
// This file implements various objective functions to be used for online
// optimization on the Cornell Ranger walking robot.
//
//============================================================================

#include "RangerMath.h"
#include "PSO.h"
#include "objectiveFunction.h"
#include "motorControl.h"
#include "mb_estimator.h"

/******************************************************************
 *                       Quadratic Bowl                           *
 ******************************************************************/
static float quadBowl_objVal;

/* This function should be called once to create the optimization
 * problem, and then pass this information along to the optimization
 * method. */
void objFun_set_quadraticBowl(void) {
	int dim;
	int nDim = 5;
	int nPop = 11;
	void (*objFunSend)(float * x, int nDim);
	float (*objFunEval)();
	float xLow[5];
	float xUpp[5];
	objFunSend = &quadraticBowl_send;
	objFunEval = &quadraticBowl_eval;
	for (dim = 0; dim < nDim; dim++) {
		xLow[dim] = -2.0 * FastRand();
		xUpp[dim] = 2.0 * FastRand();
	}
	setObjFunInfo(xLow, xUpp, nDim, nPop, objFunSend, objFunEval); // Send problem to PSO
}


/* Objective function for a quadratic bowl. Evaluates the query point
 * the is sent by the optimizer and saves it to memory. */
void quadraticBowl_send(float* x, int nDim) {
	int dim; // counter to loop over each dimension
	float val;
	quadBowl_objVal = 0.0;  // value of the objective function
	for (dim = 0; dim < nDim; dim++) {
		val = x[dim];
		quadBowl_objVal += val * val;  	// Simple quadratic bowl
	}
}


/* Returns the value of the objective function at the last called
 * query point. */
float quadraticBowl_eval(void) {
	return quadBowl_objVal;
}




/******************************************************************
 *                   Ankle Track Sine Wave                        *
 ******************************************************************
 *
 * This is another unit test for the particle swarm optimization.
 * The goal is to learn the parameters of a tracking controller for
 * the inner ankles that matches (in a least-squares sense) a target
 * trajectory in time.
 *
 * The anke joint should track the following function of time:
 * 		q(t) = qLow + (qUpp-qLow)*(0.5+0.5*Sin(2*pi*t/period))
 *
 * 		qLow = 0.3;     
 * 		qUpp = 1.6;    
 * 		period = 1.4;   
 * 		kp = ??;  		
 * 		kd = ??; 		
 *
 * 		decVar = [qLow; qUpp; period; kp; kd]
 */

/* Initialize variables at low bound*/
static float sineTrack_qLow = 0.2;
static float sineTrack_qUpp = 1.2;
static float sineTrack_period = 0.9;
static float sineTrack_kp = 1.0;
static float sineTrack_kd = 0.0;

static float sineTrack_objVal = 0.0; // Accumulated objective function:
static bool sineTrack_initComplete = false; 

/* This function sets up the sineTrack optimization problem
 * and sends it to PSO to be optimized */
void objFun_set_sineTrack(void) {
	int nDim = 5;
	int nPop = 11;
	void (*objFunSend)(float * x, int nDim);
	float (*objFunEval)();
	float xLow[5];
	float xUpp[5];
	objFunSend = &sineTrack_send;
	objFunEval = &sineTrack_eval;
	xLow[0] = 0.2;  xUpp[0] = 0.8; //qLow   --> 0.3
	xLow[1] = 1.3;  xUpp[1] = 2.1; //qUpp   --> 1.6
	xLow[2] = 1.0;  xUpp[2] = 1.8; //period --> 1.4
	xLow[3] = 1.0;  xUpp[3] = 10.0; //kp  --> 7 ?
	xLow[4] = 0.0;  xUpp[4] = 2.0; //kd  --> 1 ?
	setObjFunInfo(xLow, xUpp, nDim, nPop, objFunSend, objFunEval); // Send problem to PSO
}

/* This function is called by pso to start a new trial. It updates
 * all of the parameters to the tracking controller and resets the
 * accumulation of the objective function */
void sineTrack_send(float* x, int nDim) {
	sineTrack_qLow = x[0];
	sineTrack_qUpp = x[1];
	sineTrack_period = x[2];
	sineTrack_kp = x[3];
	sineTrack_kd = x[4];
}

/* This function reads the accumulation of the objective function
 * and then sends it back to pso. */
float sineTrack_eval(void){
	return sineTrack_objVal;
}

/* Entry-point for the sineTrack optimization loop. Returns the 
 * target angle for the ankle joint. */
float sineTrack_run(void){
	static int loopCount = 0;
	int loopCountMax = 700;  // period*loopFreq = (1.4s)*(500Hz)
	float dt = 0.002;  // nominal clock cycle time 1/(500Hz)
	float t = dt*loopCount;
	float qCmd;  // Target angle send to motors
	float qMeasured = STATE_q1;  // Inner ankle angle (measured);
	float qRef;  //  The "real" target angle 
	float qErr;  // Difference between measured and reference;
	float qLow = sineTrack_qLow;
	float qUpp = sineTrack_qUpp;
	float period = sineTrack_period;
	float kp = sineTrack_kp;
	float kd = sineTrack_kd;

	// Send the problem to PSO when program first runs
	if (!sineTrack_initComplete){
		objFun_set_sineTrack();  // Send problem to pso
		sineTrack_initComplete = true;
	} 

	// Check if we are at the beginning of an experiment
	if (loopCount == 0){  //Beginning of experiment!
		pso_send_point();  // Tell PSO to send a candidate controller
		sineTrack_objVal = 0.0;  // Zero out obj fun accumulator
	}

	// Send commands to the motor controller
	qCmd = qLow + (qUpp-qLow)*(0.5+0.5*Sin(TWO_PI*t/period));  
	trackRel_ankInn(qCmd, kp, kd);  

	// Compute error between reference and measured
	qRef = 0.3 + (1.6-0.3)*(0.5+0.5*Sin(TWO_PI*t/1.4)); 
	qErr = qRef-qMeasured;
	sineTrack_objVal += qErr*qErr; // Minimize the sum or squared error along trajectory

	// Check if we are at the end of an experiment
	loopCount++;
	if (loopCount >= loopCountMax){  // Then at end of experiment
		pso_eval_point();  // Tell PSO to read the objective function now
		loopCount = 0;  // Reset the loop counter --> trigger new experiment 
	}

	return qRef; // Send the target joint angle

}
