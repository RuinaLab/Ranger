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


/******************************************************************
 *                       Quadratic Bowl                           *
 ******************************************************************/
static float quadBowl_objVal; 

/* This function should be called once to create the optimization 
 * problem, and then pass this information along to the optimization
 * method. */
void objFun_set_quadraticBowl(void){
	int dim;
	int nDim = 5; 
	int nPop = 11;
	void (*objFunSend)(float* x, int nDim);
	float (*objFunEval)();
	float xLow[5];
	float xUpp[5];
	objFunSend = &quadraticBowl_send;
	objFunEval = &quadraticBowl_eval;
	for (dim=0; dim < nDim; dim++){
		xLow[dim] = -2.0*FastRand();
		xUpp[dim] = 2.0*FastRand();
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
