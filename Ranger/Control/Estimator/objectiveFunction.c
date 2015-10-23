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


/* This function should be called once to create the optimization 
 * problem, and then pass this information along to the optimization
 * method. */
void objFun_set_quadraticBowl(void){
	int dim;
	int nDim = 5; 
	int nPop = 11;
	float (*objFun)(float* x, int nDim);
	float xLow[5];
	float xUpp[5];
	objFun = &quadraticBowl;
	for (dim=0; dim < nDim; dim++){
		xLow[dim] = -2.0*FastRand();
		xUpp[dim] = 2.0*FastRand();
	}
	setObjFunInfo(xLow, xUpp, nDim, nPop, objFun); // Send problem to PSO
}


/* Objective function for a quadratic bowl. Optimal solution is at the
 * origin. Input is an arbitrary length vector. */
float quadraticBowl(float* x, int nDim) {
	int dim; // counter to loop over each dimension
	float val;
	float f = 0.0;  // value of the objective function
	for (dim = 0; dim < nDim; dim++) {
		val = x[dim];
		f += val * val;  	// Simple quadratic bowl
	}
	return f;
}
