#include "objFun.h"

/******************************************************************
 *                    Objective Function                          *
 ******************************************************************/
float objectiveFunction(float* x, int dimState) {
	int i; // counter to loop over each dimension
	float f = 0;  // value of the objective function

	// Simple quadratic bowl:
	for (i = 0; i < dimState; i++) {
		f += x[i] * x[i];
	}

	// Add noise to make optimization more difficult:
	//f = f + 0.4*(0.5-FastRand());   // uniform, bounded, zero mean noise

	return f;
}