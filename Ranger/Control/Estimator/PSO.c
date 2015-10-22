//============================================================================
// Name        : main.c
// Authors     : Matthew Kelly
//
// A simple implementation of particle swarm optimization intended to be used
// on Cornell Ranger for on-line controller optimization. This code is designed
// to be run offline, just to check that the algorithm works before transferring
// it to the libraries on the robot itself.
//
//============================================================================

#include <stdio.h>
#include <float.h>
#include "RangerMath.h"  // FastRand()
#include "PSO.h"

#define DIM_STATE 15 // Dimension of the search space for optimization
#define POP_COUNT 11  // Number of particles 

float omega = 0.7;  // particle velocity damping
float alpha = 0.7;  // local search parameter
float beta = 0.9;  // global search parameter

bool PSO_RUN = false;  // Actively run particle swarm optimization?

/* Bounds on the initial search space */
const float xLow[DIM_STATE] = { -6.0, -1.0, -0.1, -0.4, -2, -3, -1, -2, -1, -2, -4, -1, -3, -4, -5};
const float xUpp[DIM_STATE] = {0.1, 5.0, 7.0, 1.0, 10, 3, 7, 8, 9, 2.1, 1.1, 1.2, 2.3, 3.4, 1.5};

/* Arrays to store the population data */
float x[POP_COUNT][DIM_STATE];  // Current location of the particle
float v[POP_COUNT][DIM_STATE];  // best ever location of the particle
float f[POP_COUNT];  // Current value of the particle
float xBest[POP_COUNT][DIM_STATE]; // best ever value of the particle
float fBest[POP_COUNT]; // velocity of the particle

/* Index for pointing to key parts of the arrays */
int idxPopGlobal = 0;  // Index of the populations best-ever particle
int idxPopSelect = 0;  // Index of the particle that is currently selected
bool initComplete = false;  // Has the entire population been initialized?

/******************************************************************
 *               Public Interface Functions                       *
 ******************************************************************/

/* Clears the particle swarm and restarts the optimization */
void psoReset(void) {
	idxPopGlobal = 0;
	idxPopSelect = 0;
	initComplete = false;
}

/* Returns the objective function value for the global best point. Returns
 * zero if the population has not been initialized. */
float psoGetGlobalBest(void) {
	if (initComplete) {
		return fBest[idxPopGlobal];
	} else {
		return 0.0;
	}
}

/* Returns the best objective function value for the selected particle. Returns
 * zero if the population has not been initialized. */
float psoGetSelectBest(void) {
	if (initComplete) {
		return fBest[idxPopSelect];
	} else {
		return 0.0;
	}
}

/* Returns the most recent objective function value. Returns
 * zero if the population has not been initialized. */
float psoGetSelectObjVal(void) {
	if (initComplete) {
		return f[idxPopSelect];
	} else {
		return 0.0;
	}
}

/* Returns the index of the currently selected particle */
int psoGetParticleId(void) {
	return idxPopSelect;
}

/******************************************************************
 *                    Objective Function                          *
 ******************************************************************/
float objectiveFunction() {
	int dim; // counter to loop over each dimension
	float val;
	int idx = idxPopSelect; // Index of the currently selected particle
	float f = 0.0;  // value of the objective function

	// Simple quadratic bowl:
	for (dim = 0; dim < DIM_STATE; dim++) {
		val = x[idx][dim];
		f += val * val;
	}

	return f;
}



/******************************************************************
 *                    Initialize Particle                         *
 ******************************************************************
 * Initializes each particle at a random position and velocity in
 * the search space.
 */
void initializeParticle() {
	int idx = idxPopSelect; // Index of the currently selected particle
	int dim;
	float r1, r2;
	for (dim = 0; dim < DIM_STATE; dim++) {
		r1 = FastRand();
		r2 = FastRand();
		x[idx][dim] = xLow[dim] + (xUpp[dim] - xLow[dim]) * r1;
		v[idx][dim] = -(xUpp[dim] - xLow[dim]) + 2 * (xUpp[dim] - xLow[dim]) * r2;
	}
	f[idx] = objectiveFunction();

	// This point is the new best point by definition:
	fBest[idx] = f[idx];
	for (dim = 0; dim < DIM_STATE; dim++) {
		xBest[idx][dim] = x[idx][dim];
	}
}


/******************************************************************
 *                      Update Particle                           *
 ******************************************************************
 * Basically "time-steps the dynamics" for the particle and then checks
 * to see if the new point is an improvement.
 */
void updateParticle() {
	float r1, r2;
	int idx = idxPopSelect; // Index of the currently selected particle

	// Compute the new point
	int dim = 0;
	for (dim = 0; dim < DIM_STATE; dim++) {
		r1 = FastRand();
		r2 = FastRand();
		v[idx][dim] = omega * v[idx][dim] +
		              alpha * r1 * (xBest[idx][dim] - x[idx][dim]) +
		              beta * r2 * (xBest[idxPopGlobal][dim] - x[idx][dim]);
		x[idx][dim] = x[idx][dim] + v[idx][dim];
	}
	f[idx] = objectiveFunction();

	// Check if the new point is an improvement
	if (f[idx] < fBest[idx]) {
		fBest[idx] = f[idx];
		for (dim = 0; dim < DIM_STATE; dim++) {
			xBest[idx][dim] = x[idx][dim];
		}
	}
}

/******************************************************************
 *                       Main Entry-Point                         *
 ******************************************************************/
void particleSwarmOptimization(void) {
	if (PSO_RUN) {

		// Update particle:
		if (initComplete) {
			updateParticle();
		} else {
			initializeParticle();
		}

		// Check to see if the current point is better than the global best
		if (fBest[idxPopSelect] < fBest[idxPopGlobal] ) {
			idxPopGlobal = idxPopSelect;
		}

		// Advance the index to point to next particle, check initPop:
		idxPopSelect++;
		if (idxPopSelect >= POP_COUNT) {
			idxPopSelect = 0;
			initComplete = true; // We've ran through the population at least once
		}

	}
}
