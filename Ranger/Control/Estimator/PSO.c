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

#define DIM_STATE_MAX 20 // Max dimension of the search space for optimization
#define POP_COUNT_MAX 25  // Max number of particles 

float PSO_DIM_STATE = 5;  // dimension of the search space
float PSO_POP_COUNT = 11;  // number of particles in the search

float (*PSO_OBJ_FUN)(void);  // Pointer to the objective function

float PSO_OMEGA = 0.7;  // particle velocity damping
float PSO_ALPHA = 1.8;  // local search parameter
float PSO_BETA = 1.5;  // global search parameter

bool PSO_RUN = false;  // Actively run particle swarm optimization?

/* Strict bounds on the search space (coerced) */
const float xLow[] = { -6.0, -1.0, -0.1, -0.4, -2};
const float xUpp[] = {0.1, 5.0, 7.0, 1.0, 10};

/* Arrays to store the population data */
float x[POP_COUNT_MAX][DIM_STATE_MAX];  // Current location of the particle
float v[POP_COUNT_MAX][DIM_STATE_MAX];  // best ever location of the particle
float f[POP_COUNT_MAX];  // Current value of the particle
float xBest[POP_COUNT_MAX][DIM_STATE_MAX]; // best ever value of the particle
float fBest[POP_COUNT_MAX]; // velocity of the particle

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
	for (dim = 0; dim < PSO_DIM_STATE; dim++) {
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
	PSO_OBJ_FUN = &objectiveFunction;

	for (dim = 0; dim < PSO_DIM_STATE; dim++) {
		r1 = FastRand();
		r2 = FastRand();
		x[idx][dim] = xLow[dim] + (xUpp[dim] - xLow[dim]) * r1;
		v[idx][dim] = -(xUpp[dim] - xLow[dim]) + 2 * (xUpp[dim] - xLow[dim]) * r2;
	}
	f[idx] = PSO_OBJ_FUN();

	// This point is the new best point by definition:
	fBest[idx] = f[idx];
	for (dim = 0; dim < PSO_DIM_STATE; dim++) {
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
	float xNew;
	int dim = 0;  // counter to loop over each dimension of the search space
	int idx = idxPopSelect; // Index of the currently selected particle

	PSO_OBJ_FUN = &objectiveFunction;

	// Compute the new point
	for (dim = 0; dim < PSO_DIM_STATE; dim++) {
		r1 = FastRand();
		r2 = FastRand();
		v[idx][dim] = PSO_OMEGA * v[idx][dim] +
		              PSO_ALPHA * r1 * (xBest[idx][dim] - x[idx][dim]) +
		              PSO_BETA * r2 * (xBest[idxPopGlobal][dim] - x[idx][dim]);
		xNew = x[idx][dim] + v[idx][dim];
		x[idx][dim] = Clamp(xNew, xLow[dim], xUpp[dim]);
	}
	f[idx] = PSO_OBJ_FUN();

	// Check if the new point is an improvement
	if (f[idx] < fBest[idx]) {
		fBest[idx] = f[idx];
		for (dim = 0; dim < PSO_DIM_STATE; dim++) {
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
		if (idxPopSelect >= PSO_POP_COUNT) {
			idxPopSelect = 0;
			initComplete = true; // We've ran through the population at least once
		}

	}
}
