//============================================================================
// Name        : PSO.c
// Authors     : Matthew Kelly
//
// A simple implementation of particle swarm optimization intended to be used
// on Cornell Ranger for on-line controller optimization. This code is designed
// to be run offline, just to check that the algorithm works before transferring
// it to the libraries on the robot itself.
//
//============================================================================

#include "RangerMath.h"  // FastRand()
#include "PSO.h"
#include "objectiveFunction.h"

#define DIM_STATE_MAX 20 // Max dimension of the search space for optimization
#define POP_COUNT_MAX 25  // Max number of particles 

float DIM_STATE;  // dimension of the search space
float POP_COUNT;  // number of particles in the search

float (*OBJ_FUN)(float* x, int nDim);  // Pointer to the objective function

float PSO_OMEGA = 0.7;  // particle velocity damping
float PSO_ALPHA = 1.8;  // local search parameter
float PSO_BETA = 1.5;  // global search parameter

bool PSO_RUN = false;  // Actively run particle swarm optimization?

/* Strict bounds on the search space (coerced) */
float X_LOW[DIM_STATE_MAX];
float X_UPP[DIM_STATE_MAX];

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

/* Sets the problem that will be solved by the optimization. This
 * is typically called by one of the problem setting methods in
 * objectiveFunction.c  */
void setObjFunInfo(float * xLow, float *xUpp, int nDim, int nPop, float (*objFun)(float* x, int nDim)) {
	int dim;
	OBJ_FUN = objFun;
	DIM_STATE = Clamp(nDim, 1, DIM_STATE_MAX);
	POP_COUNT = Clamp(nPop, 1, POP_COUNT_MAX);

	if (nDim != DIM_STATE) {
		////TODO//// Send error message!
	}
	if (nPop != POP_COUNT) {
		////TODO//// Send error message!
	}

	for (dim = 0; dim < DIM_STATE; dim++) {
		X_LOW[dim] = xLow[dim];
		X_UPP[dim] = xUpp[dim];
	}
}




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
		x[idx][dim] = X_LOW[dim] + (X_UPP[dim] - X_LOW[dim]) * r1;
		v[idx][dim] = -(X_UPP[dim] - X_LOW[dim]) + 2 * (X_UPP[dim] - X_LOW[dim]) * r2;
	}
	f[idx] = OBJ_FUN(x[idx], DIM_STATE);

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
	float xNew;
	int dim = 0;  // counter to loop over each dimension of the search space
	int idx = idxPopSelect; // Index of the currently selected particle

	// Compute the new point
	for (dim = 0; dim < DIM_STATE; dim++) {
		r1 = FastRand();
		r2 = FastRand();
		v[idx][dim] = PSO_OMEGA * v[idx][dim] +
		              PSO_ALPHA * r1 * (xBest[idx][dim] - x[idx][dim]) +
		              PSO_BETA * r2 * (xBest[idxPopGlobal][dim] - x[idx][dim]);
		xNew = x[idx][dim] + v[idx][dim];
		x[idx][dim] = Clamp(xNew, X_LOW[dim], X_UPP[dim]);
	}
	f[idx] = OBJ_FUN(x[idx], DIM_STATE);

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
