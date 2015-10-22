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

#define DIM_STATE 6 // Dimension of the search space for optimization
#define POP_COUNT 12  // Number of particles 

float omega = 0.5;  // particle velocity damping
float alpha = 0.7;  // local search parameter
float beta = 0.9;  // global search parameter

bool PSO_RUN = false;  // Actively run particle swarm optimization?

/* Bounds on the initial search space */
const float xLow[DIM_STATE] = { -6.0, -1.0, -0.1, -0.4, -2, -3};
const float xUpp[DIM_STATE] = {0.1, 5.0, 7.0, 1.0, 10, 3};

typedef struct {
	float x[DIM_STATE];  // Current location of the particle
	float xBest[DIM_STATE];  // best ever location of the particle
	float f; // Current value of the particle
	float fBest; // best ever value of the particle
	float v[DIM_STATE]; // velocity of the particle
} Particle;

Particle population[POP_COUNT]; // Population of particles for the search
int idxPopBest = 0;  // Index of the populations best-ever particle
int idxPopSelect = 0;  // Index of the particle that is currently selected
bool initComplete = false;  // Has the entire population been initialized?

/******************************************************************
 *               Public Interface Functions                       *
 ******************************************************************/

/* Clears the particle swarm and restarts the optimization */
void psoReset(void) {
	idxPopBest = 0;
	idxPopSelect = 0;
	initComplete = false;
}

/* Returns the objective function value for the global best point. Returns
 * zero if the population has not been initialized. */
float psoGetGlobalBest(void) {
	if (initComplete) {
		return population[idxPopBest].fBest;
	} else {
		return 0.0;
	}
}

/* Returns the best objective function value for the selected particle. Returns
 * zero if the population has not been initialized. */
float psoGetSelectBest(void) {
	if (initComplete) {
		return population[idxPopSelect].fBest;
	} else {
		return 0.0;
	}
}

/* Returns the most recent objective function value. Returns
 * zero if the population has not been initialized. */
float psoGetSelectObjVal(void) {
	if (initComplete) {
		return population[idxPopSelect].f;
	} else {
		return 0.0;
	}
}

/* Returns the index of the currently selected particle */
int psoGetParticleId(void){
	return idxPopSelect;
}

/******************************************************************
 *                    Objective Function                          *
 ******************************************************************/
float objectiveFunction() {
	int dim; // counter to loop over each dimension
	float val;
	float f = 0.0;  // value of the objective function

	// Simple quadratic bowl:
	for (dim = 0; dim < DIM_STATE; dim++) {
		val = population[idxPopSelect].x[dim];
		f += val*val;
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
	int dim;
	int idx = idxPopSelect; // Index of the currently selected particle
	float r1, r2;
	for (dim = 0; dim < DIM_STATE; dim++) {
		r1 = FastRand();
		r2 = FastRand();
		population[idx].x[dim] = xLow[dim] + (xUpp[dim] - xLow[dim]) * r1;
		population[idx].xBest[dim] = population[idx].x[dim];
		population[idx].v[dim] = -(xUpp[dim] - xLow[dim]) + 2 * (xUpp[dim] - xLow[dim]) * r2;
	}
	population[idx].f = objectiveFunction();
	population[idx].fBest = population[idx].f;
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
		population[idx].v[dim] = omega * population[idx].v[dim] +
		                         alpha * r1 * (population[idx].xBest[dim] - population[idx].x[dim]) +
		                         beta * r2 * (population[idxPopBest].xBest[dim] - population[idx].x[dim]);
		population[idx].x[dim] = population[idx].x[dim] + population[idx].v[dim];
	}
	population[idx].f = objectiveFunction();

	// Check if the new point is an improvement
	if (population[idx].f < population[idx].fBest) {
		population[idx].fBest = population[idx].f;
		for (dim = 0; dim < DIM_STATE; dim++) {
			population[idx].xBest[dim] = population[idx].x[dim];
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
			initializeParticle();
		} else {
			updateParticle();
		}

		// Check to see if the current point is better than the global best
		if (population[idxPopSelect].fBest < population[idxPopBest].fBest) {
			idxPopBest = idxPopSelect;
		}

		// Advance the index to point to next particle, check initPop:
		idxPopSelect++;
		if (idxPopSelect >= POP_COUNT) {
			idxPopSelect = 0;
			initComplete = true; // We've ran through the population at least once
		}

	}
}
