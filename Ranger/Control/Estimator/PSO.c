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
#define ITER_COUNT 25 // Number of generations to complete

float omega = 0.5;  // particle velocity damping
float alpha = 0.7;  // local search parameter
float beta = 0.9;  // global search parameter

bool runPso = false;  // Actively run particle swarm optimization?

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
bool initPop = false;  // Has the entire population been initialized?


/* Clears the particle swarm and restarts the optimization */
void resetPso(void) {
	idxPopBest = 0;
	idxPopSelect = 0;
	initPop = false;
}

/******************************************************************
 *                    Objective Function                          *
 ******************************************************************/
float objectiveFunction(float* x, int dimState) {
	int dim; // counter to loop over each dimension
	float f = 0;  // value of the objective function

	// Simple quadratic bowl:
	for (dim = 0; dim < dimState; dim++) {
		f += x[dim] * x[dim];
	}

	// Add noise to make optimization more difficult:
	//f = f + 0.4*(0.5-FastRand());   // uniform, bounded, zero mean noise

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
	population[idx].f = objectiveFunction(population[idx].x, DIM_STATE);
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
	population[idx].f = objectiveFunction(population[idx].x, DIM_STATE);

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
	if (runPso) {

		// Update particle:
		if (initPop) {
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
			initPop = true; // We've ran through the population at least once
		}

	}
}
