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
#include "objFun.h"

#define DIM_STATE 6  // Dimension of the search space
#define POP_COUNT 12  // Number of particles 
#define ITER_COUNT 25 // Number of generations to complete

float omega = 0.5;  // particle velocity damping
float alpha = 0.7;  // local search parameter
float beta = 0.9;  // global search parameter

static const float xLow[DIM_STATE] = { -6.0, -1.0, -0.1, -0.4, -2, -3};
static const float xUpp[DIM_STATE] = {0.1, 5.0, 7.0, 1.0, 10, 3};

static float xGlobalBest[DIM_STATE]; // Store the best particle location
static float fGlobalBest = FLT_MAX;  // best particle value (attempting to minimize)

typedef struct {
	float x[DIM_STATE];  // Current location of the particle
	float xBest[DIM_STATE];  // best ever location of the particle
	float f; // Current value of the particle
	float fBest; // best ever value of the particle
	float v[DIM_STATE]; // velocity of the particle
} Particle;

/******************************************************************
 *                    Initialize Particle                         *
 ******************************************************************
 * Initializes each particle at a random position and velocity in
 * the search space.
 */
void initializeParticle(Particle* p) {
	int i;
	float r1, r2;
	for (i = 0; i < DIM_STATE; i++) {
		r1 = FastRand();
		r2 = FastRand();
		p->x[i] = xLow[i] + (xUpp[i] - xLow[i]) * r1;
		p->xBest[i] = p->x[i];
		p->v[i] = -(xUpp[i] - xLow[i]) + 2 * (xUpp[i] - xLow[i]) * r2;
	}
	p->f = objectiveFunction(p->x, DIM_STATE);
	p->fBest = p->f;
}


/******************************************************************
 *                      Update Particle                           *
 ******************************************************************
 * Basically "time-steps the dynamics" for the particle and then checks
 * to see if the new point is an improvement.
 */
void updateParticle(Particle* p) {
	float r1, r2;

	// Compute the new point
	int dim = 0;
	for (dim = 0; dim < DIM_STATE; dim++) {
		r1 = FastRand();
		r2 = FastRand();
		p->v[dim] = omega * p->v[dim] +
		            alpha * r1 * (p->xBest[dim] - p->x[dim]) +
		            beta * r2 * (xGlobalBest[dim] - p->x[dim]);
		p->x[dim] = p->x[dim] + p->v[dim];
	}
	p->f = objectiveFunction(p->x, DIM_STATE);

	// Check if the new point is an improvement
	if (p->f < p->fBest) {
		p->fBest = p->f;
		for (dim = 0; dim < DIM_STATE; dim++) {
			p->xBest[dim] = p->x[dim];
		}
	}
}


/******************************************************************
 *                      Update Global                             *
 ******************************************************************
 * Checks to see if the new point is better than the global best. If
 * so, then it updates the value in the global best.
 */
void updateGlobal(Particle* p) {
	int i = 0;
	if (p->fBest < fGlobalBest) {
		fGlobalBest = p->fBest;
		for (i = 0; i < DIM_STATE; i++) {
			xGlobalBest[i] = p->xBest[i];
		}
	}
}


/******************************************************************
 *                       Main Entry-Point                         *
 ******************************************************************/
void particleSwarmOptimization(void) {

	printf("Running particle swarm optimization...\n");
	printf("  - Population Size: %d \n", POP_COUNT);
	printf("  - omega: %3.3f \n", omega);
	printf("  - alpha: %3.3f \n", alpha);
	printf("  - beta: %3.3f \n", beta);

	Particle population[POP_COUNT];

// Top-level iteration loop
	int iter;
	int ind;
	int dim;
	for (iter = 0; iter < ITER_COUNT; iter++) {
		for (ind = 0; ind < POP_COUNT; ind++) {
			if (iter == 0) {
				initializeParticle(&population[ind]);
			} else {
				updateParticle(&population[ind]);
			}
			updateGlobal(&population[ind]);  // Check for the new global best
		}
		printf("iter: %2d,  fBest: %10.4g,  xBest: [", iter, fGlobalBest);
		for (dim = 0; dim < DIM_STATE; dim++) {
			if (dim > 0) printf(", ");
			printf("%10.4g", xGlobalBest[dim]);
		}
		printf("]\n");
	}

	printf("Done!\n");
}