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

#define DIM_STATE 2  // Dimension of the search space
#define POP_COUNT 15  // Number of particles
#define ITER_COUNT 20 // Number of generations to complete

static const float omega = 0.6;  // particle velocity damping
static const float alpha = 0.4;  // local search parameter
static const float beta = 0.6;  // global search parameter

static const float xLow[DIM_STATE] = { -2.0, -5.0};
static const float xUpp[DIM_STATE] = {6.0, 0.5};

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
 *                    Objective Function                          *
 ******************************************************************/
float objectiveFunction(float* x) {
	int i; // counter to loop over each dimension
	float f;  // value of the objective function

	// Simple quadratic bowl:
	for (i = 0; i < DIM_STATE; i++) {
		f = f + x[i] * x[i];
	}

	return f;
}



/******************************************************************
 *                    Initialize Particle                         *
 ******************************************************************
 * Initializes each particle at a random position in the search space
 * and then assigns a "velocity" that will move it to another random
 * point in the search space.
 */
void initializeParticle(Particle* p) {
	float r1 = FastRand();
	float r2 = FastRand();
	int i;
	for (i = 0; i < DIM_STATE; i++) {
		p->x[i] = xLow[i] + (xUpp[i] - xLow[i]) * r1;
		p->xBest[i] = p->x[i];
		p->v[i] = (xLow[i] + (xUpp[i] - xLow[i]) * r2) - p->x[i];
	}
	p->f = objectiveFunction(p->x);
	p->fBest = p->f;
}


/******************************************************************
 *                      Update Particle                           *
 ******************************************************************
 * Basically "time-steps the dynamics" for the particle and then checks
 * to see if the new point is an improvement.
 */
void updateParticle(Particle* p) {

	// Compute the new point
	int i = 0;
	float r1 = FastRand();
	float r2 = FastRand();
	for (i = 0; i < DIM_STATE; i++) {
		p->v[i] = omega * p->v[i] +
		          alpha * r1 * (p->xBest[i] - p->x[i]) +
		          beta * r2 * (xGlobalBest[i] - p->x[i]);
		p->x[i] = p->x[i] + p->v[i];
	}
	p->f = objectiveFunction(p->x);

	// Check if the new point is an improvement
	if (p->f < p->fBest) {
		p->fBest = p->f;
		for (i = 0; i < DIM_STATE; i++) {
			p->xBest[i] = p->x[i];
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
int main( int argc, const char ** argv ) {

	printf("Running particle swarm optimization...\n");

	Particle population[POP_COUNT];

// Top-level iteration loop
	int iter;
	int ind;
	for (iter = 0; iter < ITER_COUNT; iter++) {
		for (ind = 0; ind < POP_COUNT; ind++) {
			if (iter == 0) { // Initialization!
				initializeParticle(&population[ind]);
			} else {   // generation loop
				updateParticle(&population[ind]);
			}
			updateGlobal(&population[ind]);  // Check for the new global best
		}
		printf("iter: %2d,  fBest: %4.4f \n", iter, fGlobalBest);
	}

	printf("Done!\n");
	return 0;
}