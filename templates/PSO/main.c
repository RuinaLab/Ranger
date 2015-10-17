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

static const float omega = 0.3;  // particle velocity damping
static const float alpha = 0.5;  // local search parameter
static const float beta = 0.7;  // global search parameter

static const float xLow[DIM_STATE] = { -1.0, -1.0};
static const float xUpp[DIM_STATE] = {1.0, 1.0};

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

		// //// HACK ////
		// CODE ONLY WORKS WHEN THE FOLLOWING LINE IS UNCOMMENTED
		// MEMORY VIOLATION SUSPECTED. FIX SOON.
		// printf("[%3.3f, %3.3f]\n",0.0,0.0);



	}
	p->f = objectiveFunction(p->x);

	// Check if the new point is an improvement
	if (p->f < p->fBest) {
		p->fBest = p->f;
		for (dim = 0; dim < DIM_STATE; dim++) {
			p->xBest[dim] = p->x[dim];
		}
	}

	// //// HACK ////
	// printf("(");
	// for (dim = 0; dim < DIM_STATE; dim++) {
	// 	if (dim > 0) printf(", ");
	// 	printf("%4.4f", xGlobalBest[dim]);
	// }
	// printf(")\n");

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





//// THIS CODE IS BROKEN!!!
// Probably a memory error somewhere.










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
		printf("iter: %2d,  fBest: %4.4f,  xBest: [", iter, fGlobalBest);
		for (dim = 0; dim < DIM_STATE; dim++) {
			if (dim > 0) printf(", ");
			printf("%4.4f", xGlobalBest[dim]);
		}
		printf("]\n");
	}

	printf("Done!\n");
	return 0;
}