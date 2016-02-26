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
#include "mb_includes.h"
#include "RangerMath.h"  // FastRand()
#include "PSO.h"
#include "objectiveFunction.h"
#include "bufferDataOut.h"

#define DIM_STATE_MAX 20 // Max dimension of the search space for optimization
#define POP_COUNT_MAX 25  // Max number of particles 
#define COUNT_TRIAL_REPEAT 2 // Repeat each trial this many times

float DIM_STATE;  // dimension of the search space
float POP_COUNT;  // number of particles in the search

void (*OBJ_FUN_SEND)(float* x, int nDim);  // send query to obj. fun.
float (*OBJ_FUN_EVAL)(void);  // evaluate obj. fun.

float PSO_OMEGA = 0.7;  // particle velocity damping
float PSO_ALPHA = 1.8;  // local search parameter
float PSO_BETA = 1.5;  // global search parameter

/* Strict bounds on the search space (coerced) */
float X_LOW[DIM_STATE_MAX];
float X_UPP[DIM_STATE_MAX];

/* Arrays to store the population data */
float x[POP_COUNT_MAX][DIM_STATE_MAX];  // Current location of the particle
float v[POP_COUNT_MAX][DIM_STATE_MAX];  // best ever location of the particle
float f[POP_COUNT_MAX][COUNT_TRIAL_REPEAT];  // Current value of the particle
float xBest[POP_COUNT_MAX][DIM_STATE_MAX]; // best ever value of the particle
float fBest[POP_COUNT_MAX]; // velocity of the particle

/* Index for pointing to key parts of the arrays */
int idxPopGlobal = 0;  // Index of the populations best-ever particle
int idxPopSelect = 0;  // Index of the particle that is currently selected
int idxTrialRepeat = 0;  // Which trial are we on for this particle?
bool FLAG_INIT = false;  // Has the entire population been initialized?
int idxGeneration = 0;

/* Allow the user to send a "hint" point anytime */
bool FLAG_HINT = false;   // True if the user has just sent a hint to be tested
float X_HINT[DIM_STATE_MAX];   // the point that the user sent as a "hint"


/******************************************************************
 *               Public Interface Functions                       *
 ******************************************************************/

/* Sets the problem that will be solved by the optimization. This
 * is typically called by one of the problem setting methods in
 * objectiveFunction.c  */
void setObjFunInfo(float * xLow, float *xUpp, int nDim, int nPop
                   , void (*objFunSend)(float* x, int nDim), float (*objFunEval)(void)) {

	int dim;
	OBJ_FUN_SEND = objFunSend;
	OBJ_FUN_EVAL = objFunEval;
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

/* Allow the user to send a hint, which resets the position of the
 * active particle, and sets its velocity to zero. Must be called
 * after the problem is initialized with setObjFunInfo(). */
void psoGiveHint(float * xHint) {
	int dim;
	for (dim = 0; dim < DIM_STATE; dim++) {
		X_HINT[dim] = xHint[dim];
	}
	FLAG_HINT = true;
}

/* Clears the particle swarm and restarts the optimization */
void psoReset(void) {
	idxPopGlobal = 0;
	idxPopSelect = 0;
	idxTrialRepeat = 0;
	FLAG_INIT = false;
}

/* Returns the objective function value for the global best point. Returns
 * zero if the population has not been initialized. */
float psoGetGlobalBest(void) {
	if (FLAG_INIT) {
		return fBest[idxPopGlobal];
	} else {
		return 0.0;
	}
}

/* Returns the best objective function value for the selected particle. Returns
 * zero if the population has not been initialized. */
float psoGetSelectBest(void) {
	if (FLAG_INIT) {
		return fBest[idxPopSelect];
	} else {
		return 0.0;
	}
}

/* Returns the most recent objective function value. Returns
 * zero if the population has not been initialized. */
float psoGetSelectObjVal(void) {
	if (FLAG_INIT) {
		return f[idxPopSelect][idxTrialRepeat];
	} else {
		return 0.0;
	}
}

/* Returns the index of the currently selected particle */
int psoGetParticleId(void) {
	return idxPopSelect;
}

/* Returns the index of the trial for the current particle */
int psoGetTrialId(void) {
	return idxTrialRepeat;
}

/******************************************************************
 *                       Main Entry-Point                         *
 ******************************************************************
 *
 * Many objective function calls take more than one clock cycle to
 * compute, since they are tied to physical motions of the robot.
 * In order to accomodate this, I've programmed an asynchronous
 * objective function. Basically there is one call to get the next
 * point, and another call to get the value of that point.
 *
 * pso_send_point()
 * 	--> Call this first! It tells pso to compute the next search
 *      query point, and send it to the objective function. The
 * 		objective function then goes off and works on evaluating
 * 		the point.
 *
 * pso_eval_point()
 * 	--> Call this second! Be sure to only call this after the objective
 * 		function has successfully evaluated the query point.
 * 		calling this function will also increment the pointer in the
 * 		optimization to the next particle.
 */


/* Call this first! It computes a new query point for the search
 * and sends it to the objective function for evaluation. */
void pso_send_point(void) {
	float r1, r2;
	float xNew;
	int dim = 0;  // counter to loop over each dimension of the search space
	int idx = idxPopSelect; // Index of the currently selected particle

	if (FLAG_HINT) { // Initialize the current particle to the hint
		for (dim = 0; dim < DIM_STATE; dim++) {
			x[idx][dim] = X_HINT[dim];
			v[idx][dim] = 0.0;
			FLAG_HINT = false;  // Used the hint, so tell PSO to ignore it now.
		}

	} else if (!FLAG_INIT) {  // Initialize the particle randomly in search space
		for (dim = 0; dim < DIM_STATE; dim++) {
			r1 = FastRand();
			r2 = FastRand();
			x[idx][dim] = X_LOW[dim] + (X_UPP[dim] - X_LOW[dim]) * r1;
			v[idx][dim] = -(X_UPP[dim] - X_LOW[dim]) + 2 * (X_UPP[dim] - X_LOW[dim]) * r2;
		}

	} else {   // Run the standard particle update equations

		for (dim = 0; dim < DIM_STATE; dim++) {
			r1 = FastRand();
			r2 = FastRand();
			v[idx][dim] = PSO_OMEGA * v[idx][dim] +
			              PSO_ALPHA * r1 * (xBest[idx][dim] - x[idx][dim]) +
			              PSO_BETA * r2 * (xBest[idxPopGlobal][dim] - x[idx][dim]);
			xNew = x[idx][dim] + v[idx][dim];
			x[idx][dim] = Clamp(xNew, X_LOW[dim], X_UPP[dim]);
		}
	}

	mb_io_set_float(ID_OPTIM_CURRENT_GENERATION, idxGeneration);
	mb_io_set_float(ID_OPTIM_ACTIVE_PARTICLE, idxPopSelect);
	mb_io_set_float(ID_OPTIM_TRIAL_INDEX, idxTrialRepeat);

	OBJ_FUN_SEND(x[idx], DIM_STATE);   // Send query to objective function
}


/* Call this second! This should be called after the objective function has
 * finished evaluating the query point. */
void pso_eval_point(void) {
	int dim = 0;  // counter to loop over each dimension of the search space
	int idx = idxPopSelect; // Index of the currently selected particle
	float objFunVal;   // Store the tenetative best value for the objective function here  
	int iTrial;  // temp counter for looping over trials

	f[idx][idxTrialRepeat] = OBJ_FUN_EVAL();    // Reads the obj fun evaluation of query point

	// Only check for best point after all trials have been completed for a given trial
	if ( idxTrialRepeat >= (COUNT_TRIAL_REPEAT - 1) ) {  // Completed final trial!

		/* Compute the value of the objective function, which is defined to be the worst
 		 * of the trials that were attempted min(max()) optimization 
 		 *  -->   objFunVal = max(f[idx][]) 								*/
		objFunVal = f[idx][0];
		for (iTrial = 1; iTrial < COUNT_TRIAL_REPEAT; iTrial++){
			if (f[idx][iTrial] > objFunVal){
				objFunVal = f[idx][iTrial];  
			}
		}

		// Update the local best
		if (!FLAG_INIT) { // Still working on initialization
			fBest[idx] = objFunVal;  // current point is best by definition
			for (dim = 0; dim < DIM_STATE; dim++) {
				xBest[idx][dim] = x[idx][dim];
			}
		} else { // Initialization is complete
			if (objFunVal< fBest[idx]) {  // check new point for improvement
				fBest[idx] = objFunVal;
				for (dim = 0; dim < DIM_STATE; dim++) {
					xBest[idx][dim] = x[idx][dim];
				}
			}
		}

		// Update the global best:
		if (fBest[idxPopSelect] < fBest[idxPopGlobal] ) {
			idxPopGlobal = idxPopSelect;
		}

		// Incrememnt the particle pointer: new trial!
		idxPopSelect++; // Advance the index to point to next particle, check initPop:
		idxTrialRepeat = 0;  // Next particle starts on trial zero

		// See if we need to go to the next generation!
		if (idxPopSelect >= POP_COUNT) {
			idxPopSelect = 0;
			FLAG_INIT = true; // We've ran through the population at least once
			idxGeneration++;
		}

	} else {   // Same particle, next trial
		idxTrialRepeat++;  // Same particle, next evaluation
	}


}
