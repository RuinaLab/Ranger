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

#include "PSO.h"  // FastRand()


/******************************************************************
 *                       Main Entry-Point                         *
 ******************************************************************/
int main(void) {
	particleSwarmOptimization();
	return 0;
}