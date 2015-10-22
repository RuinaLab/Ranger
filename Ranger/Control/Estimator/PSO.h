#ifndef __PSO_H__
#define __PSO_H__

/* Header files for particle swarm optimization */

extern float omega;  // particle velocity damping
extern float alpha;  // local search parameter
extern float beta;  // global search parameter

extern bool runPso;  // Use to toggle the optimization

void resetPso(void); // Resets the optimization
void particleSwarmOptimization(void); // Main optimization call

#endif  // __PSO_H__
