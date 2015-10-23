#ifndef __PSO_H__
#define __PSO_H__

/* Header files for particle swarm optimization */

extern float PSO_OMEGA;  // particle velocity damping
extern float PSO_ALPHA;  // local search parameter
extern float PSO_BETA;  // global search parameter

extern bool PSO_RUN;  // Use to toggle the optimization

void psoReset(void); // Resets the optimization
float psoGetGlobalBest(void);  // Return global best obj. fun. value
float psoGetSelectBest(void);  // Return selected best obj. fun. value
float psoGetSelectObjVal(void); // Returns the most recent value of the objective function
int psoGetParticleId(void); // Returns the index of the current particle

void particleSwarmOptimization(void); // Main optimization call

#endif  // __PSO_H__
