#ifndef __PSO_H__
#define __PSO_H__

extern float omega;  // particle velocity damping
extern float alpha;  // local search parameter
extern float beta;  // global search parameter

void particleSwarmOptimization(void); // Main optimization call

#endif  // __PSO_H__