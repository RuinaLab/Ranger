#ifndef __PSO_H__
#define __PSO_H__

/* Header files for particle swarm optimization */

extern float PSO_OMEGA;  // particle velocity damping
extern float PSO_ALPHA;  // local search parameter
extern float PSO_BETA;  // global search parameter

/// Call this to define the optimization problem.
void setObjFunInfo(float * xLow, float *xUpp, int nDim, int nPop
                   , void (*objFunSend)(float* x, int nDim), float (*objFunEval)(void));

void psoGiveHint(float * xHint);  // Give the next optimization search point
void psoReset(void); // Resets the optimization
float psoGetGlobalBest(void);  // Return global best obj. fun. value
float psoGetSelectBest(void);  // Return selected best obj. fun. value
float psoGetSelectObjVal(void); // Returns the most recent value of the objective function
int psoGetParticleId(void); // Returns the index of the current particle

void pso_send_point(void); // Call 1st!  Sends query point to obj. fun.
void pso_eval_point(void); // Call 2nd!  Reads evaluation of query point.
void saveOptim(void);


#endif  // __PSO_H__
