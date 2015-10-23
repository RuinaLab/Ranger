#ifndef __OBJECTIVEFUNCTION_H__
#define __OBJECTIVEFUNCTION_H__

/* Header files for objective function (passed to PSO) */

void objFun_set_quadraticBowl(void);
float quadraticBowl(float* x, int nDim);

#endif  // __OBJECTIVEFUNCTION_H__
