#ifndef __OBJECTIVEFUNCTION_H__
#define __OBJECTIVEFUNCTION_H__

/* Header files for objective function (passed to PSO) */

void objFun_set_quadraticBowl(void);
void quadraticBowl_send(float* x, int nDim);
float quadraticBowl_eval(void);

#endif  // __OBJECTIVEFUNCTION_H__
