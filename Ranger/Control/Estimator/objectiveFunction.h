#ifndef __OBJECTIVEFUNCTION_H__
#define __OBJECTIVEFUNCTION_H__

/* Header files for objective function (passed to PSO) */

void objFun_set_quadraticBowl(void);
void quadraticBowl_send(float* x, int nDim);
float quadraticBowl_eval(void);

void objFun_set_sineTrack(void);
void sineTrack_send(float* x, int nDim);
float sineTrack_eval(void);
float sineTrack_run(void);

#endif  // __OBJECTIVEFUNCTION_H__
