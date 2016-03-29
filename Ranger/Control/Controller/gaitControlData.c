#include <gaitControlData.h>
 
/* On-line optimized solution.*/
float GAITDATA_SPEED_KNOT_POINTS[] = { 0.00000000,  0.50000000,  1.00000000};
float GAITDATA_WALK_SCISSOR_OFFSET[] = { 0.10000000,  0.0798,  0.13129540};
float GAITDATA_WALK_SCISSOR_GAIN[] = { 1.21895300,  1.0410,  1.17150049};
float GAITDATA_WALK_ANK_PUSH[] = { 0.70000000,  0.5000,  0.41975362};
float GAITDATA_WALK_CRIT_STEP_LENGTH[] = { 0.20057375,  0.1599,  0.23211128};
float GAITDATA_WALK_DS_DELAY[] = { 0.04795225,  0.0400,  0.04456843};
int GAITDATA_NGRID = 3;
float GAITDATA_TARGET_SPEED = 0.5;
 
int GAITDATA_NBOUND = 5;
// float GAITDATA_LOW_BOUND[] = { 0.10000000,  1.10000000,  0.30000000,  0.20000000,  0.00000000};
// float GAITDATA_UPP_BOUND[] = { 0.15000000,  1.30000000,  0.70000000,  0.30000000,  0.06000000};

// Center new bounds about initial search point
// The off-line optimization was sitting right on a constraint boundary. Here we relax those
// bounds and center them about the target point (middle data point only)
float GAITDATA_LOW_BOUND[] = { 0.05,  1.0,  0.5,  0.15,  0.04};
float GAITDATA_UPP_BOUND[] = { 0.15,  1.2,  0.9,  0.25,  0.08};
