#include <gaitControlData.h>
 
float GAITDATA_SPEED_KNOT_POINTS[] = { 0.00000000,  0.55000000,  1.00000000};

/* General comments:  The optimization is definitely multi-modal. */ 

/* This data is for the 1st-best controller found using off-line optimization. This 
 * seems to work well. */
float GAITDATA_WALK_SCISSOR_OFFSET[] = { 0.05000000,  0.0547,  0.25725469};
float GAITDATA_WALK_SCISSOR_GAIN[] = { 1.19430377,  1.2410,  1.00672954};
float GAITDATA_WALK_ANK_PUSH[] = { 0.70000000,  0.6575,  0.03346935};
float GAITDATA_WALK_CRIT_STEP_LENGTH[] = { 0.20309416,  0.1626,  0.35000000};
float GAITDATA_WALK_DS_DELAY[] = { 0.03132715,  0.0106,  0.01000000};

/* This data is for the 2nd-best controller found using off-line optimization.
 * seems to work ok, but is not obviously better than the first. More "bangy" 
 * than the first. */
// float GAITDATA_WALK_SCISSOR_OFFSET[] = { 0.05000000,  0.0578,  0.25725469};
// float GAITDATA_WALK_SCISSOR_GAIN[] = { 1.19430377,  1.2429,  1.00672954};
// float GAITDATA_WALK_ANK_PUSH[] = { 0.70000000,  0.6570,  0.03346935};
// float GAITDATA_WALK_CRIT_STEP_LENGTH[] = { 0.20309416,  0.1502,  0.35000000};
// float GAITDATA_WALK_DS_DELAY[] = { 0.03132715,  0.0169,  0.01000000};

/* This data is for the 3rd-best controller found using off-line optimization.
//  *    */
// float GAITDATA_WALK_SCISSOR_OFFSET[] = { 0.05000000,  0.0575,  0.25725469};
// float GAITDATA_WALK_SCISSOR_GAIN[] = { 1.19430377,  1.2369,  1.00672954};
// float GAITDATA_WALK_ANK_PUSH[] = { 0.70000000,  0.6145,  0.03346935};
// float GAITDATA_WALK_CRIT_STEP_LENGTH[] = { 0.20309416,  0.1518,  0.35000000};
// float GAITDATA_WALK_DS_DELAY[] = { 0.03132715,  0.0163,  0.01000000};



int GAITDATA_NGRID = 3;
float GAITDATA_TARGET_SPEED = 0.55;
 
int GAITDATA_NBOUND = 5;
float GAITDATA_LOW_BOUND[] = { 0.00000000,  1.00000000,  0.00000000,  0.10000000,  0.00000000};
float GAITDATA_UPP_BOUND[] = { 0.10000000,  1.30000000,  0.70000000,  0.25000000,  0.05000000};
