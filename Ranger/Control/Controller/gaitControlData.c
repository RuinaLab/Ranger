#include <gaitControlData.h>
 
/* This data is manually selected! */
float GAITDATA_SPEED_KNOT_POINTS[] = { 0.00000000,  0.60000000,  1.00000000};

float GAITDATA_WALK_SCISSOR_OFFSET[] = {0.15, 0.1, 0.05}; // HACK //{0.15, 0.1, 0.05};
// {0.15, 0.1, 0.05} --> ORIGINAL
// {0.15, 0.15, 0.15} --> Vastly worse
// {0.05, 0.05, 0.05} --> Also worse
// Reverting

float GAITDATA_WALK_SCISSOR_GAIN[] = {1.1, 1.1, 1.2};
// {1.1, 1.1, 1.2} --> ORIGINAL 
// {1., 1.1, 1.2} --> bad, scuffs every step when getting started
// {1.2,1.1,1.2} --> Not really better

float GAITDATA_WALK_ANK_PUSH[] = {0.8, 0.6, 0.2};
float GAITDATA_WALK_CRIT_STEP_LENGTH[] = {0.25, 0.25, 0.25};
// {0.25, 0.25, 0.25} --> ORIGINAL
// {0.15,0.25,0.15} --> More symmetric. Little rough on first step.
// {0.15,0.25,0.10} --> Eh, better than original, slightly worse than prev
// {0.15,0.35,0.15} --> Nah, too big steps
// {0.20,0.25,0.15} --> Not as good as the first adjusted one.
// {0.12,0.25,0.20} --> Very hard to start.
// {0.18,0.25,0.12} --> Shrug
// {0.15,0.25,0.15} --> Back to this for awhile.
// {0.15,0.25,0.35} --> Less symmetric
// {0.25,0.25,0.10} --> not bad, not really better though

float GAITDATA_WALK_PUSH_INTEGRAL[] = {1.1, 0.7, 0.4};

int GAITDATA_NGRID = 3;
float GAITDATA_TARGET_SPEED = 0.6;
 
int GAITDATA_NBOUND = 5;
float GAITDATA_LOW_BOUND[] = { 0.05000000,  1.05000000,  0.20000000,  0.15,  0.5};
float GAITDATA_UPP_BOUND[] = { 0.15000000,  1.20000000,  0.90000000,  0.30,  1.0};
