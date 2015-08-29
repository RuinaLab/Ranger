#ifndef __MB_ESTIMATOR_H__
#define __MB_ESTIMATOR_H__

#include <RangerMath.h>   // Tan(), bool

extern bool INITIALIZE_ESTIMATOR; // Should the estimator be initialized?

/* Robot state variables. Naming conventions in docs. Matches simulator. */
extern float STATE_qh;  // hip angle
extern float STATE_q0;  // outer ankle angle
extern float STATE_q1;  // inner ankle angle
extern float STATE_dqh;  // hip rate
extern float STATE_dq0;  // outer ankle rate
extern float STATE_dq1;  // inner ankle rate
extern float STATE_th0;  // absolute orientation of outer legs
extern float STATE_th1;  // absolute orientation of inner legs
extern float STATE_phi0;  // absolute orientation of outer feet
extern float STATE_phi1;  // absolute orientation of inner feet
extern float STATE_dth0;  // absolute orientation rate of outer legs
extern float STATE_dth1;  // absolute orientation rate of inner legs
extern float STATE_dphi0;  // absolute orientation rate of outer feet
extern float STATE_dphi1;  // absolute orientation rate of inner feet

/* Robot physical parameters. */
extern const float PARAM_Phi;  // ankle joint orientation constant

/* helper functions */
bool getContactOuter(void); // Returns true if outer feet on ground
bool getContactInner(void);  // returns true if outer feet on ground
void resetOuterLegAngle(float);  // hard reset the outer leg angle to some value
void updateOuterLegAngle(void); // Call on each heel-strike to correct drift in the rate gyro

/* Entry-point function */
void mb_estimator_update(void);

#endif  // __MB_ESTIMATOR_H__

