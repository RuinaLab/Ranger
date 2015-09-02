#ifndef __MB_ESTIMATOR_H__
#define __MB_ESTIMATOR_H__

#include <RangerMath.h>   // Tan(), bool

extern bool INITIALIZE_ESTIMATOR; // Should the estimator be initialized?

/* Parameters set by LabVIEW */
extern bool LABVIEW_HIP_COMPENSATION_TARGET; // Hip compensation at the target (true) or measured state (false)
extern bool LABVIEW_HIP_GRAVITY_COMPENSATION;
extern bool LABVIEW_HIP_SPRING_COMPENSATION;
extern float LABVIEW_HIP_KP;  // hip pd controller p gain
extern float LABVIEW_HIP_KD;  // hip pd controller d gain
extern float LABVIEW_ANK_PUSH_KP;  // ankle p gain used during push off 
extern float LABVIEW_ANK_PUSH_KD;  // ankle d gain used during push off 
extern float LABVIEW_ANK_STANCE_KP;  // ankle pd controller p gain when foot on ground.
extern float LABVIEW_ANK_STANCE_KD;  // ankle pd controller d gain when foot on ground.
extern float LABVIEW_ANK_SWING_KP;  // ankle pd controller p gain when foot in air.
extern float LABVIEW_ANK_SWING_KD;  // ankle pd controller d gain when foot in air.
extern float LABVIEW_WALK_ANK_PUSH; // magnitude of the push-off during walking  normalized to be on the range 0 to 1
extern float LABVIEW_WALK_CRIT_STANCE_ANGLE; // the critical stance leg angle when push-off should occur
extern float LABVIEW_WALK_HIP_STEP_ANGLE; //	Target angle for the hip to reach by the end of the step
extern float LABVIEW_WALK_HIP_TARGET_RATE;  //Target angular rate for the swing leg (hip joint) during the glide phase of motion
extern float LABVIEW_WALK_SCISSOR_GAIN;  
extern float LABVIEW_WALK_SCISSOR_OFFSET;

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

typedef enum {
	CONTACT_S0,
	CONTACT_S1,
	CONTACT_DS,
	CONTACT_FL
} ContactMode;

/* Robot contact configuration. */
extern bool STATE_c0; // true if outer feet are in contact
extern bool STATE_c1; // true if inner feet are in contact
extern ContactMode STATE_contactMode;  // stores current contact mode

/* Entry-point function */
void mb_estimator_update(void);

#endif  // __MB_ESTIMATOR_H__

