#ifndef __ROBOT_PARAMETERS_H__
#define __ROBOT_PARAMETERS_H__

/* Robot physical parameters. */
extern const float PARAM_Phi0;  // inner ankle joint orientation constant
extern const float PARAM_Phi1;  // outer ankle joint orientation constant
extern const float PARAM_l;  // length of ranger's leg
extern const float PARAM_d;  // distance between foot joint and virtual center
extern const float PARAM_m;  // Mass of a single leg on Ranger
extern const float PARAM_c;  // Distance between hip joint and CoM along the leg
extern const float PARAM_g;  // acceleration due to gravity

/* Parameters for motor control */
extern const float PARAM_hip_spring_const;  // (Nm/rad) Hip spring constant
extern const float PARAM_inv_hip_motor_const; // (Amp/Nm)  ==  (1.0)/(P.Km*P.Gh) == (1.0)/(0.018*66)
extern const float PARAM_inv_ank_motor_const; // (Amp/Nm)  ==  (1.0)/(P.Km*P.Ga) == (1.0)/(0.018*34)

/* Parameters and set-points for walking sub-functions */
extern const float PARAM_ctrl_ank_flipTarget;  // relative ankle angle when foot is flipped up. Hard stop at 0.0.
extern const float PARAM_ctrl_ank_holdLevel;  // absolute foot angle for the stance foot to hold during the step
extern const float PARAM_ctrl_ank_pushTarget_0;  // relative ankle angle when foot is flipped down for push-off. Hard stop at 3.2.
extern const float PARAM_ctrl_ank_pushTarget_1;

/* Other stuff */
extern const float PARAM_critical_fall_leg_angle; // 45 deg = pi/4  <--> if either ansolute leg angle exceeds this angle, then go to stand-by immediately
extern const float PARAM_critDoubleFailAngleSqr;  // If double stance, and BOTH legs at this angle, then fall

#endif  // __ROBOT_PARAMETERS_H__
