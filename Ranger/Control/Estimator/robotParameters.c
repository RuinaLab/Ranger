#include <robotParameters.h>

/* Robot physical parameters. */
const float PARAM_Phi0 = 1.86;  // outer ankle joint orientation constant
const float PARAM_Phi1 = 1.84;  // inner ankle joint orientation constant
const float PARAM_l = 0.96;  // length of ranger's leg
const float PARAM_d = 0.14;  // distance between foot joint and virtual center
const float PARAM_m = 4.5;  // Mass of a single leg on Ranger
const float PARAM_c = 0.15;  // Distance between hip joint and CoM along the leg
const float PARAM_g = 9.81;  // acceleration due to gravity

const float PARAM_hip_spring_const = 8.045;  // (Nm/rad) Hip spring constant
const float PARAM_inv_hip_motor_const = 0.841750841750842; // (Amp/Nm)  ==  (1.0)/(P.Km*P.Gh) == (1.0)/(0.018*66)
const float PARAM_inv_ank_motor_const = 1.633986928104575; // (Amp/Nm)  ==  (1.0)/(P.Km*P.Ga) == (1.0)/(0.018*34)

/* Parameters and set-points for walking sub-functions */
const float PARAM_ctrl_ank_flipTarget = 0.4;  // relative ankle angle when foot is flipped up. Hard stop at 0.0.
const float PARAM_ctrl_ank_holdLevel = 0.0;  // absolute foot angle for the stance foot to hold during the step
const float PARAM_ctrl_ank_pushTarget = -1.0;  // absolute foot angle when foot is flipped down for push-off.

/* Other stuff */
const float PARAM_critical_fall_leg_angle = 0.785398163397448; // 45 deg = pi/4  <--> if either ansolute leg angle exceeds this angle, then go to stand-by immediately
const float PARAM_critDoubleFailAngleSqr = 0.0076;  // (5*pi/180)^2 If double stance, and BOTH legs at this angle, then fall
