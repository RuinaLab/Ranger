#ifndef __ROBOT_PARAMETERS_H__
#define __ROBOT_PARAMETERS_H__

/* Robot physical parameters. */
extern const float PARAM_Phi;  // ankle joint orientation constant
extern const float PARAM_l;  // length of ranger's leg
extern const float PARAM_d;  // distance between foot joint and virtual center
extern const float PARAM_m;  // Mass of a single leg on Ranger
extern const float PARAM_c;  // Distance between hip joint and CoM along the leg
extern const float PARAM_g;  // acceleration due to gravity

/* Parameters for motor control */
extern const float PARAM_hip_spring_const;  // (Nm/rad) Hip spring constant
extern const float PARAM_inv_hip_motor_const; // (Amp/Nm)  ==  (1.0)/(P.Km*P.Gh) == (1.0)/(0.018*66)
extern const float PARAM_inv_ank_motor_const; // (Amp/Nm)  ==  (1.0)/(P.Km*P.Ga) == (1.0)/(0.018*34)

#endif  // __ROBOT_PARAMETERS_H__
