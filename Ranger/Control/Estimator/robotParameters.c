#include <robotParameters.h>

/* Robot physical parameters. */
extern const float PARAM_Phi = 1.8;  // ankle joint orientation constant
extern const float PARAM_l = 0.96;  // length of ranger's leg
extern const float PARAM_d = 0.14;  // distance between foot joint and virtual center
extern const float PARAM_m = 4.5;  // Mass of a single leg on Ranger
extern const float PARAM_c = 0.15;  // Distance between hip joint and CoM along the leg
extern const float PARAM_g = 9.81;  // acceleration due to gravity

extern const float PARAM_ank_motor_const = 0.612;  // (Nm/Amp) Motor Constant, including gear box
extern const float PARAM_hip_motor_const = 1.188;  // (Nm/Amp) Motor Constant, including gear box
extern const float PARAM_hip_spring_const = 8.045;  // (Nm/rad) Hip spring constant

extern const float PARAM_inv_hip_motor_const = 0.841750841750842; // (Amp/Nm)  ==  (1.0)/(P.Km*P.Gh) == (1.0)/(0.018*66)
extern const float PARAM_inv_ank_motor_const = 1.633986928104575; // (Amp/Nm)  ==  (1.0)/(P.Km*P.Ga) == (1.0)/(0.018*34)


