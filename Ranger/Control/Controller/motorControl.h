#ifndef __MOTORCONTROL_H__
#define __MOTORCONTROL_H__

/*static*/ extern float leg_m;// = 2.5; //4.95 mass of the inner leg
/*static const*/extern float leg_r;// = 0.15; //length to the center of mass of inner leg
/*static const*/extern float g;// = 9.8;

/* joint limits */
/*static const*/extern float param_joint_ankle_flip;// = 0.3; // Hard stop at 0.0. Foot flips up to this angle to clear ground.
/*static const*/extern float param_joint_ankle_push;// = 2.5; // Hard stop at 3.0. Foot pushes off to inject energy, this is maximum bound.
/*static const*/extern float param_joint_ankle_hold;// = 1.662;

/* standardized controller input struct */
struct ControllerData {
	float wn;	// natural frequency 
	float xi;	// damping ratio
	float uRef;	// reference (nominal) torque required to achieve xRef and vRef
	float xRef; 	// reference joint angle
	float vRef;  	// reference joint angle rate 
	float kp;	//stiffness, proportional gain, torque
	float kd;	//dampness, derivative gain, torque
	float Cp;   // proportional gain, current, passed to the motor controller
	float Cd;	// derivative gain, current, passed to the motor controller
};

float MotorModel_Current(float, float);
void controller_hip(struct ControllerData *);
float getAnkleControllerCurrent( struct ControllerData *);
void controller_ankleOuter(struct ControllerData *);
void controller_ankleInner(struct ControllerData *);
float RangerAnkleControl(struct ControllerData * C, float x, float v);
float RangerHipControl(struct ControllerData * C, float x, float v);

void disable_motors(void);
void test_motor_control(void);
void test_sign(void);
void test_inner_foot(void);

void test_trajectory(void);
void test_freq_control(void);
void track_sin(void);
void double_stance(void);
void foot_flip(void);
float FO_flat_angle(void);
float FO_flat_rate(void);
float FI_flat_angle(void);
float FI_flat_rate(void);

void check_30(void);
void step(void);
void setPush(void);

#endif // __MOTORCONTROL_H__
