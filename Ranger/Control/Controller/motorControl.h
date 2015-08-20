#ifndef __MOTORCONTROL_H__
#define __MOTORCONTROL_H__

/* standardized controller input struct */
struct ControllerData {
	float uRef;		// reference (nominal) torque required to achieve xRef and vRef
	float xRef; 	// reference joint angle
	float vRef;  	// reference joint angle rate 
	float kp;	//stiffness, proportional gain, torque
	float kd;	//dampness, derivative gain, torque
	float Cp;   // proportional gain, current, passed to the motor controller
	float Cd;	// derivative gain, current, passed to the motor controller
};

extern float leg_m;// = 2.5; //4.95 mass of the inner leg
extern float leg_r;// = 0.15; //length to the center of mass of inner leg
extern float g;// = 9.8;

/* Ankle Offset */
#define ZERO_POS_INN 1.85 //constant offset between absolute and relative ankle angle
#define ZERO_POS_OUT 1.8

/* joint limits */
extern float param_joint_ankle_flip;// = 0.3; // Hard stop at 0.0. Foot flips up to this angle to clear ground.
extern float param_joint_ankle_push;// = 2.5; // Hard stop at 3.0. Foot pushes off to inject energy, this is maximum bound.
extern float param_joint_ankle_hold;// = 1.662;

extern float qr, qh, dqr, dqh, q0, q1, dq0, dq1; //relative
extern float th0, th1, dth0, dth1; //absolute 

float MotorModel_Current(float, float);
void controller_hip(struct ControllerData *);
float getAnkleControllerCurrentInn( struct ControllerData *);
float getAnkleControllerCurrentOut( struct ControllerData *);
void controller_ankleOuter(struct ControllerData *);
void controller_ankleInner(struct ControllerData *);
float RangerAnkleControl(struct ControllerData * C, float x, float v);
float RangerHipControl(struct ControllerData * C, float x, float v);

void disable_motors(void);
void test_motor_control(void);

float hip_gravity_compensation(void);
void hip_track_rel(struct ControllerData * ctrlData, float qh_ref, float dqh_ref, float KP, float KD);
void out_ank_track_abs(struct ControllerData * ctrlData, float phi0_ref, float dphi0_ref, float u_ref, float KP, float KD);
void inn_ank_track_abs(struct ControllerData * ctrlData, float phi1_ref, float dphi1_ref, float u_ref, float KP, float KD);
void hip_scissor_track_outer(struct ControllerData * ctrlData, float offset, float rate, float KP, float KD);
void hip_scissor_track_inner(struct ControllerData * ctrlData, float offset, float rate, float KP, float KD);
void angles_update(void);


#endif // __MOTORCONTROL_H__
