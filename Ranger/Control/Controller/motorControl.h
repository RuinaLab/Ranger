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
	int GC;	// turns on gravity compensation when GC=1; GC is always turned off for the ankle controllers 
};

extern float leg_m;// = 2.5; //4.95 mass of the inner leg
extern float leg_r;// = 0.15; //length to the center of mass of inner leg
extern float g;// = 9.8;

/* Ankle Offset */
#define ZERO_POS_INN 1.85 //constant offset between absolute and relative ankle angle
#define ZERO_POS_OUT 1.8

/* global angle parameters*/
extern float qr, qh, dqr, dqh, q0, q1, dq0, dq1; //relative	angles
extern float th0, th1, dth0, dth1; //absolute angles 

float MotorModel_Current(float, float);
void controller_hip(struct ControllerData *);
void controller_ankleOuter(struct ControllerData *);
void controller_ankleInner(struct ControllerData *);
float getAnkleControllerCurrent( struct ControllerData *);

void disable_motors(void);
void motor_off(struct ControllerData * C);
void test_motor_control(void);

float hip_gravity_compensation(void);
void hip_track_rel(struct ControllerData * ctrlData, float qh_ref, float dqh_ref, float KP, float KD);
void out_ank_track_abs(struct ControllerData * ctrlData, float phi0_ref, float dphi0_ref, float u_ref, float KP, float KD);
void inn_ank_track_abs(struct ControllerData * ctrlData, float phi1_ref, float dphi1_ref, float u_ref, float KP, float KD);
void hip_scissor_track_outer(struct ControllerData * ctrlData, float offset, float rate, float KP, float KD);
void hip_scissor_track_inner(struct ControllerData * ctrlData, float offset, float rate, float KP, float KD);
void angles_update(void);


#endif // __MOTORCONTROL_H__
