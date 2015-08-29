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

/* Ankle Offset */
#define ZERO_POS_INN 1.85 //constant offset between absolute and relative ankle angle
#define ZERO_POS_OUT 1.8

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
