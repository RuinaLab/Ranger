#ifndef __MOTORCONTROL_H__
#define __MOTORCONTROL_H__

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

void disable_motors(void);
void test_motor_control(void);

void test_trajectory(void);
void test_freq_control(void);


#endif // __MOTORCONTROL_H__
