#ifndef __MOTORCONTROL_H__
#define __MOTORCONTROL_H__

/* standardized controller input struct */
struct ControllerData {
	float kp;	 	// proportional gain
	float kd;		// derivative gain
	float xRef; 	// reference joint angle
	float vRef;  	// reference joint angle rate
	float uRef;	// reference (nominal) torque required to achieve xRef and vRef
};

float MotorModel_Current(float, float);
void controller_hip(struct ControllerData *);
float getAnkleControllerCurrent( struct ControllerData *);
void controller_ankleOuter(struct ControllerData *);
void controller_ankleInner(struct ControllerData *);

void disable_motors(void);
void test_motor_control(void);

#endif // __MOTORCONTROL_H__
