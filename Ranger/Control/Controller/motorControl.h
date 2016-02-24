#ifndef __MOTORCONTROL_H__
#define __MOTORCONTROL_H__

#include <stdbool.h>

extern float MOTOR_qh_trackErr;  // hip tracking error
extern float MOTOR_q0_trackErr;  // outer ankle tracking error
extern float MOTOR_q1_trackErr;  // inner ankle tracking error

void disable_motors(void);  // Shuts down all motors
void disable_hip(void); // shuts down hip motor
void disable_ankOut(void); // Shuts down the outer ankles
void disable_ankInn(void); // shuts down the inner ankles

void trackRel_ankOut(float q0, float kp, float kd);  // outer ankles track desired joint angle
void trackRel_ankInn(float q1, float kp, float kd);  // inner ankles track desired joint angle
void trackRel_hip(float qh, float kp, float kd);  // Hip motor tracks a desired hip joint angle
void trackVel_hip(float qh, float dqh, float kp, float kd);  // For hip tracking with velocity reference

void trackAbs_ankOut(float phi0, float kp, float kd); // Track absolute orientation of outer feet
void trackAbs_ankInn(float phi1, float kp, float kd); // Track absolute orientation of inner feet
void trackScissor_hip(float rate, float offset, float kp, float kd);  // Swing leg tracks linear function of stance angle

#endif // __MOTORCONTROL_H__
