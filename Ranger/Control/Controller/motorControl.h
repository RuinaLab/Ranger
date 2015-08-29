#ifndef __MOTORCONTROL_H__
#define __MOTORCONTROL_H__

extern bool HIP_GRAVITY_COMPENSATION;
extern bool HIP_SPRING_COMPENSATION;

void disable_motors(void);  // Shuts down all motors

void trackRel_ankOut(float q0, float kp, float kd);  // outer ankles track desired joint angle
void trackRel_ankInn(float q1, float kp, float kd);  // inner ankles track desired joint angle
void trackRel_hip(float qh, float kp, float kd);  // Hip motor tracks a desired hip joint angle

void trackAbs_ankOut(float phi0, float kp, float kd); // Track absolute orientation of outer feet
void trackAbs_ankInn(float phi1, float kp, float kd); // Track absolute orientation of inner feet
void trackScissor_hip(float rate, float offset, float kp, float kd);  // Swing leg tracks linear function of stance angle

#endif // __MOTORCONTROL_H__
