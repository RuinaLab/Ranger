#ifndef __MOTORCONTROL_H__
#define __MOTORCONTROL_H__

#include <stdbool.h>

extern bool HIP_GRAVITY_COMPENSATION;
extern bool HIP_SPRING_COMPENSATION;

void disable_motors(void);  // Shuts down all motors
void disable_hip(void); // shuts down hip motor
void disable_ankOut(void); // Shuts down the outer ankles
void disable_ankInn(void); // shuts down the inner ankles

void trackRel_ankOut(float q0, float kp, float kd);  // outer ankles track desired joint angle
void trackRel_ankInn(float q1, float kp, float kd);  // inner ankles track desired joint angle
void trackRel_hip(float qh, float kp, float kd);  // Hip motor tracks a desired hip joint angle

void trackAbs_ankOut(float phi0, float kp, float kd); // Track absolute orientation of outer feet
void trackAbs_ankInn(float phi1, float kp, float kd); // Track absolute orientation of inner feet
void trackScissor_hip(float rate, float offset, float kp, float kd);  // Swing leg tracks linear function of stance angle

/* Functions to be called during walking. All use gains from LabVIEW
 * and controller setpoints from robotParameters. */
void holdStance_ankOut(void);  // Call to hold the outer foot level on the ground
void holdStance_ankInn(void);  // Call to hold the inner foot level on the ground
void flipUp_ankOut(void);  // Flips the outer feet up for swing phase
void flipUp_ankInn(void);  // Flips the inner feet up for swing phase
void flipDown_ankOut(void); // flips the outer feet down, preparing for heel-strike
void flipDown_ankInn(void); // flips the inner feet down, preparing for heel-strike
void pushOff_ankOut(float push); // push off with outer feet, using both feed-forward and feed-back terms.
void pushOff_ankInn(float push); // push off with inner feet, using both feed-forward and feed-back terms.
void hipGlide(float rate, float offset); // Hip scissor tracking, using labview gains
void hipHold(float qh);  // Hold the hip, using gains from labview

#endif // __MOTORCONTROL_H__
