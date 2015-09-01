#ifndef __WALKCONTROL_H__
#define __WALKCONTROL_H__

extern float PHASE_HIP_ANGLE_START;    // The angle of the hip at the start of the glide phase
extern float PHASE_HIP_ANGLE_FINAL;    // The angle of the hip at the end of the glide phase
extern float PHASE_STANCE_ANGLE_START;    // The angle of the stance leg at the start of the glide phase
extern float PHASE_STANCE_ANGLE_FINAL;    // The angle of the stance leg at the end of the glide phase

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
void hipGlide(void); // Hip scissor tracking, using labview gains
void hipHold(float qh);  // Hold the hip, using gains from labview

/* For unit testing */
void walkControl_test(void);   // Used for unit testing, called by unitTest

/* To be called from top-level fsm */
void walkControl_entry(void);  // called at the start of walking 
void walkControl_main(void);   // called on every loop of walking

#endif
