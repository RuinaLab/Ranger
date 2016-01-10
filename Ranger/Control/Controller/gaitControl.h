#ifndef __GAITCONTROL_H__
#define __GAITCONTROL_H__


/* These are the key variables - they are used to compute the 
 * control actions used by the walking finite state machine */
extern float GAIT_WALK_ANK_PUSH;
extern float GAIT_WALK_CRIT_STANCE_ANGLE;
extern float GAIT_WALK_HIP_STEP_ANGLE;
extern float GAIT_WALK_SCISSOR_GAIN;
extern float GAIT_WALK_SCISSOR_OFFSET;

/* Entry-point functions */
void gaitControl_entry(void);
void gaitControl_main(void);

#endif // __GAITCONTROL_H__
