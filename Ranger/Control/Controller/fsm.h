#ifndef __FSM_H__
#define __FSM_H__		   

void angles_update(void);
void fsm_init(void);
void fsm_run(void);
void fsm_update(void);
float hip_gravity_compensation(void);
void hip_track_rel(struct ControllerData * ctrlData, float qh_ref, float dqh_ref, float KP, float KD);
void hip_scissor_track(struct ControllerData * ctrlData, float c0, float c1, float KP, float KD);
void out_ank_track_abs(struct ControllerData * ctrlData, float phi0_ref, float dphi0_ref, float u_ref, float KP, float KD);
void inn_ank_track_abs(struct ControllerData * ctrlData, float phi1_ref, float dphi1_ref, float u_ref, float KP, float KD);
void hip_scissor_track_outer(struct ControllerData * ctrlData, float offset, float rate, float KP, float KD);
void hip_scissor_track_inner(struct ControllerData * ctrlData, float offset, float rate, float KP, float KD);


void test_foot(void);
void test_fsm(void);
void test_init(void);

#endif  // __FSM_H__
