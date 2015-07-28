#ifndef __FSM_H__
#define __FSM_H__		   

void fsm_init(void);
void fsm_run(void);
void fsm_update(void);
void hip_scissor_track(struct ControllerData * ctrlData, float c0, float c1);
void set_ctrl_data(struct ControllerData * ctrlData, float KP, float KD, float x, float v, float u);
float hip_gravity_compensation(void);

#endif  // __FSM_H__
