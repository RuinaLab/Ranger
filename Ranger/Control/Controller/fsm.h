#ifndef __FSM_H__
#define __FSM_H__		   

void fsm_param_update(void);
void fsm_init(void);
void fsm_run(void);
void fsm_update(void);

void correct_gyro_angle(void);

void test_init(void);
void test_fsm(void);

#endif  // __FSM_H__
