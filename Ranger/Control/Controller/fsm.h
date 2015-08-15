#ifndef __FSM_H__
#define __FSM_H__		   

#define ZERO_POS_INN 1.85 //constant offset between absolute and relative ankle angle
#define ZERO_POS_OUT 1.8


void fsm_init(void);
void fsm_run(void);
void fsm_update(void);

void test_fsm_ank(void);
void test_fsm(void);
void test_init(void);
void param_update(void);

#endif  // __FSM_H__
