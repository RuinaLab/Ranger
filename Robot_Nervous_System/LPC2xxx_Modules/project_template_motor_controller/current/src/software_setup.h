#ifndef __SW_SETUP_H__
#define __SW_SETUP_H__

void init_software(void);
void init_values(void);
void can_init(void);

//Task Declarations
void task_every_row(void);
void task_can_transmit1(void);
void task_can_transmit2(void);
void task_can_transmit3(void);
void task_can_transmit4(void);
void task_wait(void);

#endif /* __SW_SETUP_H__ */


