#ifndef __SW_SETUP_H__
#define __SW_SETUP_H__

void init_software(void);
void init_values(void);

//#ifndef TASKS
//#define TASKS

//Task Declarations
//void task(void);
//void task_adci_convert(void);
void task_mc_run(void);
//void task_adcx_convert(void);
void task_every_row(void);
void task_print_data(void); //ADD:PID
void task_can_transmit1(void);
void task_can_transmit2(void);
void task_can_transmit3(void);
void task_can_transmit4(void);
void task_swing(void);
void task_test_mc(void);
void tic(void);
void toc(void);
void print_float(float f);
void task_test_wait(void);
void task_test_exit(void);

#endif /* __SW_SETUP_H__ */


