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
void task_can_transmit(void);
void task_swing(void);
void tic(void);
void toc(void);
void print_float(float f);

#endif /* __SW_SETUP_H__ */


