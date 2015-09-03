#ifndef __SW_SETUP_H__
#define __SW_SETUP_H__

void setup_software(void);

	// *******************************************************************************
	// Initialize Internal ADC 
	// *******************************************************************************
	
	// Which channels to read from; if you want to read from a channel, set its value to 1
	#define ADCI_CH0_READ 0
	#define ADCI_CH1_READ 1
	#define ADCI_CH2_READ 0
	#define ADCI_CH3_READ 0
	
	// *******************************************************************************
	// Initialize Tic Toc Timer
	// *******************************************************************************
	#define TT_CLOCK T1TC //uses timer 1 (or 0) to measure clock cycles

	// *******************************************************************************
	// Initialize Error Reporting
	// *******************************************************************************
	#define BOARD_NAME BOARD_MC_INVERTATRON
	
//#ifndef TASKS
//#define TASKS

//Task Declarations
//void task(void);
//void task_adci_convert(void);
void task_mc_run(void);
//void task_adcx_convert(void);
void task_every_row(void);
/*
void task_every_second(void);
void task_every_x(void);
void task_every_y(void);
void task_every_z(void);
void task_wait(void);

*/
//#endif







  
  
  


#endif /* __SW_SETUP_H__ */
