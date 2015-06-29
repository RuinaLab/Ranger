#include <mb_includes.h>


#define SCHEDULE_TICK_DIVIDER 1 //number of system jiffies (should be 1 mS) per time slot

const TASK_PTR schedule[]={
   (TASK_PTR)&mb_task_heartbeat,(TASK_PTR)NULL,
   (TASK_PTR)&mb_task_heartbeat,(TASK_PTR)NULL,
   (TASK_PTR)&mb_task_heartbeat,(TASK_PTR)NULL,
   (TASK_PTR)&mb_task_heartbeat,(TASK_PTR)NULL,
   (TASK_PTR)&mb_task_heartbeat,(TASK_PTR)NULL,
   (TASK_PTR)&mb_task_heartbeat,(TASK_PTR)NULL,
   (TASK_PTR)&mb_task_heartbeat,(TASK_PTR)NULL,
   (TASK_PTR)&mb_task_heartbeat,(TASK_PTR)NULL,
   (TASK_PTR)&mb_task_heartbeat,(TASK_PTR)NULL,
   (TASK_PTR)&mb_task_heartbeat,(TASK_PTR)NULL,
   (TASK_PTR)NULL
   };

void mb_setup_software(void){
  /*********************************************
   * Scheduler Initialization.  Do not modify. *
   *********************************************/ 
  mb_schedule_init(schedule,SCHEDULE_TICK_DIVIDER);
 
  /*********************************************
   * Put user initialization code below here.  *
   *********************************************/ 
  mb_set_heartbeat_period_ms(250);
  
  ////////////////////////////////////
  //Set up interrupts
  
  #ifdef DEBUG
  while (P3_INP_STATE & 1<<1)
  {
    P3_OUTP_SET = 1<<26;  // **** TEST CODE **** turn on red LED 
  }
  #endif
  
  //Standard Timer/Counter 0 interrupt setup
  MIC_APR &= ~(1<<16);	//Standard Timer/Counter 0 is active low level (do this _FIRST_)
  MIC_ATR &= ~(1<<16);	//Standard Timer/Counter 0 interrupt is level-actuated
  MIC_ER |= 1<<16;		  //Standard Timer/Counter 0 interrupt enable
  
  //SSP1 interrupt setup
  MIC_APR |= 1<<21;	//Set SSP1 interrupt to active high	(do this _FIRST_)
  MIC_ITR |= 1<<21;	//Set SSP1 interrupt to FIQ
  MIC_ER |= 1<<21;	//Enable SSP1 interrupt (then enable interrupts)
  
  //Start SSP communications
  SSP1CR1 |= 1<<1;   // enable SSP/SPI1. Should be last step of SSP setup
  SSP1DR = 0;         //start SSP send process
}
