#include <mb_includes.h>


#define SCHEDULE_TICK_DIVIDER 1 //number of system jiffies (should be 1 mS) per time slot

const TASK_PTR schedule[]={
 //  (TASK_PTR)&mb_task_heartbeat, (TASK_PTR)&test_frame_sender, (TASK_PTR)NULL,
   (TASK_PTR)&mb_task_heartbeat, (TASK_PTR)&mb_ssp_pop_frames,(TASK_PTR)NULL,
   (TASK_PTR)&mb_task_heartbeat, (TASK_PTR)&mb_ssp_pop_frames,(TASK_PTR)NULL,
   (TASK_PTR)&mb_task_heartbeat, (TASK_PTR)&mb_ssp_pop_frames,(TASK_PTR)NULL,
   (TASK_PTR)&mb_task_heartbeat, (TASK_PTR)&mb_ssp_pop_frames,(TASK_PTR)NULL,
   (TASK_PTR)&mb_task_heartbeat, (TASK_PTR)&mb_ssp_pop_frames,(TASK_PTR)NULL,
   (TASK_PTR)&mb_task_heartbeat, (TASK_PTR)&mb_ssp_pop_frames,(TASK_PTR)NULL,
   (TASK_PTR)&mb_task_heartbeat, (TASK_PTR)&mb_ssp_pop_frames,(TASK_PTR)NULL,
   (TASK_PTR)&mb_task_heartbeat, (TASK_PTR)&mb_ssp_pop_frames,(TASK_PTR)NULL,
   (TASK_PTR)&mb_task_heartbeat, (TASK_PTR)&mb_ssp_pop_frames,(TASK_PTR)NULL,
   (TASK_PTR)&mb_task_heartbeat, (TASK_PTR)&mb_ssp_pop_frames,(TASK_PTR)NULL,
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
  
  ////////////////////////////////////////////////
  
  // **** TEST CODE **** initialize variable pointer array with temp array locations
  mb_csr_dma_var_pointer_init();
  
  //Set up interrupts
  mb_csr_gpdma_rx0_isr();
  mb_csr_gpdma_tx1_isr();
}
