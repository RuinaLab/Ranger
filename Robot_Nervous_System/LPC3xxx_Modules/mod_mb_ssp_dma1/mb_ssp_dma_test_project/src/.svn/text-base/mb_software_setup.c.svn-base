#include <mb_includes.h>


#define SCHEDULE_TICK_DIVIDER 1 //number of system jiffies (should be 1 mS) per time slot

#define RUN_EVERY_LINE (TASK_PTR)&mb_task_heartbeat, (TASK_PTR)&a9_dn_ssp_parse
const TASK_PTR schedule[]={
 //  (TASK_PTR)&mb_task_heartbeat, (TASK_PTR)&test_frame_sender, (TASK_PTR)NULL,
 // (TASK_PTR)&mb_task_heartbeat, (TASK_PTR)&a9_dn_ssp_parse,(TASK_PTR)NULL,
  RUN_EVERY_LINE, (TASK_PTR)&a9_dn_ssp_send_data, (TASK_PTR)NULL,
  RUN_EVERY_LINE, (TASK_PTR)NULL,
  RUN_EVERY_LINE, (TASK_PTR)NULL,
  RUN_EVERY_LINE, (TASK_PTR)NULL,
  RUN_EVERY_LINE, (TASK_PTR)NULL,
  RUN_EVERY_LINE, (TASK_PTR)NULL,
  RUN_EVERY_LINE, (TASK_PTR)NULL,
  RUN_EVERY_LINE, (TASK_PTR)NULL,
  RUN_EVERY_LINE, (TASK_PTR)NULL,
  RUN_EVERY_LINE, (TASK_PTR)NULL,
  (TASK_PTR)NULL  //don't erase, this is the stop point
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
  a9_dn_ssp_init_tx_list();
  
  //Set up FIQ interrupts on SSP1 SSEL1 (GPIO4)
  MIC_ITR = (MIC_ITR | ((unsigned)1<<31)) & MIC_ITR_RBMASK;   //Set bit 31 to enable SIC2 FIQ interrupts
  MIC_ATR = (MIC_ATR & ~((unsigned)1<<31)) & MIC_ATR_RBMASK;  //clear bit 31 for level-sense SIC2 FIQ
  MIC_APR = (MIC_APR & ~((unsigned)1<<31)) & MIC_APR_RBMASK;  //clear bit 31 for active-low SIC2 FIQ
  MIC_ER  = (MIC_ER  | ((unsigned)1<<31)) & MIC_ER_RBMASK;    //Set bit 31 to enable SIC2 FIQ interrupts 
 
  SIC2_ITR = (SIC2_ITR | (1<<4)) & SIC2_ITR_RBMASK;   //Set bit 4 to enable GPIO4 FIQ interrupts
  SIC2_ATR = (SIC2_ATR | (1<<4)) & SIC2_ATR_RBMASK;   //Set bit 4 for edge-sense GPIO4 FIQ
//  SIC2_APR = (SIC2_APR | (1<<4)) & SIC2_APR_RBMASK;   //Set bit 4 for rising-edge GPIO4 FIQ
  SIC2_APR = (SIC2_APR & ~(1<<4)) & SIC2_APR_RBMASK;   //Set bit 4 for falling-edge GPIO4 FIQ
  SIC2_ER  = (SIC2_ER  | (1<<4)) & SIC2_ER_RBMASK;    //Set bit 4 to enable GPIO4 FIQ interrupts 
  
  SSP1CR1 = (unsigned char)((SSP1CR1 | (1<<1)) & (0xF));   // enable SSP/SPI1
  
//  mb_csr_gpdma_rx0_isr();
 // mb_csr_gpdma_tx1_isr();
}
