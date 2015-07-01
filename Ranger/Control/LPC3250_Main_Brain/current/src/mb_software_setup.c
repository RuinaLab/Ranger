#include <mb_includes.h>

#define SCHEDULE_TICK_DIVIDER 2 //number of system jiffies (should be 1 mS) per time slot


//#define RUN_EVERY_LINE (TASK_PTR)&mb_update_elapsed_time, (TASK_PTR)&mb_error_update, (TASK_PTR)&mb_task_heartbeat, (TASK_PTR)&a9_dn_ssp_parse, (TASK_PTR)&mb_estimator_update, (TASK_PTR)&a9_dn_ssp_send_data, (TASK_PTR)&a9_dn_update_leds, (TASK_PTR)&mb_fsm_run, (TASK_PTR)&mb_send_data
//#define RUN_EVERY_LINE (TASK_PTR)&mb_error_update, (TASK_PTR)&mb_task_heartbeat, (TASK_PTR)&a9_dn_ssp_parse, (TASK_PTR)&a9_dn_ssp_send_data, (TASK_PTR)&a9_dn_update_leds, (TASK_PTR)&mb_fsm_run, (TASK_PTR)&mb_send_data


// You Use this
// #define RUN_EVERY_LINE (TASK_PTR)&mb_error_update, (TASK_PTR)&mb_task_heartbeat, (TASK_PTR)&a9_dn_ssp_parse, (TASK_PTR)&a9_dn_ssp_send_data, (TASK_PTR)&a9_dn_update_leds, (TASK_PTR)&mb_fsm_run, (TASK_PTR)&mb_send_data
//#define RUN_EVERY_LINE (TASK_PTR)&mb_error_update, (TASK_PTR)&mb_task_heartbeat, (TASK_PTR)&a9_dn_ssp_parse, (TASK_PTR)&a9_dn_ssp_send_data, (TASK_PTR)&a9_dn_update_leds, (TASK_PTR)&mb_fsm_run, (TASK_PTR)&mb_send_data


//I USE THIS: ANOOP   (it has estimator update, otherwise its same)
/*YW:this calls the FSM*/
#define RUN_EVERY_LINE (TASK_PTR)&mb_error_update, (TASK_PTR)&mb_task_heartbeat, (TASK_PTR)&a9_dn_ssp_parse, (TASK_PTR)&mb_estimator_execution_time_start , (TASK_PTR)&mb_estimator_update, (TASK_PTR)&mb_estimator_execution_time_stop, (TASK_PTR)&a9_dn_ssp_send_data, (TASK_PTR)&a9_dn_update_leds, (TASK_PTR)&mb_fsm_run, (TASK_PTR)&mb_send_data

/*YW:this calls the simple controller*/
//#define RUN_EVERY_LINE (TASK_PTR)&mb_error_update, (TASK_PTR)&mb_task_heartbeat, (TASK_PTR)&a9_dn_ssp_parse, (TASK_PTR)&mb_estimator_execution_time_start , (TASK_PTR)&mb_estimator_update, (TASK_PTR)&mb_estimator_execution_time_stop, (TASK_PTR)&a9_dn_ssp_send_data, (TASK_PTR)&a9_dn_update_leds, (TASK_PTR)&control_run, (TASK_PTR)&mb_send_data

const TASK_PTR schedule[]={

  RUN_EVERY_LINE, (TASK_PTR)&mb_bt_pop_receive_packets,     (TASK_PTR)NULL,
  RUN_EVERY_LINE, (TASK_PTR)&a9_bt_dma_receive,             (TASK_PTR)NULL,
  RUN_EVERY_LINE, (TASK_PTR)&mb_create_display_data_lists,  (TASK_PTR)NULL,
  RUN_EVERY_LINE, (TASK_PTR)&mb_distribute_error_frames, (TASK_PTR)NULL,
 // RUN_EVERY_LINE, (TASK_PTR)NULL,
  RUN_EVERY_LINE, (TASK_PTR)&mb_test_ui_board1,               (TASK_PTR)NULL,
  RUN_EVERY_LINE, (TASK_PTR)NULL,
  RUN_EVERY_LINE, (TASK_PTR)NULL,
  RUN_EVERY_LINE, (TASK_PTR)NULL,
  RUN_EVERY_LINE, (TASK_PTR)NULL,
  RUN_EVERY_LINE, (TASK_PTR)NULL,
  RUN_EVERY_LINE, (TASK_PTR)&mb_update_execution_time, (TASK_PTR)NULL,
  (TASK_PTR)NULL  //don't erase, this is the stop point
   };
   

  
//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Software setup of PC/laptop data display transmitter/receiver
//////////////////////////////////////////////////////////////////////////////////////////////////////////

//Global arrays to contain data lists for medium and fast transmissions to PC/laptop display
//Array size must be _at least_ one greater than the number of LabView data channels.  
unsigned short mb_medium_data_display_list[ID_LV_CH_47 - ID_LV_CH_0 + 2];
unsigned short mb_fast_data_display_list[ID_LV_CH_47 - ID_LV_CH_0 + 2];

/*
// Wrapper function to call the elapsed time function
// and update the mb_io_data array value 0
void mb_update_elapsed_time(void)
{
  mb_io_set_float(ID_TIMESTAMP, (float)mb_clock_get_time());
}
*/

// Wrapper function to get the value of ID_TIMESTAMP
unsigned long mb_get_timestamp(void)
{
  return mb_io_get_time(ID_TIMESTAMP);
}

// Wrapper function to call the execution time function
void mb_update_execution_time(void)
{
  mb_io_set_float(ID_MB_EXECUTION_TIME, mb_clock_get_execution_time());
}

// Wrapper functions to call C functions from C++

float get_io_float(unsigned short data_id)
{
  return mb_io_get_float(data_id); 
}

void set_io_float(short unsigned int data_id, float value)
{
  mb_io_set_float(data_id, value);
}

float get_io_ul(unsigned short data_id)
{
  return mb_io_get_ul(data_id); 
}



void set_io_ul(short unsigned int data_id, unsigned long value)
{
  mb_io_set_ul(data_id, value);
}

void mark_as_read(unsigned short data_id, unsigned short subscriber_id)
{
  mb_io_mark_as_read(data_id, subscriber_id);
}

//Returns 0 if data has not been read by the subscriber (data is new), 1 if already read
unsigned short data_was_read(unsigned short data_id, unsigned short subscriber_id)
{
  return mb_io_data_was_read(data_id, subscriber_id);
}

// Wrapper function to write error, if any, into IO_DATA array Main Brain error location
void mb_read_error(void)
{
  mb_error_get_frame(mb_io_get_pointer(ID_ERROR_MB));
}
   
// Wrapper function to call the data list generation function.
// Call during power-up initialization and also occasionally (~100 mS)
// to allow run-time changes to data transmit schedule
void mb_create_display_data_lists(void)
{
  mb_bt_create_data_lists(
    ID_LV_CH_0,                     //First LabView channel data_id to search
    ID_LV_CH_47,                    //Last LabView channel data_id to search
    mb_medium_data_display_list,    //Pointer to array to hold list; size = # channels + 1
    mb_fast_data_display_list,      //Pointer to array to hold list; size = # channels + 1
    mb_io_get_float            //Pointer to function to get value at data_id
  );
}

// Wrapper function to call data send function -
// sends data to laptop/PC data display according to schedule defined in lists.
void mb_send_data(void)
{
  a9_bt_data_sender(
  ID_LAST,                       //Highest data_id to send at slow rate (0 is lowest)
  LABVIEW,                     //Unique process identifier, to determine if data has yet been read, range 0 - 31
  mb_medium_data_display_list,   //Pointer to array (list) of data_ids to send at medium rate                                //
  mb_fast_data_display_list     //Pointer to array (list) of data_ids to send at fast rate
  );    
}

///////////////////////////////////////////////////////////////////////////////////////////////
void mb_setup_software(void){
  /*********************************************
   * Scheduler Initialization.  Do not modify. *
   *********************************************/ 
  mb_schedule_init(schedule,SCHEDULE_TICK_DIVIDER);
 
  /*********************************************
   * Put user initialization code below here.  *
   *********************************************/ 
  mb_set_heartbeat_period_ms(250);
  
  
  //Initialize IO data array
  mb_io_init();
  
  //Initialize data display lists
  mb_create_display_data_lists();
  
  //Initialize finite state machine (controller)
  mb_fsm_init();

  //Initialize error handling module
  mb_error_init(mb_get_timestamp, BOARD_MB);
  
  ////////////////////////////////////////////////
  
  // Initialize Main Brain transmit list. These are values sent from the main brain to the satellites
  // via the SSP port.
  //a9_dn_ssp_init_tx_list();
 
  ////////////////////////////////////////////////////////////////////////////////////
  // ARM7 synchronization code
  // Run this in software setup immediately before enabling interrupts
  // Execution will halt until two ready codes have been exchanged with the ARM7 board
  // via SSP1.
  ////////////////////////////////////////////////////////////////////////////////////
  //a9_ssp_synchronize_arm7();
  ////////////////////////////////////////////////////////////////////////////////////
  
  SSP1CR1 = (unsigned char)((SSP1CR1 & ~(1<<1)) & (0xF));   // disable SSP/SPI1
  SSP1DMACR = 3;	 //Enable transmit and receive FIFO for DMA 
    
  //Set up IRQ interrupts for Timer/Counter 0
  MIC_APR = MIC_APR & ~(1<<16) & MIC_APR_RBMASK;	//Standard Timer/Counter 0 is active low
  MIC_ATR = MIC_ATR & ~(1<<16) & MIC_ATR_RBMASK;	//Standard Timer/Counter 0 interrupt is level-actuated
  MIC_ITR = MIC_ITR & ~(1<<16) & MIC_ITR_RBMASK;	//Standard Timer/Counter 0 interrupt is IRQ
  MIC_ER |= 1<<16;		//Standard Timer/Counter 0 interrupt enable
  T0TCR = 1<<0;		//Enable counter 
  
  //Set up GPDMA IRQ interrupts
  MIC_APR = (MIC_APR |(1<<28)) & MIC_APR_RBMASK;	//GPDMA interrupt is active high level
  MIC_ATR = MIC_ATR & ~(1<<28) & MIC_ATR_RBMASK;	//GPDMA interrupt is level-actuated
  MIC_ITR = MIC_ITR & ~(1<<28) & MIC_ITR_RBMASK;	//GPDMA interrupt is IRQ
  MIC_ER = (MIC_ER | (1<<28)) & MIC_ER_RBMASK;		//enable GPDMA interrupts
  
  //Set up software interrupts
  SIC1_APR = (SIC1_APR | (1<<24)) & SIC1_APR_RBMASK;  //Software interrupt is active high
  SIC1_ATR = SIC1_ATR & ~(1<<24) & SIC1_ATR_RBMASK;   //Software interrupt is level-actuated
  SIC1_ITR = SIC1_ITR & ~(1<<24) & SIC1_ITR_RBMASK;   //Software interrupt is IRQ
  SIC1_ER = (SIC1_ER | (1<<24)) & SIC1_ER_RBMASK;     //Enable software interrupt in subinterrupt controller 1
  
  MIC_APR = MIC_APR & ~(1<<0) & MIC_APR_RBMASK;	//SIC1 interrupt is active low
  MIC_ATR = MIC_ATR & ~(1<<0) & MIC_ATR_RBMASK;	//SIC1 interrupt is level-actuated
  MIC_ITR = MIC_ITR & ~(1<<0) & MIC_ITR_RBMASK;	//SIC1 interrupt is IRQ
  MIC_ER = (MIC_ER | (1<<0)) & MIC_ER_RBMASK;		//enable SIC1 interrupts
  
  HSU1_IIR = 1<<6;    //Force HSU 1 transmit interrupt
  HSU2_IIR = 1<<6;    //Force HSU 2 transmit interrupt
  
  //Set up FIQ interrupts on SSP1 SSEL1 (GPIO4)
  MIC_ITR = (MIC_ITR | ((unsigned)1<<31)) & MIC_ITR_RBMASK;   //Set bit 31 to enable SIC2 FIQ interrupts
  MIC_ATR = (MIC_ATR & ~((unsigned)1<<31)) & MIC_ATR_RBMASK;  //clear bit 31 for level-sense SIC2 FIQ
  MIC_APR = (MIC_APR & ~((unsigned)1<<31)) & MIC_APR_RBMASK;  //clear bit 31 for active-low SIC2 FIQ
  MIC_ER  = (MIC_ER  | ((unsigned)1<<31)) & MIC_ER_RBMASK;    //Set bit 31 to enable SIC2 FIQ interrupts 
 
  SIC2_ITR = (SIC2_ITR | (1<<4)) & SIC2_ITR_RBMASK;   //Set bit 4 to enable GPIO4 FIQ interrupts
  SIC2_ATR = (SIC2_ATR | (1<<4)) & SIC2_ATR_RBMASK;   //Set bit 4 for edge-sense GPIO4 FIQ
  SIC2_APR = (SIC2_APR & ~(1<<4)) & SIC2_APR_RBMASK;   //Set bit 4 for falling-edge GPIO4 FIQ
  SIC2_ER  = (SIC2_ER  | (1<<4)) & SIC2_ER_RBMASK;    //Set bit 4 to enable GPIO4 FIQ interrupts 
  
  SSP1CR1 = (unsigned char)((SSP1CR1 | (1<<1)) & (0xF));   // enable SSP/SPI1
}
