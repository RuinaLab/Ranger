#include <includes.h>


#define SCHEDULE_TICK_DIVIDER 1 //number of system jiffies (should be 1 mS) per time slot
#define SCHEDULE_TICKS_PER_MS 1 //number of ticks per millisecond

const TASK_PTR schedule[]=
  {
    (TASK_PTR)&run_every_line, (TASK_PTR)&run_occasionally, (TASK_PTR)NULL,
    (TASK_PTR)&run_every_line, (TASK_PTR)NULL,
    (TASK_PTR)&run_every_line, (TASK_PTR)NULL,
    (TASK_PTR)&run_every_line, (TASK_PTR)NULL,
    (TASK_PTR)&run_every_line, (TASK_PTR)NULL,
    (TASK_PTR)&run_every_line, (TASK_PTR)NULL,
    (TASK_PTR)&run_every_line, (TASK_PTR)NULL,
    (TASK_PTR)&run_every_line, (TASK_PTR)NULL,
    (TASK_PTR)&run_every_line, (TASK_PTR)NULL,
    (TASK_PTR)&run_every_line, (TASK_PTR)NULL,
    (TASK_PTR)NULL
  };
  
//////////////////////////////////////////////////////////////////////////////////////////////

void run_every_line(void)
{
  router_timestamp_transmit();
  error_update();
  router_update_can_loading();
  b10a_update_mcu_leds();
  b10a_update_can_leds();
  hb_beat();
}

//A simple function to run other functions at a slow, not necessarily well-timed rate.
//Add additional else if statements below for each new function that needs to run.
//Be sure that the index i values are continuous from 0 to (number of functions - 1),
//because it will skip any functions after a gap.
void run_occasionally(void)
{
  static short i = 0;
  
  if (i == 0)
  {
    error_send_next();  // Send error from error buffer over the CAN bus
  }
  else if (i == 1)
  {
    csr_send_battery_power();
  }
  else if (i == 2)
  {
    csr_send_battery_current();
  }
  else if (i == 3)
  {
    csr_send_battery_voltage();
  }
  else if (i == 4)
  {
    router_can1_load_transmit();
  }
  else if (i == 5)
  {
    router_can2_load_transmit();
  }
  else if (i == 6)
  {
    router_can3_load_transmit();
  }
  else if (i == 7)
  {
    router_can4_load_transmit();
  }
  else
  {
    i = -1;
  }
  ++i; 
}

  
void heartbeat_blink_blue(void)
{
  b10a_mcu_blue_led_blink(50);
}

void setup_software(void)
{
  /*********************************************
   * Scheduler Initialization.  Do not modify. *
   *********************************************/ 
  asched_init(schedule,SCHEDULE_TICK_DIVIDER);
  
 /*********************************************
  * Put user initialization code below here.  *
  *********************************************/
 // error_init(router_error_transmit, asched_get_timestamp, BOARD_CSR);
  error_init(router_error_transmit, csr_elapsed_ms, BOARD_CSR);
  hb_init(9, &heartbeat_blink_blue, csr_elapsed_ms);

  //Zero out global variables. Buffer indexes can be corrupted during debugger operation
  csr_global_variable_init();
   
  // Initialize CAN router board functions and buffers
  router_data_nexus_init();
  
  csr_routing_table_init(); // initialize routing table to default values
    
////////////////////////////////////////////////////////////////////
//Enable interrupts
////////////////////////////////////////////////////////////////////
  VICIntEnable = 0;
  VICVectAddr  = 0;
  VICSoftIntClr = 0xffffffff;  	 //clear all interupts - only do this once
  
  // ************ PRIORITY FIQ ****************** 
  //SSP and associated Timer1 interrupt setup FIQ
  VICIntSelect |= 1 << 11;    //Enable FIQ interrupts for SSP
  VICIntSelect |= 1 << 5;     //Enable FIQ interrupts for Timer1
  VICIntEnable = 1 << 11;     // Enable SSP interrupt
  VICIntEnable = 1 << 5;      // Enable Timer1 interrupt
  T1TCR = 1;                  //Start Timer1
  
  
  
  // ************ PRIORITY 0 ******************
  //Timer0 interrupt setup
  VICVectAddr0 = (unsigned long)timer0_isr;
  VICVectCntl0 = 0x20 | 4; /* Timer0 Interrupt */
  VICIntEnable = 1 << 4;   /* Enable Timer0 Interrupt */
  
  // ************ PRIORITY 1 ******************

  // ************ PRIORITY 2 ******************
  //CAN1 RX
  VICIntSelect |= 1 << VIC_CAN1RX;    //Enable FIQ interrupts
  VICIntEnable = 1 << VIC_CAN1RX;

  // ************ PRIORITY 3 ******************
  //CAN2 RX
  VICIntSelect |= 1 << VIC_CAN2RX;    //Enable FIQ interrupts
  VICIntEnable = 1 << VIC_CAN2RX;

  // ************ PRIORITY 4 ******************
  //CAN3 RX
  VICIntSelect |= 1 << VIC_CAN3RX;    //Enable FIQ interrupts
  VICIntEnable = 1 << VIC_CAN3RX;

  // ************ PRIORITY 5 ******************
  //CAN4 RX
  VICIntSelect |= 1 << VIC_CAN4RX;    //Enable FIQ interrupts
  VICIntEnable = 1 << VIC_CAN4RX;
  
  
  // ************ PRIORITY 6 ******************

  // ************ PRIORITY 7 ******************
  
  // ************ PRIORITY 8 ******************
 
  // ************ PRIORITY 9 ******************
    
  // ************ PRIORITY 10 ******************
  //CAN common (for error handling)
  VICVectAddr10 = (unsigned long)can_error_isr;
  VICVectCntl10 = 0x20 | 19;
  VICIntEnable = 1 << 19;
}
