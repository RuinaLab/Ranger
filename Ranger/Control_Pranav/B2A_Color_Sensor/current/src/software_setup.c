/*
  
  software_setup.c
  
  Board dependent software setup.

*/

#include <includes.h>

// **********************  Variables  ************************** 
// *******************************************
// Scheduler Variables
// *******************************************
const VOID_VOID_F main_schedule[]=
{
  task_every_row, dn_sched_done,NULL,
  task_every_row, run_occasionally, dn_sched_done,NULL,
  NULL
};

// ********************************* TASKS ********************************** //
void task_every_row(void) 
{ 
  adci_convert_all();
  can_rx_dispatch_all();
  hb_beat();
  mcu_led_update();
  error_update();
  
  can_tx1();    // Send out frames on CAN1
}

//A simple function to run other functions at a slow, not necessarily well-timed rate.
//Add additional else if statements below for each new function that needs to run.
//Be sure that the index i values are continuous from 0 to (number of functions - 1),
//because it will skip any functions after a gap.
void run_occasionally(void)
{
  static short i = 0;
  
  if (i == 0){error_send_next();}                 //Send error from error buffer over the CAN bus
  else if (i == 1){can_tx_status();}              //Send board operation status code to main brain
  else if (i == 2){can_tx_color_white();}
  else if (i == 3){can_tx_color_red();}
  else if (i == 4){can_tx_color_green();}
  else if (i == 5){can_tx_color_blue();}
  else if (i == 6){can_tx_exec_time();}     
  else if (i == 7){can_tx_max_exec_time();}    
  else{i = -1;}
  ++i; 
}


// **********************  Software Initialization  **************************
void init_software(void){

  // *********************************************
  // Standard board initialization. Do not modify. Unless you dare.
  // *********************************************
  asched_init(main_schedule,1);
  error_init(dn_error_transmit, asched_get_timestamp, BOARD_COL);
  hb_init(9, dn_blue_heartbeat, asched_get_timestamp); //heartbeat

  // *********************************************
  // Put user initialization code below here.  
  // *********************************************
  adci_init(ADCI_NULL, ADCI_NO_FILTER, ADCI_NULL, ADCI_NULL); //batt voltage on ch1

  i2c_color_init(); //i2c color sensor initialization
 
}

void init_values(void)
{ //startup calibrations and the like

}




