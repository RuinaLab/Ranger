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
  task_every_row, can_tx_left_ankle_rate, dn_sched_done,NULL,
  task_every_row, can_tx_left_ankle_angle, dn_sched_done,NULL,
  task_every_row, run_occasionally, dn_sched_done,NULL,
  NULL
};

// ********************************* TASKS ********************************** //
void task_every_row(void) 
{ 
  adci_convert_all();
  qdc_tmr0_cap23_angle_update();
  qdc_tmr0_cap23_rate_update();
  ae_update();
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
  else if (i == 2){can_tx_exec_time();}     
  else if (i == 3){can_tx_max_exec_time();}
  else if (i == 4){can_tx_color_white();} 
  else if (i == 5){can_tx_color_red();}
  else if (i == 6){can_tx_color_green();}
  else if (i == 7){can_tx_color_blue();}     
  else{i = -1;}
  ++i; 
}


// **********************  Software Initialization  **************************
void init_software(void){

  // *********************************************
  // Standard board initialization. Do not modify. Unless you dare.
  // *********************************************
  asched_init(main_schedule,1);
  error_init(dn_error_transmit, asched_get_timestamp, BOARD_MCSO);
  hb_init(9, dn_blue_heartbeat, asched_get_timestamp); //heartbeat

  // *********************************************
  // Put user initialization code below here.  
  // *********************************************
  adci_init(ADCI_NULL, ADCI_NO_FILTER, ADCI_NULL, ADCI_NULL); //batt voltage on ch1

  i2c_color_init(); //i2c color sensor initialization

  //Initialize incremental encoder position and rate calculations.
  //Put setup, smoothing/filtering, and calibration values here
  qdc_tmr0_init
  (
    60000000,     //Count frequency of timer 0 in cycles per second
    512,          //Pulses per revolution per channel (1X) of CAP01 encoder
    34,         //Gear ratio to obtain output shaft angle/rate, if desired, for CAP01 encoder.
    0.0005,       //Time per call to qdc_tmr0_cap01_rate_update in seconds
    4,            //Maximum exponential smoothing coefficient exponent for cap01. See instructions.
    4,            //Minimum exponential smoothing coefficient exponent for cap01
    2048,         //Pulses per revolution per channel (1X) of CAP23 encoder
    1.0,          //Gear reduction ratio to obtain output shaft angle/rate, if desired, for CAP23 encoder
    0.0005,       //Timer per call to qdc_tmr1_cap23_rate_update in seconds
    4,           //Maximum exponential smoothing coefficient exponent for cap23  See instructions.
    4             //Minimum exponential smoothing coefficient exponent for cap23
  );

  ae_init_encoder
  (AE_2, 
  2326, 
  186, 
  8192);    
}

void init_values(void)
{ //startup calibrations and the like

}




