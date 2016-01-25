/*
  
  software_setup.c
  
  Board-dependent software setup.

*/

#include <includes.h>

// **********************  Variables  ************************** 
// *******************************************
// Scheduler Variables
// *******************************************
const VOID_VOID_F main_schedule[]=
{
  task_every_row, can_transmit1, dn_sched_done, NULL,
  task_every_row, can_transmit2, dn_sched_done, NULL,
  task_every_row, can_transmit3, dn_sched_done, NULL,
  task_every_row, run_occasionally, dn_sched_done, NULL,
  (TASK_PTR)NULL
};

void task_every_row(void) 
{ 
  adcx_convert_all();
  adci_convert_all();
  can_rx_dispatch_all();
  hb_beat();
  mcu_led_update();
//  dn_safety();
  error_update();
  can_tx2();
  adcx_conversion_wait();

  dn_update_pwm();
//  mc_compliant_control();
}

//A simple function to run other functions at a slow, not necessarily well-timed rate.
//Add additional else if statements below for each new function that needs to run.
//Be sure that the index i values are continuous from 0 to (number of functions - 1),
//because it will skip any functions after a gap.
void run_occasionally(void)
{
  static short i = 0;
  
  if (i == 0){error_send_next();}             // Send error from error buffer over the CAN bus
  else if (i == 1){can_transmit_status();}    // Send board operation status code to main brain
  else if (i == 2){can_transmit4();}          // Send execution time over the CAN bus
  else if (i == 3){can_transmit5();}          // Send max overall execution time over the CAN bus
//  else if (i == 4){can_tx_board_status();}      
//  else if (i == 5){can_tx_batt_current();}     // Send battery current over the CAN bus
//  else if (i == 6){can_tx_batt_voltage();}     // Send battery voltage over the CAN bus
//  else if (i == 7){can_tx_hbridge_temp();}     // Send hbridge temp over the CAN bus
//  else if (i == 10){can_tx_exec_time();}     
//  else if (i == 11){can_tx_max_exec_time();}     
  else{i = -1;}
  ++i; 
}

// **********************  Software Initialization  **************************
void init_software(void){

  // *********************************************
  // Standard board initialization. Do not modify. Unless you dare.
  // *********************************************
  asched_init(main_schedule,1);
  error_init(dn_error_transmit, asched_get_timestamp, BOARD_MCSI);
  hb_init(9, dn_blue_heartbeat, asched_get_timestamp); //heartbeat

  // *********************************************
  // Put user initialization code below here.  
  // *********************************************
  adci_init(ADCI_NULL, ADCI_NO_FILTER, ADCI_NULL, ADCI_NULL); //batt voltage on ch1
  adcx_init(); 
  adcx_add_config( ADCX_CH_0, ADCX_GAIN_2); //steer angle from potentiometer
  adcx_add_config( ADCX_CH_P4N5, ADCX_GAIN_2); //motor current
  adcx_add_config( ADCX_CH_P2N3, ADCX_GAIN_20); //battery current
  mc_init
  (
    12.0, 
    0.3, 
    -0.3, 
    0.25, 
    -0.25, 
    0.3, 
    10.1,
    0.0,  // **** TEST CODE **** Set Kp to zero 
    //220.0, 
    20.0, 
    0.0, 
    200, 
    4, 
    10.0, 
    MC_NORMAL, 
    &dn_get_raw_motor_current, 
    &dn_get_steer_angle, 
    &floatvoid_noerror
  );

  mc_set_stiffness(0.0); //0.001
  mc_set_dampness(0.0);
  mc_set_command_current(0.0); //+-0.175
}

/*********************  Value Initialization  **************************/
void init_values(void){ //startup calibrations and the like
  extern int dn_steer_zero_offset;
  adcx_convert_all();
  adcx_conversion_wait();
  dn_steer_zero_offset = dn_get_steer_angle_raw();
}
