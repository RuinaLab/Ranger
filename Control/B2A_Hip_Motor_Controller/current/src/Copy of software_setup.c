/*
  
  software_setup.c
  
  Board-dependent software setup.

*/

#include <includes.h>

// **********************  Variables  **************************
 
// *******************************************
// Schedule
// *******************************************

const VOID_VOID_F main_schedule[]=
{
  task_every_row, can_tx_angle, dn_sched_done,NULL,
  task_every_row, can_tx_angle_rate, dn_sched_done,NULL,
  task_every_row, can_tx_battery_power, dn_sched_done,NULL,  // **** TEST CODE ****
  task_every_row, run_occasionally,dn_sched_done,NULL,
  NULL
};

//Call at every iteration of the scheduler, for a fast update rate
void task_every_row(void)
{ 
  adcx_convert_all();
  adci_convert_all();
  qdc_tmr0_cap01_angle_update();
  qdc_tmr0_cap01_rate_update();
  qdc_tmr0_cap23_angle_update();
  qdc_tmr0_cap23_rate_update();
  ae_update();
  can_rx_dispatch_all();  // Dispatch all incoming frames in rx buffer
  hb_beat();
  mcu_led_update();
  dn_safety();
  error_update();
  can_tx1();    // Send out frames on CAN1
  adcx_conversion_wait();
  
  /** PWM CONTROL **/
//  mc_set_pwm(40);
//  mc_run_no_control();
  
  /** CURRENT CONTROL **/
//  mc_set_target_current(0.1);
//  mc_pid_current();
  /** COMPLIANT CONTROL **/
  mc_compliant_control();
}

//A simple function to run other functions at a slow, not necessarily well-timed rate.
//Add additional else if statements below for each new function that needs to run.
//Be sure that the index i values are continuous from 0 to (number of functions - 1),
//because it will skip any functions after a gap.
void run_occasionally(void)
{
  static short i = 0;
  if (i == 0){error_send_next();}                 // Send error from error buffer over the CAN bus
  else if (i == 1){can_tx_motor_position();}      // Send motor angle over the CAN bus
  else if (i == 2){can_tx_motor_current();}       // Send motor current over the CAN bus
  else if (i == 3){can_tx_battery_power();}       // Send motor controller input power over the CAN bus
  else if (i == 4){can_tx_board_status();}        // Send board operation status code to main brain
  else if (i == 5){can_tx_battery_current();}     // Send motor controller input power over the CAN bus
  else if (i == 6){can_tx_battery_voltage();}     // Send motor controller input power over the CAN bus
  else if (i == 7){can_tx_motor_velocity();}      //Send hip angular rate as calculated from motor encoder
  else if (i == 8){dn_hip_motor_encoder_rezero();}//Adjust hip motor encoder offset slightly to keep aligned with hip joint encoder
  else if (i == 9){can_tx_exec_time();}     
  else if (i == 10){can_tx_max_exec_time();}   

  //Restart the list - don't change this line  
  else{i = -1;}
  ++i; 
}

// **********************  Software Initialization  **************************
void init_software(void){

  // *********************************************
  // Standard board initialization. Do not modify. Unless you dare.
  // *********************************************
  asched_init(main_schedule,1);
  error_init(dn_error_transmit, asched_get_timestamp, BOARD_MCH);
  hb_init(9, dn_blue_heartbeat, asched_get_timestamp); //heartbeat

  // *********************************************
  // Put user initialization code below here.  
  // *********************************************
  adci_init(ADCI_NULL, ADCI_NO_FILTER, ADCI_NULL, ADCI_NULL); //batt voltage on
  adcx_init(); 
  adcx_add_config( ADCX_CH_P4N5, ADCX_GAIN_2); //motor current
  adcx_add_config( ADCX_CH_P2N3, ADCX_GAIN_2); //battery current

  qdc_tmr0_init
  (
    60000000,     //Count frequency of timer 0 in cycles per second
    2048,          //Pulses per revolution per channel (1X) of CAP01 encoder
    1.0,         //Gear ratio to obtain output shaft angle/rate, if desired, for CAP01 encoder.
    0.0005,       //Time per call to qdc_tmr0_cap01_rate_update in seconds
    4,            //Maximum exponential smoothing coefficient exponent for cap01. See instructions.
    4,            //Minimum exponential smoothing coefficient exponent for cap01
    192,         //Pulses per revolution per channel (1X) of CAP23 encoder
    66.0,          //Gear reduction ratio to obtain output shaft angle/rate, if desired, for CAP23 encoder
    0.0005,       //Timer per call to qdc_tmr1_cap23_rate_update in seconds
    4,           //Maximum exponential smoothing coefficient exponent for cap23  See instructions.
    4             //Minimum exponential smoothing coefficient exponent for cap23
  );
 
  //Motor controller setup and initialization
  mc_init
  (
    12.0, 
    5.0, 
    -5.0, 
    4.0, 
    -4.0, 
    3.1, 
    10.0,
    0.0,  // **** TEST CODE **** lower Kp
    //220.0, 
    20.0, 
    0.0, 
    250, 
    4, 
    20.0, 
    MC_NORMAL, 
    &dn_get_raw_motor_current, 
    &dn_get_shaft_pos_rads, 
    &dn_get_motor_vel_rads
  );

  ae_init_encoder(AE_1,4590,94,8192); //abs encoder for hip angle is on J3

  mc_set_stiffness(0.0);
  mc_set_dampness(0.0);  
  mc_set_command_current(0.0);
  
}

void init_values(void)
{ //startup calibrations and the like
  ae_update();
  ae_wait();
  qdc_tmr0_cap01_set_angle_float(dn_get_shaft_pos_rads());
}

