/*
	can_setup.c
	
	All of the CAN receives and transmits for the board.
	
	Nicolas Williamson - March 2010
*/

#include <includes.h>

/*********************  CAN Initialization  **************************/

// ***** RX Frames
#define RX_LIST_SIZE 15
CAN_FRAME_DESC * rxlist[RX_LIST_SIZE];

// ***** RTR Frames
#define RTR_LIST_SIZE 10
CAN_FRAME_DESC * rtrlist[RTR_LIST_SIZE];

// ***** Transmit
CAN_FRAME_DESC tx_fd_error;
CAN_FRAME_DESC tx_fd_motor_pos;
CAN_FRAME_DESC tx_fd_motor_vel;
CAN_FRAME_DESC tx_fd_motor_current;
CAN_FRAME_DESC tx_fd_angle;
CAN_FRAME_DESC tx_fd_rate;
CAN_FRAME_DESC tx_fd_batt_power;
CAN_FRAME_DESC tx_fd_ls_right;
CAN_FRAME_DESC tx_fd_ls_left;
CAN_FRAME_DESC tx_fd_hs_right;
CAN_FRAME_DESC tx_fd_hs_left;
CAN_FRAME_DESC tx_fd_status;
CAN_FRAME_DESC tx_fd_batt_current;
CAN_FRAME_DESC tx_fd_batt_voltage;
CAN_FRAME_DESC tx_fd_hbridge_temp;
CAN_FRAME_DESC tx_fd_right_heel_sense;
CAN_FRAME_DESC tx_fd_left_heel_sense;
CAN_FRAME_DESC tx_fd_exec_time;
CAN_FRAME_DESC tx_fd_max_exec_time;

// ***** Receive
CAN_FRAME_DESC rx_fd_timestamp;
CAN_FRAME_DESC rx_fd_mb_ready;
CAN_FRAME_DESC rx_fd_start;
CAN_FRAME_DESC rx_fd_motor_current;
CAN_FRAME_DESC rx_fd_mc_shutdown;
CAN_FRAME_DESC rx_fd_mc_sleep;
CAN_FRAME_DESC rx_fd_command_current;
CAN_FRAME_DESC rx_fd_stiffness;
CAN_FRAME_DESC rx_fd_dampness;


void init_can(void)
{
  // ************* First, fill regular RX descriptors ***************** //  
  int i = 0;

  //Timestamp
  rxlist[i++] = &rx_fd_timestamp;  
  can_set_rx_descriptor_fi(&rx_fd_timestamp, ID_TIMESTAMP, CHAN_CAN2, can_rx_setter_float_dummy, dn_rx_timestamp);
  //Command Current for compliant control
  rxlist[i++] = &rx_fd_command_current;  
  can_set_rx_descriptor_fi(&rx_fd_command_current, ID_MCFI_COMMAND_CURRENT, CHAN_CAN2, mc_set_command_current, can_rx_setter_int_dummy);
  //Stiffness
  rxlist[i++] = &rx_fd_stiffness;  
  can_set_rx_descriptor_fi(&rx_fd_stiffness, ID_MCFI_STIFFNESS, CHAN_CAN2, mc_set_stiffness, can_rx_setter_int_dummy);
  //Dampness
  rxlist[i++] = &rx_fd_dampness;  
  can_set_rx_descriptor_fi(&rx_fd_dampness, ID_MCFI_DAMPNESS, CHAN_CAN2, mc_set_dampness, can_rx_setter_int_dummy);
  //Motor Control - target current
  rxlist[i++] = &rx_fd_motor_current;  
  can_set_rx_descriptor_fi(&rx_fd_motor_current, ID_MCFI_MOTOR_TARGET_CURRENT, CHAN_CAN2, mc_set_target_current, can_rx_setter_int_dummy);
  //Motor Shutdown
  rxlist[i++] = &rx_fd_mc_shutdown;  
  can_set_rx_descriptor_ii(&rx_fd_mc_shutdown, ID_MCFI_SHUTDOWN, CHAN_CAN2, mc_set_shutdown, can_rx_setter_int_dummy);
  //Motor Sleep
  rxlist[i++] = &rx_fd_mc_sleep;  
  can_set_rx_descriptor_ii(&rx_fd_mc_sleep, ID_MCFI_SLEEP, CHAN_CAN2, mc_set_sleep, can_rx_setter_int_dummy);

  //NULL Terminator
  rxlist[i++] = (CAN_FRAME_DESC *) 0;  
  if(i >= RX_LIST_SIZE) 
  {
    error_occurred(ERROR_CAN_RXLST_OF);
  }

  // ************* NOW, fill RTR descriptors ************************** //
  i = 0;

  //NULL Terminator
  rtrlist[i++] = (CAN_FRAME_DESC *) 0;
  if(i >= RTR_LIST_SIZE) {
    error_occurred(ERROR_CAN_RTRLST_OF);
  }

  // ************** NOW, fill standard non-RTR TX descriptors (not in a list) *** //
  can_set_tx_descriptor_ii(&tx_fd_error, ID_ERROR_MCFI, CHAN_CAN2, error_get_info, error_get_time); 
  can_set_tx_descriptor_fi(&tx_fd_motor_pos, ID_MCFI_MOTOR_POSITION, CHAN_CAN2, dn_get_motor_pos_rads, asched_get_timestamp);
  can_set_tx_descriptor_fi(&tx_fd_motor_vel, ID_MCFI_MOTOR_VELOCITY, CHAN_CAN2, dn_get_motor_vel_rads, asched_get_timestamp);
  can_set_tx_descriptor_fi(&tx_fd_motor_current, ID_MCFI_MOTOR_CURRENT, CHAN_CAN2, dn_get_motor_current, asched_get_timestamp);
  can_set_tx_descriptor_fi(&tx_fd_angle, ID_MCFI_MID_ANKLE_ANGLE, CHAN_CAN2, dn_get_ankle_pos_rads, asched_get_timestamp);
  can_set_tx_descriptor_fi(&tx_fd_rate, ID_MCFI_ANKLE_RATE, CHAN_CAN2, dn_get_ankle_vel_rads, asched_get_timestamp);
  can_set_tx_descriptor_fi(&tx_fd_batt_power, ID_MCFI_BATT_POWER, CHAN_CAN2, dn_get_battery_power, asched_get_timestamp);
  can_set_tx_descriptor_fi(&tx_fd_ls_right, ID_MCFI_RIGHT_LS, CHAN_CAN2, dn_get_right_ls, asched_get_timestamp);
  can_set_tx_descriptor_fi(&tx_fd_ls_left, ID_MCFI_LEFT_LS, CHAN_CAN2, dn_get_left_ls, asched_get_timestamp);
  can_set_tx_descriptor_fi(&tx_fd_hs_right, ID_MCFI_RIGHT_HS, CHAN_CAN2, dn_get_right_hs, asched_get_timestamp);
  can_set_tx_descriptor_fi(&tx_fd_hs_left, ID_MCFI_LEFT_HS, CHAN_CAN2, dn_get_left_hs, asched_get_timestamp);
  can_set_tx_descriptor_fi(&tx_fd_status, ID_MCFI_STATUS, CHAN_CAN2, dn_get_status, asched_get_timestamp);
  can_set_tx_descriptor_fi(&tx_fd_batt_current, ID_MCFI_BATT_CURRENT, CHAN_CAN2, dn_get_battery_current, asched_get_timestamp);
  can_set_tx_descriptor_fi(&tx_fd_batt_voltage, ID_MCFI_BATT_VOLTAGE, CHAN_CAN2, dn_get_battery_voltage, asched_get_timestamp);
  can_set_tx_descriptor_fi(&tx_fd_hbridge_temp, ID_MCFI_HBRIDGE_TEMP, CHAN_CAN2, dn_get_hbridge_temp, asched_get_timestamp);
  can_set_tx_descriptor_fi(&tx_fd_right_heel_sense, ID_MCFI_RIGHT_HEEL_SENSE, CHAN_CAN2, dn_get_right_heel_sense, asched_get_timestamp);
  can_set_tx_descriptor_fi(&tx_fd_left_heel_sense, ID_MCFI_LEFT_HEEL_SENSE, CHAN_CAN2, dn_get_left_heel_sense, asched_get_timestamp); 
  can_set_tx_descriptor_fi(&tx_fd_exec_time, ID_MCFI_EXECUTION_TIME, CHAN_CAN2, dn_get_execution_time, asched_get_timestamp);
  can_set_tx_descriptor_fi(&tx_fd_max_exec_time, ID_MCFI_MAX_EXECUTION_TIME, CHAN_CAN2, dn_get_max_execution_time, asched_get_timestamp);  

  // ******** Set CAN rx descriptors ***********
  can_rx_set_descriptors(rxlist, rtrlist);

  // ******** Initialize low-level CAN RX struct for assembly function *********
  can_init();
}

//Wrapper functions for calling CAN transmit from the scheduler
void can_tx_motor_pos(void){can_transmit(&tx_fd_motor_pos);}
void can_tx_motor_vel(void){can_transmit(&tx_fd_motor_vel);}
void can_tx_motor_current(void){can_transmit(&tx_fd_motor_current);}
void can_tx_angle(void){can_transmit(&tx_fd_angle);}
void can_tx_rate(void){can_transmit(&tx_fd_rate);}
void can_tx_batt_power(void){can_transmit(&tx_fd_batt_power);}
void can_tx_ls_right(void){can_transmit(&tx_fd_ls_right);}
void can_tx_ls_left(void){can_transmit(&tx_fd_ls_left);}
void can_tx_hs_right(void){can_transmit(&tx_fd_hs_right);}
void can_tx_hs_left(void){can_transmit(&tx_fd_hs_left);}
void can_tx_board_status(void){can_transmit(&tx_fd_status);}
void can_tx_batt_current(void){can_transmit(&tx_fd_batt_current);}
void can_tx_batt_voltage(void){can_transmit(&tx_fd_batt_voltage);}
void can_tx_hbridge_temp(void){can_transmit(&tx_fd_hbridge_temp);}
void can_tx_left_heel_sense(void){can_transmit(&tx_fd_left_heel_sense);}
void can_tx_right_heel_sense(void){can_transmit(&tx_fd_right_heel_sense);}
void can_tx_exec_time(void){can_transmit(&tx_fd_exec_time);} 
void can_tx_max_exec_time(void){can_transmit(&tx_fd_max_exec_time);} 

