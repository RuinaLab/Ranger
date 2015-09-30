/*
	can_setup.c
	
	All of the CAN receives and transmits for the board.
	
	Nicolas Williamson - March 2010
*/

#include <includes.h>

// *******************************************
// CAN Variables
// *******************************************

// ***** RX Frames
#define RX_LIST_SIZE 15
CAN_FRAME_DESC * rxlist[RX_LIST_SIZE];
#define RTR_LIST_SIZE 10
CAN_FRAME_DESC * rtrlist[RTR_LIST_SIZE];

//void fill_descriptors(void);

// ***** Transmit
CAN_FRAME_DESC tx_fd_error;
CAN_FRAME_DESC tx_fd_ready;
CAN_FRAME_DESC tx_fd_motor_position;
CAN_FRAME_DESC tx_fd_motor_velocity;
CAN_FRAME_DESC tx_fd_motor_current;
CAN_FRAME_DESC tx_fd_angle;
CAN_FRAME_DESC tx_fd_angle_rate;
CAN_FRAME_DESC tx_fd_battery_power;
CAN_FRAME_DESC tx_fd_status;
CAN_FRAME_DESC tx_fd_battery_current;
CAN_FRAME_DESC tx_fd_battery_voltage;
CAN_FRAME_DESC tx_fd_exec_time;
CAN_FRAME_DESC tx_fd_max_exec_time;
// ***** Receive
CAN_FRAME_DESC rx_fd_timestamp;
CAN_FRAME_DESC rx_fd_motor_current;
CAN_FRAME_DESC rx_fd_command_current;
CAN_FRAME_DESC rx_fd_stiffness;
CAN_FRAME_DESC rx_fd_dampness;
CAN_FRAME_DESC rx_fd_mc_shutdown;
CAN_FRAME_DESC rx_fd_mc_sleep;

void init_can(void){
  
  // ************* First, fill regular RX descriptors ***************** //  
  int i = 0;
  rxlist[i++] = &rx_fd_motor_current;  //Motor Control - target current
  can_set_rx_descriptor_fi(&rx_fd_motor_current, ID_MCH_MOTOR_TARGET_CURRENT, CHAN_CAN1, mc_set_target_current, can_rx_setter_int_dummy);
  rxlist[i++] = &rx_fd_timestamp;  //Timestamp
  can_set_rx_descriptor_fi(&rx_fd_timestamp, ID_TIMESTAMP, CHAN_CAN1,can_rx_setter_float_dummy, dn_rx_timestamp);
  rxlist[i++] = &rx_fd_mc_shutdown;  //Motor Shutdown
  can_set_rx_descriptor_ii(&rx_fd_mc_shutdown, ID_MCH_SHUTDOWN,CHAN_CAN1, mc_set_shutdown, can_rx_setter_int_dummy);
  rxlist[i++] = &rx_fd_command_current;  //Command Current for compliant control
  can_set_rx_descriptor_fi(&rx_fd_command_current, ID_MCH_COMMAND_CURRENT,CHAN_CAN1, mc_set_command_current, can_rx_setter_int_dummy);
  rxlist[i++] = &rx_fd_stiffness;  //Stiffness
  can_set_rx_descriptor_fi(&rx_fd_stiffness, ID_MCH_STIFFNESS,CHAN_CAN1, mc_set_stiffness, can_rx_setter_int_dummy);
  rxlist[i++] = &rx_fd_dampness;  //Dampness
  can_set_rx_descriptor_fi(&rx_fd_dampness, ID_MCH_DAMPNESS,CHAN_CAN1, mc_set_dampness, can_rx_setter_int_dummy);
  rxlist[i++] = &rx_fd_mc_sleep;  //Motor Sleep
  can_set_rx_descriptor_ii(&rx_fd_mc_sleep, ID_MCH_SLEEP,CHAN_CAN1, mc_set_sleep, can_rx_setter_int_dummy);
  rxlist[i++] = (CAN_FRAME_DESC *) 0;  //NULL Terminator
  if(i >= RX_LIST_SIZE) {
    error_occurred(ERROR_CAN_RXLST_OF);
  }

  // ************* NOW, fill RTR descriptors ************************** //
  i = 0;
  rtrlist[i++] = (CAN_FRAME_DESC *) 0;
  if(i >= RTR_LIST_SIZE) 
  {
    error_occurred(ERROR_CAN_RTRLST_OF);
  }

  // ************** NOW, fill standard non-RTR TX descriptors (not in a list) *** //
  can_set_tx_descriptor_fi(&tx_fd_motor_position, ID_MCH_MOTOR_POSITION, CHAN_CAN1, dn_get_motor_pos_rads, asched_get_timestamp);
  can_set_tx_descriptor_fi(&tx_fd_motor_velocity, ID_MCH_MOTOR_VELOCITY, CHAN_CAN1, dn_get_motor_vel_rads, asched_get_timestamp);
  can_set_tx_descriptor_fi(&tx_fd_angle, ID_MCH_ANGLE, CHAN_CAN1, dn_get_shaft_pos_rads, asched_get_timestamp);
  can_set_tx_descriptor_fi(&tx_fd_motor_current, ID_MCH_MOTOR_CURRENT, CHAN_CAN1, dn_get_motor_current, asched_get_timestamp);
  can_set_tx_descriptor_fi(&tx_fd_battery_power, ID_MCH_BATT_POWER, CHAN_CAN1, dn_get_battery_power, asched_get_timestamp);
  can_set_tx_descriptor_ii(&tx_fd_error, ID_ERROR_MCH, CHAN_CAN1, error_get_info, error_get_time); 
  can_set_tx_descriptor_fi(&tx_fd_status, ID_MCH_STATUS, CHAN_CAN1, dn_get_status, asched_get_timestamp); 
  can_set_tx_descriptor_fi(&tx_fd_angle_rate, ID_MCH_ANG_RATE, CHAN_CAN1, dn_get_shaft_vel_rads, asched_get_timestamp);
  can_set_tx_descriptor_fi(&tx_fd_battery_current, ID_MCH_BATT_CURRENT, CHAN_CAN1, dn_get_battery_current, asched_get_timestamp);
  can_set_tx_descriptor_fi(&tx_fd_battery_voltage, ID_MCH_BATT_VOLTAGE, CHAN_CAN1, dn_get_battery_voltage, asched_get_timestamp);
  can_set_tx_descriptor_fi(&tx_fd_exec_time, ID_MCH_EXECUTION_TIME, CHAN_CAN1, dn_get_execution_time, asched_get_timestamp);
  can_set_tx_descriptor_fi(&tx_fd_max_exec_time, ID_MCH_MAX_EXECUTION_TIME, CHAN_CAN1, dn_get_max_execution_time, asched_get_timestamp);
   
  
  // ******** Set CAN rx descriptors ***********
  can_rx_set_descriptors(rxlist, rtrlist);
  
  // ******** Initialize low-level CAN RX struct for assembly function *********
  can_init();  
}


/********************* TASKS ***********************/
void can_tx_motor_position(void){can_transmit(&tx_fd_motor_position);}
void can_tx_motor_velocity(void){can_transmit(&tx_fd_motor_velocity);}
void can_tx_angle(void){can_transmit(&tx_fd_angle);}
void can_tx_motor_current(void){can_transmit(&tx_fd_motor_current);}
void can_tx_battery_power(void){can_transmit(&tx_fd_battery_power);}
void can_tx_angle_rate(void){can_transmit(&tx_fd_angle_rate);}
void can_tx_board_status(void){can_transmit(&tx_fd_status);} 
void can_tx_battery_current(void){can_transmit(&tx_fd_battery_current);}
void can_tx_battery_voltage(void){can_transmit(&tx_fd_battery_voltage);} 
void can_tx_exec_time(void){can_transmit(&tx_fd_exec_time);} 
void can_tx_max_exec_time(void){can_transmit(&tx_fd_max_exec_time);} 

