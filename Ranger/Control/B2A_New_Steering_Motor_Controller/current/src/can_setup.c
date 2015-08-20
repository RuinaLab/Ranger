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
CAN_FRAME_DESC tx_fd_motor_current;
CAN_FRAME_DESC tx_fd_angle;
CAN_FRAME_DESC tx_fd_battery_power;
CAN_FRAME_DESC tx_fd_status;
CAN_FRAME_DESC tx_fd_exec_time;
CAN_FRAME_DESC tx_fd_max_exec_time;

/* clearing up stuff; petr 5/30/13
// adding more transmission can_id's // noopetr, Feb/12/2013
CAN_FRAME_DESC tx_fd_test1;
CAN_FRAME_DESC tx_fd_test2;
*/

// ***** Receive
CAN_FRAME_DESC rx_fd_timestamp;
CAN_FRAME_DESC rx_fd_motor_current;
CAN_FRAME_DESC rx_fd_mc_shutdown;
CAN_FRAME_DESC rx_fd_mc_sleep;
CAN_FRAME_DESC rx_fd_mc_stiffness;
CAN_FRAME_DESC rx_fd_mc_command_current;
CAN_FRAME_DESC rx_fd_mc_dampness;

/*  clearing up stuff // petr 5/9/13
// adding more received can_id's // noopetr, Feb/12/2013
CAN_FRAME_DESC rx_fd_steer_filter;
*/


void init_can(void)
{
  // ************* First, fill regular RX descriptors ***************** //  
  int i = 0;

  //Timestamp 
  rxlist[i++] = &rx_fd_timestamp;  
  can_set_rx_descriptor_fi(&rx_fd_timestamp, ID_TIMESTAMP,CHAN_CAN2, can_rx_setter_float_dummy, dn_rx_timestamp);
  //Motor Control - command current
  rxlist[i++] = &rx_fd_mc_command_current;  
  can_set_rx_descriptor_fi(&rx_fd_mc_command_current, ID_MCSI_COMMAND_ANG,CHAN_CAN2, mc_set_command_current, can_rx_setter_int_dummy);
  //Motor Control - stiffness
  rxlist[i++] = &rx_fd_mc_stiffness;  
//  can_set_rx_descriptor_fi(&rx_fd_mc_stiffness, ID_MCSI_STIFFNESS,CHAN_CAN2, mc_set_stiffness, can_rx_setter_int_dummy); // commented on Feb/7/2013 to have more control on the gains of pid controller
  can_set_rx_descriptor_fi(&rx_fd_mc_stiffness, ID_MCSI_PROP_COEFF,CHAN_CAN2, mc_set_kp, can_rx_setter_int_dummy); // added on Feb/7/2013 to have more control on the gains of pid controller
  //Motor Control - dampness
  rxlist[i++] = &rx_fd_mc_dampness;  
//  can_set_rx_descriptor_fi(&rx_fd_mc_dampness, ID_MCSI_DAMPNESS,CHAN_CAN2, mc_set_dampness, can_rx_setter_int_dummy); // commented on Feb/7/2013 to have more control on the gains of pid controller
  can_set_rx_descriptor_fi(&rx_fd_mc_dampness, ID_MCSI_INT_COEFF,CHAN_CAN2, mc_set_ki, can_rx_setter_int_dummy); // added on Feb/7/2013 to have more control on the gains of pid controller
  //Motor Sleep
  rxlist[i++] = &rx_fd_mc_sleep;  
  can_set_rx_descriptor_ii(&rx_fd_mc_sleep, ID_MCSI_SLEEP,CHAN_CAN2, mc_set_sleep, can_rx_setter_int_dummy);
 //Motor Shutdown
  rxlist[i++] = &rx_fd_mc_shutdown;  
  can_set_rx_descriptor_ii(&rx_fd_mc_shutdown, ID_MCSI_SHUTDOWN,CHAN_CAN2, mc_set_shutdown, can_rx_setter_int_dummy);
   //Motor Control - target current
  rxlist[i++] = &rx_fd_motor_current;  
  can_set_rx_descriptor_fi(&rx_fd_motor_current, ID_MCSI_MOTOR_TARGET_CURRENT,CHAN_CAN2, mc_set_target_current, can_rx_setter_int_dummy);


/*  clearing up stuff // petr 5/9/13
  // adding more received can_id's // noopetr, Feb/12/2013
  rxlist[i++] = &rx_fd_motor_current;  
  can_set_rx_descriptor_fi(&rx_fd_steer_filter, ID_MCSI_EMPTY_TX1, CHAN_CAN2, dn_set_steer_filter, can_rx_setter_int_dummy);
  
  Feb/12/2013:
  ID_MCSI_EMPTY_TX1 receives the steer filter coefficient
*/
  

  //NULL Terminator
  rxlist[i++] = (CAN_FRAME_DESC *) 0;  
  if(i >= RX_LIST_SIZE) {
    error_occurred(ERROR_CAN_RXLST_OF);
  }

  // ************* NOW, fill RTR descriptors ************************** //
  i = 0;

  //NULL Terminator
  rtrlist[i++] = (CAN_FRAME_DESC *) 0;
  if(i >= RTR_LIST_SIZE) 
  {
    error_occurred(ERROR_CAN_RTRLST_OF);
  }

  // ************** NOW, fill standard non-RTR TX descriptors (not in a list) *** //
  can_set_tx_descriptor_fi(&tx_fd_angle, ID_MCSI_STEER_ANGLE, CHAN_CAN2, dn_get_steer_angle, asched_get_timestamp);
  can_set_tx_descriptor_fi(&tx_fd_motor_current, ID_MCSI_MOTOR_CURRENT, CHAN_CAN2, dn_get_motor_current, asched_get_timestamp);
  can_set_tx_descriptor_fi(&tx_fd_battery_power, ID_MCSI_BATT_POWER, CHAN_CAN2, dn_get_batt_power, asched_get_timestamp);
  can_set_tx_descriptor_ii(&tx_fd_error, ID_ERROR_MCSI, CHAN_CAN2, error_get_info, error_get_time); 
  can_set_tx_descriptor_fi(&tx_fd_status, ID_MCSI_STATUS, CHAN_CAN2, dn_get_status, asched_get_timestamp); 
  can_set_tx_descriptor_fi(&tx_fd_exec_time, ID_MCSI_EXECUTION_TIME, CHAN_CAN2, dn_get_execution_time, asched_get_timestamp);
  can_set_tx_descriptor_fi(&tx_fd_max_exec_time, ID_MCSI_MAX_EXECUTION_TIME, CHAN_CAN2, dn_get_max_execution_time, asched_get_timestamp);
  
  /* clearing up stuff; petr 5/30/13
  // adding more transmission can_id's // noopetr, Feb/12/2013
  can_set_tx_descriptor_fi(&tx_fd_test1, ID_MCSI_EMPTY_RX1, CHAN_CAN2, dn_get_mech_mult, asched_get_timestamp);
  can_set_tx_descriptor_fi(&tx_fd_test2, ID_MCSI_EMPTY_RX2, CHAN_CAN2, dn_get_therm_mult, asched_get_timestamp);
  */
  
  // ******** Set CAN rx descriptors ***********
  can_rx_set_descriptors(rxlist, rtrlist);

  // ******** Initialize low-level CAN RX struct for assembly function *********
  can_init();     
}

//Wrapper functions for calling CAN transmit from the scheduler
void can_transmit1(void){can_transmit(&tx_fd_angle);}
void can_transmit2(void){can_transmit(&tx_fd_motor_current);}
void can_transmit3(void){can_transmit(&tx_fd_battery_power);}
void can_transmit4(void){can_transmit(&tx_fd_exec_time);}
void can_transmit5(void){can_transmit(&tx_fd_max_exec_time);}
void can_transmit_status(void){can_transmit(&tx_fd_status);}

/* clearing up stuff; petr 5/30/13
// adding more transmission can_id's // noopetr, Feb/12/2013
void can_transmit_test(void)
{
  can_transmit(&tx_fd_test1);
  can_transmit(&tx_fd_test2);
}
*/
