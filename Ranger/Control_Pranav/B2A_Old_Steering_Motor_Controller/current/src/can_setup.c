/*
	can_setup.c
	
	All of the CAN receives and transmits for the board.
	
	Nicolas Williamson - March 2010
*/

#include <includes.h>

// *********************  CAN Initialization  ************************** //

// ***** RX Frames
#define RX_LIST_SIZE 15
CAN_FRAME_DESC * rxlist[RX_LIST_SIZE];

// ***** RTR Frames
#define RTR_LIST_SIZE 10
CAN_FRAME_DESC * rtrlist[RTR_LIST_SIZE];

// ***** Transmit
CAN_FRAME_DESC tx_fd_error;
CAN_FRAME_DESC tx_fd_angle;
CAN_FRAME_DESC tx_fd_color_white;
CAN_FRAME_DESC tx_fd_color_red;
CAN_FRAME_DESC tx_fd_color_green;
CAN_FRAME_DESC tx_fd_color_blue;
CAN_FRAME_DESC tx_fd_left_ankle_angle;
CAN_FRAME_DESC tx_fd_left_ankle_rate;
CAN_FRAME_DESC tx_fd_status;
CAN_FRAME_DESC tx_fd_exec_time;
CAN_FRAME_DESC tx_fd_max_exec_time;

// ***** Receive
CAN_FRAME_DESC rx_fd_timestamp;

void init_can(void){

  // ************* First, fill regular RX descriptors ***************** //  
  int i = 0;

  //Timestamp 
  rxlist[i++] = &rx_fd_timestamp;  
  can_set_rx_descriptor_fi(&rx_fd_timestamp, ID_TIMESTAMP, CHAN_CAN1, can_rx_setter_float_dummy, dn_rx_timestamp);

  //Add NULL terminator
  rxlist[i++] = (CAN_FRAME_DESC *) 0;  
  if(i >= RX_LIST_SIZE) {
    error_occurred(ERROR_CAN_RXLST_OF);
  }

  // ************* NOW, fill RTR descriptors ************************** //
  i = 0;

   //Add NULL terminator
  rtrlist[i++] = (CAN_FRAME_DESC *) 0;
  if(i >= RTR_LIST_SIZE) {
    error_occurred(ERROR_CAN_RTRLST_OF);
  }

  // ************** NOW, fill standard non-RTR TX descriptors (not in a list) *** //
  can_set_tx_descriptor_ii(&tx_fd_error, ID_ERROR_MCSO, CHAN_CAN1, error_get_info, error_get_time);
  can_set_tx_descriptor_fi(&tx_fd_color_white, ID_MCSO_COLOR_BACK_WHITE, CHAN_CAN1, dn_get_color_white, asched_get_timestamp);
  can_set_tx_descriptor_fi(&tx_fd_color_red, ID_MCSO_COLOR_BACK_RED, CHAN_CAN1, dn_get_color_red, asched_get_timestamp);
  can_set_tx_descriptor_fi(&tx_fd_color_green, ID_MCSO_COLOR_BACK_GREEN, CHAN_CAN1, dn_get_color_green, asched_get_timestamp);
  can_set_tx_descriptor_fi(&tx_fd_color_blue, ID_MCSO_COLOR_BACK_BLUE, CHAN_CAN1, dn_get_color_blue, asched_get_timestamp);
  can_set_tx_descriptor_fi(&tx_fd_left_ankle_angle, ID_MCSO_LEFT_ANKLE_ANGLE, CHAN_CAN1, dn_get_ankle_pos_rads, asched_get_timestamp);
  can_set_tx_descriptor_fi(&tx_fd_left_ankle_rate, ID_MCSO_LEFT_ANKLE_RATE, CHAN_CAN1, dn_get_ankle_vel_rads, asched_get_timestamp);   
  can_set_tx_descriptor_fi(&tx_fd_status, ID_MCSO_STATUS, CHAN_CAN1, dn_get_status, asched_get_timestamp); 
  can_set_tx_descriptor_fi(&tx_fd_exec_time, ID_MCSO_EXECUTION_TIME, CHAN_CAN1, dn_get_execution_time, asched_get_timestamp);
  can_set_tx_descriptor_fi(&tx_fd_max_exec_time, ID_MCSO_MAX_EXECUTION_TIME, CHAN_CAN1, dn_get_max_execution_time, asched_get_timestamp);
  
  // ******** Set CAN rx descriptors ***********
  can_rx_set_descriptors(rxlist, rtrlist);

  // ******** Initialize low-level CAN RX struct for assembly function *********
  can_init();
}


//Wrapper functions for calling CAN transmit from the scheduler
void can_tx_left_ankle_rate(void){can_transmit(&tx_fd_left_ankle_rate);}
void can_tx_left_ankle_angle(void){can_transmit(&tx_fd_left_ankle_angle);}
void can_tx_color_white(void){can_transmit(&tx_fd_color_white);}
void can_tx_color_red(void){can_transmit(&tx_fd_color_red);}
void can_tx_color_green(void){can_transmit(&tx_fd_color_green);}
void can_tx_color_blue(void){can_transmit(&tx_fd_color_blue);}
void can_tx_exec_time(void){can_transmit(&tx_fd_exec_time);}
void can_tx_max_exec_time(void){can_transmit(&tx_fd_max_exec_time);}
void can_tx_status(void){can_transmit(&tx_fd_status);}
