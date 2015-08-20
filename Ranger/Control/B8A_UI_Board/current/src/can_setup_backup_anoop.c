/*
	can_setup.c
	
	All of the CAN receives and transmits for the board.
	
	Nicolas Williamson - March 2010
*/

#include <includes.h>

// *******************************************
// CAN Variables
// *******************************************

#define RX_LIST_SIZE 16
CAN_FRAME_DESC * rxlist[RX_LIST_SIZE];

#define RTR_LIST_SIZE 15
CAN_FRAME_DESC * rtrlist[RTR_LIST_SIZE];

// ***** Transmit
CAN_FRAME_DESC tx_fd_error;
CAN_FRAME_DESC tx_fd_roll;
CAN_FRAME_DESC tx_fd_pitch;
CAN_FRAME_DESC tx_fd_yaw;
CAN_FRAME_DESC tx_fd_ang_rate_x;
CAN_FRAME_DESC tx_fd_ang_rate_y;
CAN_FRAME_DESC tx_fd_ang_rate_z;
CAN_FRAME_DESC tx_fd_buttons;
CAN_FRAME_DESC tx_fd_rc_0;
CAN_FRAME_DESC tx_fd_rc_1;
CAN_FRAME_DESC tx_fd_status;
CAN_FRAME_DESC tx_fd_exec_time;
CAN_FRAME_DESC tx_fd_max_exec_time;

// ***** Receive
CAN_FRAME_DESC rx_fd_timestamp;
CAN_FRAME_DESC rx_fd_lcd_quad_1;
CAN_FRAME_DESC rx_fd_lcd_quad_2;
CAN_FRAME_DESC rx_fd_lcd_quad_3;
CAN_FRAME_DESC rx_fd_lcd_quad_4;
CAN_FRAME_DESC rx_fd_led_1;
CAN_FRAME_DESC rx_fd_led_2;
CAN_FRAME_DESC rx_fd_led_3;
CAN_FRAME_DESC rx_fd_led_4;
CAN_FRAME_DESC rx_fd_led_5;
CAN_FRAME_DESC rx_fd_led_6;
CAN_FRAME_DESC rx_fd_buzz_freq;
CAN_FRAME_DESC rx_fd_buzz_amp;
CAN_FRAME_DESC rx_fd_song;


void init_can(void){

  // ************* First, fill regular RX descriptors ***************** //  
  int i = 0;
  //Timestamp
  rxlist[i++] = &rx_fd_timestamp;  
  can_set_rx_descriptor_fi(&rx_fd_timestamp, ID_TIMESTAMP, CHAN_CAN2, can_rx_setter_float_dummy, dn_rx_timestamp);
  //lcd_quad_1
  rxlist[i++] = &rx_fd_lcd_quad_1;  
  can_set_rx_descriptor_ii(&rx_fd_lcd_quad_1, ID_UI_SET_LCD_QUAD_1, CHAN_CAN2, dn_rx_lcd_quad_1, can_rx_setter_int_dummy);
  //lcd_quad_2
  rxlist[i++] = &rx_fd_lcd_quad_2;  
  can_set_rx_descriptor_ii(&rx_fd_lcd_quad_2, ID_UI_SET_LCD_QUAD_2, CHAN_CAN2, dn_rx_lcd_quad_2, can_rx_setter_int_dummy);
  //lcd_quad_3
  rxlist[i++] = &rx_fd_lcd_quad_3;  
  can_set_rx_descriptor_ii(&rx_fd_lcd_quad_3, ID_UI_SET_LCD_QUAD_3, CHAN_CAN2, dn_rx_lcd_quad_3, can_rx_setter_int_dummy);
  //lcd_quad_4
  rxlist[i++] = &rx_fd_lcd_quad_4;  
  can_set_rx_descriptor_ii(&rx_fd_lcd_quad_4, ID_UI_SET_LCD_QUAD_4, CHAN_CAN2, dn_rx_lcd_quad_4, can_rx_setter_int_dummy);
  //led_1
  rxlist[i++] = &rx_fd_led_1;  
  can_set_rx_descriptor_ii(&rx_fd_led_1, ID_UI_SET_LED_1, CHAN_CAN2, dn_rx_led_1, can_rx_setter_int_dummy);
  //led_2
  rxlist[i++] = &rx_fd_led_2;  
  can_set_rx_descriptor_ii(&rx_fd_led_2, ID_UI_SET_LED_2, CHAN_CAN2, dn_rx_led_2, can_rx_setter_int_dummy);
  //led_3
  rxlist[i++] = &rx_fd_led_3;  
  can_set_rx_descriptor_ii(&rx_fd_led_3, ID_UI_SET_LED_3, CHAN_CAN2, dn_rx_led_3, can_rx_setter_int_dummy);
  //led_4
  rxlist[i++] = &rx_fd_led_4;  
  can_set_rx_descriptor_ii(&rx_fd_led_4, ID_UI_SET_LED_4, CHAN_CAN2, dn_rx_led_4, can_rx_setter_int_dummy);
  //led_5
  rxlist[i++] = &rx_fd_led_5;  
  can_set_rx_descriptor_ii(&rx_fd_led_5, ID_UI_SET_LED_5, CHAN_CAN2, dn_rx_led_5, can_rx_setter_int_dummy);
  //led_6
  rxlist[i++] = &rx_fd_led_6;  
  can_set_rx_descriptor_ii(&rx_fd_led_6, ID_UI_SET_LED_6, CHAN_CAN2, dn_rx_led_6, can_rx_setter_int_dummy);
  //buzzer_freq
  rxlist[i++] = &rx_fd_buzz_freq;  
  can_set_rx_descriptor_ii(&rx_fd_buzz_freq, ID_UI_SET_BUZZER_FREQ, CHAN_CAN2, dn_rx_buzzer, can_rx_setter_int_dummy);
  //buzzer_amp
  rxlist[i++] = &rx_fd_buzz_amp;  
  can_set_rx_descriptor_ii(&rx_fd_buzz_amp, ID_UI_SET_BUZZER_AMPL, CHAN_CAN2, dn_rx_buzzer_amp, can_rx_setter_int_dummy);
  //song control
  rxlist[i++] = &rx_fd_song;  
  can_set_rx_descriptor_ii(&rx_fd_song, ID_UI_EMPTY_TX2, CHAN_CAN2, dn_rx_play_song, can_rx_setter_int_dummy);

  //NULL Terminator
  rxlist[i++] = (CAN_FRAME_DESC *) 0;  

  if(i >= RX_LIST_SIZE) {
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
  can_set_tx_descriptor_ii(&tx_fd_error, ID_ERROR_UI, CHAN_CAN2, error_get_info, error_get_time);  
  can_set_tx_descriptor_fi(&tx_fd_roll, ID_UI_ROLL, CHAN_CAN2, dn_get_roll, asched_get_timestamp);
  can_set_tx_descriptor_fi(&tx_fd_pitch, ID_UI_PITCH, CHAN_CAN2, dn_get_pitch, asched_get_timestamp);
  can_set_tx_descriptor_fi(&tx_fd_yaw, ID_UI_YAW, CHAN_CAN2, dn_get_yam, asched_get_timestamp);
  can_set_tx_descriptor_fi(&tx_fd_ang_rate_x, ID_UI_ANG_RATE_X, CHAN_CAN2, dn_get_angRateX, asched_get_timestamp);
  can_set_tx_descriptor_fi(&tx_fd_ang_rate_y, ID_UI_ANG_RATE_Y, CHAN_CAN2, dn_get_angRateY, asched_get_timestamp);
  can_set_tx_descriptor_fi(&tx_fd_ang_rate_z, ID_UI_ANG_RATE_Z, CHAN_CAN2, dn_get_angRateZ, asched_get_timestamp);
  can_set_tx_descriptor_ii(&tx_fd_buttons, ID_UI_BUTTONS, CHAN_CAN2, dn_get_buttons, asched_get_timestamp);
  can_set_tx_descriptor_fi(&tx_fd_rc_0, ID_UI_RC_0, CHAN_CAN2, rcx_get_chan_0, asched_get_timestamp);
  can_set_tx_descriptor_fi(&tx_fd_rc_1, ID_UI_RC_1, CHAN_CAN2, rcx_get_chan_1, asched_get_timestamp);
  
  
  can_set_tx_descriptor_fi(&tx_fd_status, ID_UI_STATUS, CHAN_CAN2, dn_get_status, asched_get_timestamp);
  can_set_tx_descriptor_fi(&tx_fd_exec_time, ID_UI_EXECUTION_TIME, CHAN_CAN2, dn_get_execution_time, asched_get_timestamp);
  can_set_tx_descriptor_fi(&tx_fd_max_exec_time, ID_UI_MAX_EXECUTION_TIME, CHAN_CAN2, dn_get_max_execution_time, asched_get_timestamp);
     
  // ******** Set CAN rx descriptors ***********
  can_rx_set_descriptors(rxlist, rtrlist);

  // ******** Initialize low-level CAN RX struct for assembly function *********
  can_init();
}


/********************* TASKS ***********************/
//transmit imu status
void can_tx_roll(void){can_transmit(&tx_fd_roll);}
void can_tx_pitch(void){can_transmit(&tx_fd_pitch);}
void can_tx_yaw(void){can_transmit(&tx_fd_yaw);}
void can_tx_ang_rate_x(void){can_transmit(&tx_fd_ang_rate_x);}
void can_tx_ang_rate_y(void){can_transmit(&tx_fd_ang_rate_y);}
void can_tx_ang_rate_z(void){can_transmit(&tx_fd_ang_rate_z);}
//transmit button status
void can_tx_buttons(void){can_transmit(&tx_fd_buttons);}
//transmit rc info
void can_tx_rc0(void){can_transmit(&tx_fd_rc_0);}
void can_tx_rc1(void){can_transmit(&tx_fd_rc_1);}
//execution time
void can_tx_exec_time(void){can_transmit(&tx_fd_exec_time);} 
void can_tx_max_exec_time(void){can_transmit(&tx_fd_max_exec_time);} 
//Status
void can_tx_status(void){can_transmit(&tx_fd_status);}

