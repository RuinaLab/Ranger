/*
	can_setup.h
	Nicolas Williamson - March 2010
*/

void init_can(void);

//Wrapper functions for calling CAN transmit from the scheduler
void can_tx_status(void);
void can_tx_left_ankle_rate(void);
void can_tx_color_white(void);
void can_tx_color_red(void);
void can_tx_color_green(void);
void can_tx_color_blue(void);
void can_tx_batt_power(void);
void can_tx_left_ankle_angle(void);
void can_tx_exec_time(void);
void can_tx_max_exec_time(void);


