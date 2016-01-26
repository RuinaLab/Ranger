/*
	can_setup.h
	Nicolas Williamson - March 2010
*/

void init_can(void);

//Wrapper functions for calling CAN transmit from the scheduler
void can_tx_motor_pos(void);
void can_tx_motor_vel(void);
void can_tx_motor_current(void);
void can_tx_angle(void);
void can_tx_rate(void);
void can_tx_batt_power(void);
void can_tx_ls_right(void);
void can_tx_ls_left(void);
void can_tx_hs_right(void);
void can_tx_hs_left(void);
void can_tx_board_status(void);
void can_tx_batt_current(void);
void can_tx_batt_voltage(void);
void can_tx_hbridge_temp(void);
void can_tx_left_heel_sense(void);
void can_tx_right_heel_sense(void);
void can_tx_exec_time(void);
void can_tx_max_exec_time(void);

