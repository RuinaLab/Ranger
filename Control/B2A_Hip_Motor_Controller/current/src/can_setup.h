/*
	can_setup.h
	Nicolas Williamson - March 2010
*/

void init_can(void);

void can_transmit_status(void);

//Wrapper functions for calling CAN transmit from the scheduler
void can_tx_motor_position(void);
void can_tx_motor_velocity(void);
void can_tx_angle(void);
void can_tx_motor_current(void);
void can_tx_battery_power(void);
void can_tx_angle_rate(void);
void can_tx_board_status(void);
void can_tx_battery_current(void);
void can_tx_battery_voltage(void);
void can_tx_exec_time(void);
void can_tx_max_exec_time(void);

