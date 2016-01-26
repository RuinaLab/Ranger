/*
	can_setup.h
	Nicolas Williamson - March 2010
*/

void init_can(void);

/********************* TASKS ***********************/
//transmit imu status
void can_tx_roll(void);
void can_tx_pitch(void);
void can_tx_yaw(void);
void can_tx_ang_rate_x(void);
void can_tx_ang_rate_y(void);
void can_tx_ang_rate_z(void);
//transmit button status
void can_tx_buttons(void);
//transmit rc info
void can_tx_rc0(void);
void can_tx_rc1(void);
void can_tx_rc2(void);
void can_tx_rc3(void);
                           
                           
//execution time
void can_tx_exec_time(void); 
void can_tx_max_exec_time(void); 
//Status
void can_tx_status(void);
