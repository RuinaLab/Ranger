/*
	
	data_nexus.h
	
	A board specific conversion module which makes function wrappers
	and contains conversions from data types of one module to those
	required by another module. Ex, take integer ADC values and convert them
	to amps in floating or fixed-point.
	
	Nicolas Williamson - September 2009

*/

#ifndef __H_DATA_NEXUS__
#define __H_DATA_NEXUS__

void dn_safety(void);
float dn_get_battery_voltage(void);
int dn_get_hbridge_raw_temp(void);
float dn_get_hbridge_temp(void);
float dn_get_battery_current(void);
fixed dn_get_raw_motor_current(void);
float dn_get_motor_current(void);
int dn_get_motor_position(void);
float dn_get_motor_velocity(void);
float dn_get_motor_vel_rads(void);
float dn_get_motor_pos_rads(void);
void dn_error_transmit(void);
void dn_set_motor_current(float);
float dn_get_shaft_pos_rads(void);
int dn_get_shaft_position(void);
void dn_blue_heartbeat(void);
void dn_green_heartbeat(void);
void dn_red_heartbeat(void);
void dn_error_print(void);
void dn_mb_ready(float f);
void dn_start(float f);
fixed dn_to_fixed(int offset, float coeff, int value);
float dn_get_battery_power(void);
float dn_get_right_hs(void);
float dn_get_right_ls(void);
float dn_get_left_ls(void);
float dn_get_left_hs(void);
int dn_get_raw_right_hs(void);
int dn_get_raw_left_hs(void);
int dn_get_p0(void);
int dn_get_p1(void);
void dn_sync_reset_timer(void);
void dn_aux_reset_timer(void);
void dn_status_heartbeat(void);
void dn_rx_timestamp(int timestamp);
float dn_get_ankle_vel_rads(void);
float dn_get_ankle_pos_rads(void);
float dn_get_status(void);
void dn_reset_qec(void);
float dn_get_left_heel_sense(void);
float dn_get_right_heel_sense(void);
void dn_sched_done(void);
float dn_get_execution_time(void);
float dn_get_max_execution_time(void);
void dn_inner_feet_motor_encoder_rezero(void);
#endif
