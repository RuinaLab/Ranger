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

float dn_get_battery_voltage(void);
int dn_get_hbridge_raw_temp(void);
int dn_get_motor_position(void);
float dn_get_motor_velocity(void);
float dn_get_motor_vel_rads(void);
float dn_get_motor_pos_rads(void);
void dn_error_transmit(void);
float dn_get_shaft_pos_rads(void);
int dn_get_shaft_position(void);
void dn_blue_heartbeat(void);
void dn_green_heartbeat(void);
void dn_red_heartbeat(void);
void dn_status_heartbeat(void);
void dn_error_print(void);
void dn_mb_ready(float f);
void dn_start(float f);
void dn_sync_reset_timer(void);
void dn_aux_reset_timer(void);
float dn_get_ankle_pos_rads(void);
float dn_get_ankle_vel_rads(void);
int dn_get_ankle_position(void);
float dn_get_steer_angle(void);
int dn_get_steer_angle_raw(void);
void dn_tick(void);
float dn_get_ankle_vel_rads(void);
void dn_waiting(void);
void dn_mb_status(float status);
void dn_rx_timestamp(int timestamp);
void dn_debugging(void);
float dn_get_status(void);
void dn_sched_done(void);
float dn_get_color_white(void);
float dn_get_color_red(void);
float dn_get_color_green(void);
float dn_get_color_blue(void);
float dn_get_execution_time(void);
float dn_get_max_execution_time(void);

#endif
