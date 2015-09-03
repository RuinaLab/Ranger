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
float dn_get_battery_current(void);
MCF32 dn_get_raw_motor_current(void);
float dn_get_motor_current(void);
int dn_get_motor_position(void);
float dn_get_motor_velocity(void);
float dn_get_motor_vel_rads(void);
float dn_get_motor_pos_rads(void);
void dn_error_transmit(void);
void dn_set_motor_current(float);
float dn_get_shaft_pos_rads(void);
int dn_get_shaft_position(void);

#endif
