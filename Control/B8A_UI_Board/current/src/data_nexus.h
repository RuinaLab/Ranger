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
void dn_error_transmit(void);
void dn_check_buttons(void);
float dn_get_roll(void);
void dn_blue_heartbeat(void);
void dn_timer1_overflow(void);
void dn_tick(void)__irq;
void dn_error_lcd(void);
//imu status
float dn_get_roll(void);
float dn_get_pitch(void);
float dn_get_yam(void);
float dn_get_angRateX(void);
float dn_get_angRateY(void);
float dn_get_angRateZ(void);
int dn_get_timer(void);
//button status
int dn_get_buttons(void);    
void dn_rx_lcd_quad_1(int data);  
void dn_rx_lcd_quad_2(int data);  
void dn_rx_lcd_quad_3(int data);  
void dn_rx_lcd_quad_4(int data);  
void dn_rx_led_1(int data); 
void dn_rx_led_2(int data); 
void dn_rx_led_3(int data); 
void dn_rx_led_4(int data); 
void dn_rx_led_5(int data); 
void dn_rx_led_6(int data);


void dn_sched_done(void);
float dn_get_execution_time(void);
float dn_get_max_execution_time(void);

void dn_rx_buzzer(int freq);
void dn_rx_buzzer_amp(int amp);

void dn_rx_timestamp(int timestamp);
float dn_get_status(void);

void dn_rx_play_song(int song);

#endif
