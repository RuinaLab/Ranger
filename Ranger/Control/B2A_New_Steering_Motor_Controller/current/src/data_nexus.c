/*

	data_nexus.c
	
	A board specific conversion module which makes function wrappers
	and contains conversions from data types of one module to those
	required by another module. Ex, take integer ADC values and convert them
	to amps in floating or fixed-point. Essentially every module which relies on
  another module must do so through the data nexus; one day these functions will 
  hopefully be automatically generated, as defining them all is a pain in the butt!
	
	Nicolas Williamson - September 2009

*/

#include <includes.h>

/******** STEERING ANGLE SAFETY CHECK *******************************/
void dn_safety(void){
  int max_dev = 200; 
  int angle_zero = 4803;
  int steer_angle = dn_get_steer_angle_raw();
  if ((steer_angle - angle_zero) > max_dev){ //turned too far left
    mc_direction_control(MC_POSITIVE, MC_STOP);
    error_occurred(ERROR_MCSI_LEFT);
    mcu_led_red_blink(100);
  } else if ((steer_angle - angle_zero) < -max_dev){ //turned too far right
    mc_direction_control(MC_NEGATIVE, MC_STOP);
    error_occurred(ERROR_MCSI_RIGHT);
    mcu_led_red_blink(100);
  } else {
    mc_direction_control(MC_NEGATIVE, MC_START);
    mc_direction_control(MC_POSITIVE, MC_START);
  }
}

/******** HEARTBEAT ******************************************************/
void dn_blue_heartbeat(void){
  mcu_led_blue_blink(50);
}
void dn_green_heartbeat(void){
  mcu_led_green_toggle();
}
void dn_red_heartbeat(void){
  mcu_led_red_toggle();
}

/******** ADCI <-> VOLTAGES ***********************************************/
int bv_offset = 0;
float bv_gain = 0.0327f;
float dn_get_battery_voltage(void){
  int adc_value = adci_get_result(ADCI_CH_1);
  float batt_voltage = (adc_value - bv_offset) * bv_gain;
  return batt_voltage;
}
fixed dn_get_raw_battery_voltage(void){
  int value = adci_get_result(ADCI_CH_1);
	return linear_to_fixed(bv_offset, bv_gain, value);
}
int dn_get_hbridge_raw_temp(void){
  return adci_get_result(ADCI_CH_3);
}

/********* MOTOR CONTROLLER <-> ADCX ***********************************/

//Called by CAN for incoming motor command value
void dn_update_pwm(void)
{
  mc_set_pwm((int)mc_get_command_current());
}

int mcurr_offset = 25;
float mcurr_coeff = 0.0001535f;
fixed dn_get_raw_motor_current(void){ //External ADC to Amps in fixed-point
  //returns a fixed-point representation of the motor current
  // Set adcx_add_config( ADCX_CH_P4N5, ADCX_GAIN_1); in software_setup
	int value = adcx_get_result(ADCX_CH_P4N5); //get adc data
  return linear_to_fixed(mcurr_offset, mcurr_coeff, value);
}
float dn_get_motor_current(void){
// Set adcx_add_config( ADCX_CH_P4N5, ADCX_GAIN_1); in software_setup
	int a = adcx_get_result(ADCX_CH_P4N5); //get adc data
  float b = -mcurr_coeff* (a - mcurr_offset);
  return b;
}
int bc_offset = -75;
float bc_coeff = 0.0000505f;
float dn_get_battery_current(void){
  // Set adcx_add_config( ADCX_CH_P2N3, ADCX_GAIN_1); in software_setup
	int a = adcx_get_result(ADCX_CH_P2N3); //get adc data
  float b = bc_coeff* (a - bc_offset); //subtract the adc offset
  return b;
}
fixed dn_get_raw_battery_current(void){
//returns a fixed-point representation of the battery current
  // Set adcx_add_config( ADCX_CH_P2N3, ADCX_GAIN_1); in software_setup
	int value = adcx_get_result(ADCX_CH_P2N3); //get adc data
	return linear_to_fixed(bc_offset, bc_coeff, value);
}

/********* BATTERY POWER (I*V) ***********************************/
float dn_get_batt_power(void){
  fixed batt_curr = dn_get_raw_battery_current();
  fixed batt_volt = dn_get_raw_battery_voltage();
  fixed batt_power = fixed_mult(batt_curr, batt_volt);
  return fixed_to_float(batt_power);
}

/******** CAN RECEIVES AND TRANSMITS ***********************************/
extern CAN_FRAME_DESC tx_fd_error;
char string[40];
int str_length;
// ********************************
// Error Transmit
// ********************************
void dn_error_transmit(void){
  can_transmit(&tx_fd_error);
}
void dn_error_print(void){
  str_length = sprintf(string,"ERROR: %i\r\n",error_get_info()&0xFF);
  uarti_tx_buf(string, str_length);
}
#define ADC_TO_ANGLE (1.0)
int dn_steer_zero_offset = 0;
int dn_get_steer_angle_raw(void){
  return adcx_get_result(ADCX_CH_0);
}
float dn_get_steer_angle(void){
  int ang = dn_get_steer_angle_raw() - dn_steer_zero_offset;
  return ((float)ang)*(ADC_TO_ANGLE);
}

// ********************************
// Timestamp
// ********************************
void dn_rx_timestamp(int timestamp)
{
  asched_set_timestamp(timestamp);
}

float dn_get_status(void){
  return 1.0;
}

/******** Execution Time ***********************************************/
float dn_ticks_to_ms = 0.0000000166667; //1/60e6
unsigned int dn_execution_ticks = 0;
unsigned int dn_max_execution_ticks = 0; //in ticks
void dn_sched_done(void){
  if (T1TC > dn_max_execution_ticks)  //Overall maximum since startup
  {
    dn_max_execution_ticks = T1TC;
  }
  if (!(asched_get_timestamp() & 0xFF)) //Reset short-term maximum when lower bits of timestamp are all zero
  {
    dn_execution_ticks = T1TC;
  }
  if (T1TC > dn_execution_ticks)
  {
    dn_execution_ticks = T1TC;
  }
}

float dn_get_execution_time(void)
{  
  return (float)dn_execution_ticks * dn_ticks_to_ms;
}

float dn_get_max_execution_time(void)
{
  return (float)dn_max_execution_ticks * dn_ticks_to_ms;
}


