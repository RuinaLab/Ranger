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

/******** TIMESTAMP ******************************************************/
void dn_rx_timestamp(int timestamp)
{
  asched_set_timestamp(timestamp);
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
void dn_error_print(void)
{
  str_length = sprintf(string,"ERROR: %i\r\n",error_get_info()&0x1FFF);
  uarti_tx_buf(string, str_length);
}

/******** I2C COLOR SENSOR ************************************************/
float dn_get_color_white(void)
{
  return i2c_get_white_data();
}

float dn_get_color_red(void)
{
  return i2c_get_red_data();
}

float dn_get_color_green(void)
{
  return i2c_get_green_data();
}

float dn_get_color_blue(void)
{
  return i2c_get_blue_data();
}

float dn_get_status(void){
  return 1.0;
}

/******** Execution Time ***********************************************/
float dn_ticks_to_ms = 0.0000000166667; //1/60e6
unsigned int dn_execution_ticks = 0;
unsigned int dn_max_execution_ticks = 0; //in ticks
void dn_sched_done(void)
{
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


