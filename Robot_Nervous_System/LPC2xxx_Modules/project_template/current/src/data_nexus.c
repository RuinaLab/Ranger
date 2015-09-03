/*

	data_nexus.c
	
	A board specific conversion module which makes function wrappers
	and contains conversions from data types of one module to those
	required by another module. Ex, take integer ADC values and convert them
	to amps in floating or fixed-point.
	
	Nicolas Williamson - September 2009

*/

#include <includes.h>

#define QEC_TO_RADS (0.000584375f) //(6.2832/(float)10752) //so it is precalculated 
#define AE_TO_RADS (0.0007669922f) //(6.2832/(float)8192) //so it is precalculated 

/******** HIP ANGLE SAFETY CHECK *******************************/
extern float global_zero_angle;
void dn_safety(void){
  float max_dev = 0.2;
  float hip_angle = dn_get_shaft_pos_rads();
  if (hip_angle > (global_zero_angle + max_dev)){
    mc_direction_control(MC_POSITIVE, MC_STOP);
//    MCU_LED_GREEN_ON;
  } else if (hip_angle < (global_zero_angle - max_dev)){
    mc_direction_control(MC_NEGATIVE, MC_STOP);
//    MCU_LED_RED_ON;
  } else {
//    MCU_LED_GREEN_OFF;
//    MCU_LED_RED_OFF;
    mc_direction_control(MC_NEGATIVE, MC_START);
    mc_direction_control(MC_POSITIVE, MC_START);
  }
}

/******** STATE ******************************************************/
void dn_mb_ready(float f){
  st_wait();
}
void dn_start(float f){
  st_run();
}

/******** HEARTBEAT ******************************************************/
void dn_blue_heartbeat(void){
  mcu_led_blue_toggle();
}
void dn_green_heartbeat(void){
  mcu_led_green_toggle();
}
void dn_red_heartbeat(void){
  mcu_led_red_toggle();
}


/******** ADCI <-> VOLTAGES ***********************************************/
float dn_get_battery_voltage(void){
  float offset = 0.0f;
  float gain = 0.0327;
  
  int adc_value = adci_get_result(ADCI_CH_1);
  
  float batt_voltage = (adc_value - offset) * gain;
  return batt_voltage;
}

int dn_get_hbridge_raw_temp(void){
  return adci_get_result(ADCI_CH_3);
}

/********* MOTOR CONTROLLER <-> ADCX ***********************************/
MCF32 dn_get_raw_motor_current(void) //External ADC to Amps in fixed-point
//returns a fixed-point representation of the motor current
{
  int adc_offset = 6;
    
  int adc_gain = 1; //gain for adc->amp conversion
  float adc_coeff = 0.00605f;
	int a, b, c, d, e;
	
  // Set adcx_add_config( ADCX_CH_P4N5, ADCX_GAIN_1); in software_setup
	a = adcx_get_result(ADCX_CH_P4N5); //get adc data
  b = a - adc_offset; //subtract the adc offset
  c = -1 * MCF32_FROMINT(b); //convert to fixed point
	d = MCF32_FROMFLOAT(adc_coeff); //convert adc->amp coefficient to fixed point
	e = MCF32_MULT(c,d)/adc_gain; //multiply gain to get amps in fixed point
	
	return e;
}
float dn_get_motor_current(void){
//	int adc_gain = 1; //gain for adc->amp conversion
  int adc_offset = 6;
  float adc_coeff = 0.00605f;
	int a;
  float b;
  
  // Set adcx_add_config( ADCX_CH_P4N5, ADCX_GAIN_1); in software_setup
	a = adcx_get_result(ADCX_CH_P4N5); //get adc data
  b = -adc_coeff* (a - adc_offset);
	
  return b;
}
float dn_get_battery_current(void){

//	int adc_gain = 1; //gain for adc->amp conversion
  int adc_offset = 1;//offset for adc->amp conversion; find 0Amp value in adc
  float adc_coeff = 0.00593;//0.00675f;
	int a;
  float b;
	
  // Set adcx_add_config( ADCX_CH_P2N3, ADCX_GAIN_1); in software_setup
	a = adcx_get_result(ADCX_CH_P2N3); //get adc data
  b = adc_coeff* (a - adc_offset); //subtract the adc offset

  return b;

}

/******** CAN RECIEVES AND TRANSMITS ***********************************/
extern CAN_FRAME_DESC error_tx_fd;
char string[40];
int str_length;
// ********************************
// Error Transmit
// ********************************
void dn_error_transmit(void){
  can_transmit(&error_tx_fd);
}
void dn_error_print(void){
  str_length = sprintf(string,"ID: %i\r\n",error_get_id()&0xFF);
  uarti_tx_buf(string, str_length);
}
// ********************************
// QEC Values
// ********************************
int dn_get_motor_position(void){
	return qec_get_abs_pos(QEC_2);
}
float dn_get_motor_velocity(void){
	return qec_get_velocity(QEC_2);
}
float dn_get_motor_pos_rads(void){
  float pos = qec_get_abs_pos(QEC_2);
  pos = (pos) * QEC_TO_RADS;
  return pos;
}
float dn_get_motor_vel_rads(void){
  float vel = qec_get_velocity(QEC_2);
  vel = (vel) * QEC_TO_RADS;
  return vel;
}
// ********************************
// Absolute Encoder
// ********************************
int dn_get_shaft_position(void){
	return ae_get_pos(0);
}
float dn_get_shaft_pos_rads(void){
  float pos = ae_get_pos(0);
  pos = (pos) * AE_TO_RADS;
  return pos;
}
