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

/******** HIP ANGLE SAFETY CHECK *******************************/
void dn_safety(void){
  int max_dev = 800; //(rad/AE_TO_RADS) deviation from zero in radians ::: About 0.6 radians
  int hip_angle = dn_get_shaft_position();
  if (hip_angle > max_dev){
    mcu_led_red_blink(100);
    mc_direction_control(MC_POSITIVE, MC_STOP);
  } else if (hip_angle < -max_dev){
    mcu_led_green_blink(100);
    mc_direction_control(MC_NEGATIVE, MC_STOP);
  } else {
    mc_direction_control(MC_NEGATIVE, MC_START);
    mc_direction_control(MC_POSITIVE, MC_START);
  }
}

/******** HIP MOTOR ENCODER REZERO *******************************/
//Over time, this function resets the motor incremental encoder value to the absolute encoder value
void dn_hip_motor_encoder_rezero(void)
{
  float motor_position, correction;

  motor_position = dn_get_motor_pos_rads();
  correction = (dn_get_shaft_pos_rads() - motor_position);
  qdc_tmr0_cap23_set_angle_float(motor_position + (correction * 0.05));
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

//Battery voltage measurements
#define DN_BV_OFFSET (0)
#define DN_BV_GAIN (0.0323f)
float dn_get_battery_voltage(void)
{
  int bv = adci_get_result(ADCI_CH_1);
  return (DN_BV_GAIN * (float)(bv - DN_BV_OFFSET));
}
fixed dn_get_raw_battery_voltage(void)
{
  int bv = adci_get_result(ADCI_CH_1);
	return linear_to_fixed(DN_BV_OFFSET, DN_BV_GAIN, bv);
}

//H-bridge (board) temperatures
int dn_get_hbridge_raw_temp(void)
{
  return adci_get_result(ADCI_CH_3);
}
#define DN_HT_OFFSET (0)
#define DN_HT_GAIN (1.0f)
float dn_get_hbridge_temp(void)
{
  int ht = adci_get_result(ADCI_CH_3);
  return (DN_HT_GAIN * (float)(ht - DN_HT_OFFSET));
}

/********* MOTOR CONTROLLER <-> ADCX ***********************************/

//Motor current measurements
#define DN_MC_OFFSET (-9)
#define DN_MC_GAIN (0.00154f)
fixed dn_get_raw_motor_current(void)
{ //External ADC to Amps in fixed-point
  //returns a fixed-point representation of the motor current
  // Set adcx_add_config( ADCX_CH_P4N5, ADCX_GAIN_2); in software_setup
	int mc = adcx_get_result(ADCX_CH_P4N5); //get adc data
//  return (linear_to_fixed(DN_MC_OFFSET, DN_MC_GAIN, mc));
  return (linear_to_fixed(DN_MC_OFFSET, DN_MC_GAIN, mc));
}
float dn_get_motor_current(void)
{
// Set adcx_add_config( ADCX_CH_P4N5, ADCX_GAIN_2); in software_setup
	int mc = adcx_get_result(ADCX_CH_P4N5); //get adc data
  return (-DN_MC_GAIN * (float)(mc - DN_MC_OFFSET));
}

//Battery current measurements
#define DN_BC_OFFSET (-4)
#define DN_BC_GAIN (0.000506f)
float dn_get_battery_current(void){
  // Set adcx_add_config( ADCX_CH_P2N3, ADCX_GAIN_2); in software_setup
	int bc = adcx_get_result(ADCX_CH_P2N3); //get adc data
  return (DN_BC_GAIN * (float)(bc - DN_BC_OFFSET)); //subtract the adc offset;
}
fixed dn_get_raw_battery_current(void){
//returns a fixed-point representation of the battery current
  // Set adcx_add_config( ADCX_CH_P2N3, ADCX_GAIN_2); in software_setup
	int bc = adcx_get_result(ADCX_CH_P2N3); //get adc data
	return linear_to_fixed(DN_BC_OFFSET, DN_BC_GAIN, bc);
}

/********* BATTERY POWER (I*V) ***********************************/
float dn_get_battery_power(void)
{
  int bc = adcx_get_result(ADCX_CH_P2N3);  //get adc data for battery current
  int bv = adci_get_result(ADCI_CH_1);     //get adc data for battery voltage

  //Calculate battery power, using only one (I think) floating-point multiply so as to reduce execution time.
  //Will the compiler pre-multiply variables declared const? What about #define values? (very likely)
  return ((float)((bv - DN_BV_OFFSET) * (bc - DN_BC_OFFSET)) * (DN_BV_GAIN * DN_BC_GAIN));
}

/******** CAN RECEIVES AND TRANSMITS ***********************************/
extern CAN_FRAME_DESC tx_fd_error;
char string[50];
int str_length;
// ********************************
// Error Transmit
// ********************************
void dn_error_transmit(void){
  can_transmit(&tx_fd_error);
}
void dn_error_print(void){
//  str_length = sprintf(string,"ERROR: %i\n\r",error_get_time());//&0xFF);
  uarti_tx_buf(string, str_length);
}
// ********************************
// QDC wrapper functions
// ********************************
float dn_get_motor_pos_rads(void)
{
  return qdc_tmr0_cap23_get_angle_float();
}

float dn_get_motor_vel_rads(void)
{
  return qdc_tmr0_cap23_get_rate_float();
}

float dn_get_shaft_vel_rads(void)
{
  return qdc_tmr0_cap01_get_rate_float();
}


// ********************************
// Absolute Encoder
// ********************************
#define AE_TO_RADS (0.0007669922f) //(6.2832/(float)8192) //so it is precalculated 
static const AE_ID dn_ae = AE_1;
int dn_get_shaft_position(void)
{
	return ae_get_pos(dn_ae);
}
float dn_get_shaft_pos_rads(void)
{
  float pos = ae_get_pos(dn_ae);
  pos = (pos) * AE_TO_RADS;
  return pos;
}
// ********************************
// Timestamp
// ********************************
void dn_rx_timestamp(int timestamp){
  static int count = 0;
  asched_set_timestamp(timestamp);
  if (count < 3){
   // count += can_send_next_rtr();
  }
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



