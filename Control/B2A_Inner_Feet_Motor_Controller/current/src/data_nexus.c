/*

	data_nexus.c
	
	A board specific conversion module which makes function wrappers
	and contains conversions from data types of one module to those
	required by another module. Ex, take integer ADC values and convert them
	to amps in floating or fixed-point.
	
	Nicolas Williamson - September 2009

*/

#include <includes.h>

/******** SAFETY CHECK **********************************************/
void dn_safety(void){
  //Flags
  int right_ls = dn_get_right_ls() > 0.0; //hardware limit switch
  int left_ls = dn_get_left_ls() > 0.0;
  float ankle_angle = dn_get_ankle_pos_rads();
  int cable_stretched = (ankle_angle - dn_get_motor_pos_rads()) > 0.3; //cable stretched by 0.3 rads
  int neg_angle = ankle_angle < 0.2; //0.2 radians
  int pos_angle = ankle_angle > 2.9; //3.2 = physical limit, 0.2 radians off
  
  //Error reporting
  if (right_ls || left_ls){error_occurred(ERROR_MCFI_ANKL_LS);}
  if (neg_angle){error_occurred(ERROR_MCFI_ANKL_LO);}
  if (pos_angle){error_occurred(ERROR_MCFI_ANKL_HI);}
  if (cable_stretched){error_occurred(ERROR_MCFI_CBL_STRCH);}
  
  //Negative direction limiting
  if (right_ls || left_ls || cable_stretched || neg_angle){
    mc_direction_control(MC_NEGATIVE, MC_STOP);
    mcu_led_red_blink(100);
  } else {
    mc_direction_control(MC_NEGATIVE, MC_START);
  }

  //Positive direction limiting
  if (pos_angle){ 
    mc_direction_control(MC_POSITIVE, MC_STOP);
    mcu_led_red_blink(100);
  } else {
    mc_direction_control(MC_POSITIVE, MC_START);
  }
  
}

/******** HEARTBEAT ******************************************************/
void dn_blue_heartbeat(void){
  mcu_led_blue_blink(50);
}
void dn_green_heartbeat(void){
  mcu_led_green_blink(50);
}
void dn_red_heartbeat(void){
  mcu_led_red_blink(50);
}

/******** ADCI <-> VOLTAGES ***********************************************/

//Battery voltage measurements
#define DN_BV_OFFSET (0)
#define DN_BV_GAIN (0.0324f)
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

//H-bridge (MC board) temperature measurements
int dn_get_hbridge_raw_temp(void){
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
#define DN_MC_OFFSET (-1)
#define DN_MC_GAIN (0.001525f)
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

#define DN_BC_OFFSET (-3)
#define DN_BC_GAIN (0.000502f)
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

/********* HEEL STRIKES ************************************************/
int dn_get_raw_right_hs(void){
  static unsigned short prev_value = 0;
  unsigned short lower_switch_point = 1500;
  unsigned short upper_switch_point = 2000;
  int adc = adcx_get_result(ADCX_CH_1);

  if (adc > upper_switch_point) // Above the upper switch point, output heelstrike = 1
  {
    prev_value = 1;
    return 1;
  }
  else if (adc < lower_switch_point) // Below the lower switch point, output heelstrike = 0
  {
    prev_value = 0;
    return 0;
  }
  else   // It's somewhere in the middle between the two values, don't change the output
  {
    return prev_value;
  }
}
int dn_get_raw_left_hs(void){
  static unsigned short prev_value = 0;
  unsigned short lower_switch_point = 1000;
  unsigned short upper_switch_point = 2000;
  int adc = adcx_get_result(ADCX_CH_0);

  if (adc > upper_switch_point) // Above the upper switch point, output heelstrike = 1
  {
    prev_value = 1;
    return 1;
  }
  else if (adc < lower_switch_point) // Below the lower switch point, output heelstrike = 0
  {
    prev_value = 0;
    return 0;
  }
  else   // It's somewhere in the middle between the two values, don't change the output
  {
    return prev_value;
  }
}
float dn_get_right_hs(void){
  return ((float)ls_get_state(2));
}
float dn_get_left_hs(void){
  return ((float)ls_get_state(3));
}

/////////////////////////////////////////////////////////////////////
float dn_get_left_heel_sense(void)
{
  return ((float)(adcx_get_result(ADCX_CH_0)));
}

float dn_get_right_heel_sense(void)
{
  //return ((float)(adcx_get_result(ADCX_CH_1) + adcx_get_result(ADCX_CH_0)));
  return ((float)(adcx_get_result(ADCX_CH_1)));
}

//////////////////////////////////////////////////////////////////////


/***********LIMIT SWITCHES *********************************************/
int dn_get_p0(void){
  return (!(FIO0PIN & (1<<0))); //normally high, so if 1 == high == off
}
int dn_get_p1(void){
  return (!(FIO0PIN & (1<<1))); //normally high, so if 1 == high == off
}
float dn_get_right_ls(void){
  if (ls_get_state(1) == LS_OFF) return 0.0f;
  else return 1.0f;
}
float dn_get_left_ls(void){
  if (ls_get_state(0) == LS_OFF) return 0.0f;
  else return 1.0f;
}

/******** CAN RECIEVES AND TRANSMITS ***********************************/
extern CAN_FRAME_DESC tx_fd_error;
char string[40];
int str_length;
// ********************************
// Error Transmit
// ********************************
void dn_error_transmit(void){
  can_transmit(&tx_fd_error);
}

// ********************************
// QDC wrapper functions
// ********************************
float dn_get_motor_pos_rads(void)
{
  return qdc_tmr0_cap01_get_angle_float();
}

float dn_get_motor_vel_rads(void)
{
  return qdc_tmr0_cap01_get_rate_float();
}

float dn_get_ankle_vel_rads(void)
{
  return qdc_tmr0_cap23_get_rate_float();
}

// ********************************
// Inner feet motor encoder rezero
// ********************************
//Over time, this function resets the motor incremental encoder value to the absolute encoder value
void dn_inner_feet_motor_encoder_rezero(void)
{
  static float correction = 0;
  float motor_position;

  //Only rezero when feet are flipped up and mostly stationary
  if ((dn_get_ankle_pos_rads() < 0.3) && (dn_get_ankle_vel_rads() < 0.01) && (dn_get_ankle_vel_rads() > -0.01))
  {
    motor_position = dn_get_motor_pos_rads();
    correction = (dn_get_ankle_pos_rads() - motor_position);
    qdc_tmr0_cap01_set_angle_float(motor_position + (correction * 0.05));
  }
}

// ********************************
// Absolute Encoder
// ********************************
#define AE_TO_RADS (0.0007669922f) //(6.2832/(float)8192) //so it is precalculated 
AE_ID dn_ae = AE_2;
int dn_get_ankle_position(void)
{
	return ae_get_pos(dn_ae);
}

float dn_get_ankle_pos_rads(void)
{
  float pos = ae_get_pos(dn_ae);
  pos = (pos) * AE_TO_RADS;
  return pos;
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
