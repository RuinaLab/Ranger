/*

	data_nexus.c
	
	A board specific conversion module which makes function wrappers
	and contains conversions from data types of one module to those
	required by another module. Ex, take integer ADC values and convert them
	to amps in floating or fixed-point.
	
	Nicolas Williamson - September 2009

*/

#include <includes.h>

unsigned int lcd_update_period = 400; //lcd_period/main_brain_period = 50ms/2ms


void dn_safety(void){

}

void dn_blue_heartbeat(void){
  mcu_led_blue_blink(50);
}

/******** CAN RECIEVES AND TRANSMITS ***********************************/
extern CAN_FRAME_DESC tx_fd_error;
// ********************************
// Error Transmit
// ********************************
void dn_error_transmit(void){
  can_transmit(&tx_fd_error);
}

void dn_tick(void)__irq{
  T0IR = 0xFFFF;
  asched_tick();
  VICVectAddr = 0;
}

void dn_error_lcd(void){
  char string[30];
  int strlength;
  strlength = sprintf(string, "%i", error_get_info()&0xFF);
  lcd_print(string, strlength, 0);
}

// ********************************
// IMU Data Transmit
// ********************************
float dn_get_roll(void){
  return msimu_get_data_float(MSIMU_DATA_ROLL);
}

float dn_get_pitch(void){
  return msimu_get_data_float(MSIMU_DATA_PITCH);
}

float dn_get_yam(void){
  return msimu_get_data_float(MSIMU_DATA_YAM);
}

float dn_get_angRateX(void){
  return msimu_get_data_float(MSIMU_DATA_ANGLE_RATE_X);
}

float dn_get_angRateY(void){
  return msimu_get_data_float(MSIMU_DATA_ANGLE_RATE_Y);
}

float dn_get_angRateZ(void){
  return msimu_get_data_float(MSIMU_DATA_ANGLE_RATE_Z);
}

int dn_get_timer(void){
  return msimu_get_data_int(MSIMU_DATA_TIMER); 
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

// ********************************
// LCD Data
// ********************************
void dn_rx_lcd_quad_1(int data){
  //if((asched_get_timestamp() % lcd_update_period) > 2) return;
  //the int will contain 4 ASCII characters
	lcd_set_quad1((data>>0)&0xFF,(data>>8)&0xFF,(data>>16)&0xFF,(data>>24)&0xFF);
}

void dn_rx_lcd_quad_2(int data){
  //the int will contain 4 ASCII characters
  //if((asched_get_timestamp() % lcd_update_period) > 2) return;
	lcd_set_quad2((data>>0)&0xFF,(data>>8)&0xFF,(data>>16)&0xFF,(data>>24)&0xFF);
}

void dn_rx_lcd_quad_3(int data){
  //the int will contain 4 ASCII characters
  //if((asched_get_timestamp() % lcd_update_period) > 2) return;
	lcd_set_quad3((data>>0)&0xFF,(data>>8)&0xFF,(data>>16)&0xFF,(data>>24)&0xFF);
}

void dn_rx_lcd_quad_4(int data){
  //the int will contain 4 ASCII characters
  //if((asched_get_timestamp() % lcd_update_period) > 2) return;
	lcd_set_quad4((data>>0)&0xFF,(data>>8)&0xFF,(data>>16)&0xFF,(data>>24)&0xFF);
}

// ********************************
// LED Data
// ********************************
void dn_rx_led_1(int data){
	unsigned int r;
	unsigned int g;
	unsigned int b;
	r = (unsigned int)(data&0xFF);
	g = (unsigned int)((data>>8)&0xFF);
	b = (unsigned int)((data>>16)&0xFF);
	ui_led_rgb(0,r,g,b);
}

void dn_rx_led_2(int data){
	unsigned int r;
	unsigned int g;
	unsigned int b;
	r = (unsigned int)(data&0xFF);
	g = (unsigned int)((data>>8)&0xFF);
	b = (unsigned int)((data>>16)&0xFF);		
	ui_led_rgb(1,r,g,b);
}

void dn_rx_led_3(int data){
	unsigned int r;
	unsigned int g;
	unsigned int b;
	r = (unsigned int)(data&0xFF);
	g = (unsigned int)((data>>8)&0xFF);
	b = (unsigned int)((data>>16)&0xFF);	
	ui_led_rgb(2,r,g,b);	  
}

void dn_rx_led_4(int data){
	unsigned int r;
	unsigned int g;
	unsigned int b;
	r = (unsigned int)(data&0xFF);
	g = (unsigned int)((data>>8)&0xFF);
	b = (unsigned int)((data>>16)&0xFF);		
	ui_led_rgb(3,r,g,b);
}

void dn_rx_led_5(int data){
	unsigned int r;
	unsigned int g;
	unsigned int b;
	r = (unsigned int)(data&0xFF);
	g = (unsigned int)((data>>8)&0xFF);
	b = (unsigned int)((data>>16)&0xFF);	
	ui_led_rgb(4,r,g,b);	  
}

void dn_rx_led_6(int data){
	unsigned int r;
	unsigned int g;
	unsigned int b;
	r = (unsigned int)(data&0xFF);
	g = (unsigned int)((data>>8)&0xFF);
	b = (unsigned int)((data>>16)&0xFF);
	//mcu_led_green_blink(500);		
	ui_led_rgb(5,r,g,b);	  
}

// ********************************
// Button Data Transmit
// ********************************
int dn_get_buttons(void){
	int i, temp = 0;
	for(i = 0; i < 6; i++){
    if (button_pushed(i)){temp = temp | (1<<i);}
	}
	return temp;
}

// ********************************
// Buzzer CAN Receive
// ********************************
void dn_rx_buzzer(int freq){
  int f = (int)freq;
  buzzer_set_frequency(f);
}
void dn_rx_buzzer_amp(int amp){
  int a = (int)amp;
  //Plays the alma mater at least once
  if (a){
    song_play();
  } 
}

// ********************************
// Song CAN Receive
// ********************************
void dn_rx_play_song(int song){
  mcu_led_green_blink(100);
  if (song){
    mcu_led_red_blink(100);
    song_play();
  }
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

float dn_get_execution_time(void){  
  return (float)dn_execution_ticks * dn_ticks_to_ms;
}

float dn_get_max_execution_time(void){
  return (float)dn_max_execution_ticks * dn_ticks_to_ms;
}

