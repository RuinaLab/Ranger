/**

	@file mcu_led.c
	
	Controls for the microcontroller LEDs on satellite boards.
  Requires SCHED_SPEED to be accurately defined in khz.
	  
  Example Hardware and Register setup for the LEDs:
  @code
  // ***********************************************
  // Heartbeat Init Section
  // ***********************************************
  //On-board LED initialization
  PINSEL2 &= ~(1<<3);   // set trace port for GPIO use (bit 3 = 0)
  FIO1DIR |= (1<<23);   // set P1.23 to be output (Green LED)
  FIO1DIR |= (1<<24);   // set P1.24 to be output (Red LED)
  FIO1DIR |= (1<<25);   // set P1.25 to be output (Blue LED)
  MCU_LED_ALL_OFF; 
  @endcode
  
	@author Nicolas Williamson 
  @date January 2010
	
*/

#include <includes.h>

static volatile signed int mcu_led_green_count; /**< Curren count of the green led timer. */
static volatile signed int mcu_led_red_count; /**< Curren count of the red led timer. */
static volatile signed int mcu_led_blue_count; /**< Curren count of the blue led timer. */

/**
  Initializes the mcu_led module.
  Call this from software setup.
*/
void mcu_led_init(void){
  mcu_led_green_count = -1;
  mcu_led_red_count = -1;
  mcu_led_blue_count = -1;
}

/**
  Updates all of the leds.
  Call this from every row of the schedule.
*/
void mcu_led_update(void){
  if (mcu_led_green_count > -1){mcu_led_green_count--;} 
  if (mcu_led_green_count == 0) {mcu_led_green_off();}
  if (mcu_led_blue_count > -1){mcu_led_blue_count--;} 
  if (mcu_led_blue_count == 0) {mcu_led_blue_off();}
  if (mcu_led_red_count > -1){mcu_led_red_count--;} 
  if (mcu_led_red_count == 0) {mcu_led_red_off();}
}

/**
  Turns on the green led for the given amount of time.
  @param time_ms The time the led should be on, in milliseconds.
  @note If SCHED_SPEED is not defined, this function will not work properly.
*/
void mcu_led_green_blink(int time_ms){
  mcu_led_green_count = time_ms * SCHED_SPEED;
  mcu_led_green_on();
}
/**
  Turns on the blue led for the given amount of time.
  @param time_ms The time the led should be on, in milliseconds.
  @note If SCHED_SPEED is not defined, this function will not work properly.
*/
void mcu_led_blue_blink(int time_ms){
  mcu_led_blue_count = time_ms * SCHED_SPEED;
  mcu_led_blue_on();
}
/**
  Turns on the red led for the given amount of time.
  @param time_ms The time the led should be on, in milliseconds.
  @note If SCHED_SPEED is not defined, this function will not work properly.
*/
void mcu_led_red_blink(int time_ms){
  mcu_led_red_count = time_ms * SCHED_SPEED;
  mcu_led_red_on();
}

void mcu_led_green_on(void){ FIO1CLR = (1<<23); }
void mcu_led_green_off(void){ FIO1SET = (1<<23); }
void mcu_led_green_toggle(void){ if(FIO1PIN & (1<<23)){mcu_led_green_on();} else{mcu_led_green_off();}}
void mcu_led_red_on(void){ FIO1CLR = (1<<24); }
void mcu_led_red_off(void){ FIO1SET = (1<<24); }
void mcu_led_red_toggle(void){ if(FIO1PIN & (1<<24)){mcu_led_red_on();} else{mcu_led_red_off();}}
void mcu_led_blue_on(void){ FIO1CLR = (1<<25); }
void mcu_led_blue_off(void){ FIO1SET = (1<<25); }
void mcu_led_blue_toggle(void){ if(FIO1PIN & (1<<25)){mcu_led_blue_on();} else{mcu_led_blue_off();}}
void mcu_led_all_on(void){
  mcu_led_green_on();
  mcu_led_red_on();
  mcu_led_blue_on();
}
void mcu_led_all_off(void){
  mcu_led_green_off();
  mcu_led_red_off();
  mcu_led_blue_off();
}
void mcu_led_all_toggle(void){
  mcu_led_green_toggle();
  mcu_led_red_toggle();
  mcu_led_blue_toggle();
}

