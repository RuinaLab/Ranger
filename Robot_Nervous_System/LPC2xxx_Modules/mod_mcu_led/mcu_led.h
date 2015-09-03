/*
	
	@file mcu_led.h
	
	@author Nicolas Williamson 
  @date January 2010
	
*/

/*
  // ***********************************************
  // Heartbeat Init Section
  // ***********************************************
  //On-board LED initialization
  PINSEL2 &= ~(1<<3);   // set trace port for GPIO use (bit 3 = 0)
  FIO1DIR |= (1<<23);   // set P1.23 to be output (Green LED)
  FIO1DIR |= (1<<24);   // set P1.24 to be output (Red LED)
  FIO1DIR |= (1<<25);   // set P1.25 to be output (Blue LED)
  MCU_LED_ALL_OFF;
*/ 

#ifndef __MCU_LED_H__
#define __MCU_LED_H__

//Public Functions
void mcu_led_init(void);

void mcu_led_update(void);

void mcu_led_green_blink(int time_ms);
void mcu_led_blue_blink(int time_ms);
void mcu_led_red_blink(int time_ms);
void mcu_led_green_on(void);
void mcu_led_green_off(void);
void mcu_led_green_toggle(void);
void mcu_led_red_on(void);
void mcu_led_red_off(void);
void mcu_led_red_toggle(void);
void mcu_led_blue_on(void);
void mcu_led_blue_off(void);
void mcu_led_blue_toggle(void);
void mcu_led_all_on(void);
void mcu_led_all_off(void);
void mcu_led_all_toggle(void);


#endif
