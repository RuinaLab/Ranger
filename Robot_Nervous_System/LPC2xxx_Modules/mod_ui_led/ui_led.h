/*
	ui_led.h
	
	Nicolas Williamson - July 2009
*/

#ifndef __UI_LED_H__
#define __UI_LED_H__

//Only for use in RGB
#define UI_LED_RED 100,0,0
#define UI_LED_ORANGE 100,50,0
#define UI_LED_YELLOW 50,100,0
#define UI_LED_GREEN 0,100,0
#define UI_LED_BLUE 0,0,100
#define UI_LED_PURPLE 75,0,75
#define UI_LED_WHITE 200,200,200
#define UI_LED_OFF 0,0,0

//Functions
void ui_led_init(void);
void ui_led_register_write(unsigned int device, unsigned int address, unsigned int w_data);
void ui_led_pwm(unsigned int led_num, unsigned int color, unsigned int pwm);
void ui_led_rgb(unsigned int led_num, unsigned int red_level, unsigned int green_level, unsigned int blue_level);


/* HARDWARE SETUP

	// *******************************************************************************
	// Initialize SSP/SPI1 for LED Control
	// *******************************************************************************
	PCONP &= ~(1<<10);		// power setting: disable SPI1	(bit 10)
	PCONP |= (1<<21);		// power setting: enable SSP (bit 21!!!!!!!!!!!!!!!!!!!!!!)
	SSPCR1 = 0;				// Disable SSP to allow setting changes
	SSPCR0 = 0;
	SSPCR0 |= (15<<0);		// data size: 16 bits (bits 0-3 = 15 means 16bit data)
	SSPCR0 &= ~(3<<4);		// SPI mode (bits 4-5 = 0)
	SSPCR0 &= ~(1<<6);		//sclk low when idle - Motorola format
	SSPCR0 &= ~(1<<7);		//sample on the first edge (rising edge)
//	SSPCPSR &= ~(3<<0);		//clear prescale divider
	SSPCPSR |= 2;		// prescale divider
	SSPCR0 |= (1<<8);		// bit frequency = PCLK/(CPSDVSR*(SCR+1)) = 15 Mbits/sec
	PINSEL1 &= ~0x3FC;		// clear P0.17~P0.20;
	PINSEL1 |= (2<<2)|(2<<4)|(2<<6);		// P0.17: SCK1, P0.18: MISO1, P0.19: MOSI1, P0.20: Manual SSEL 
//	PINSEL1 |= (2<<2)|(2<<4)|(2<<6)|(2<<8);	// P0.17: SCK1, P0.18: MISO1, P0.19: MOSI1, P0.20: auto SSEL
	FIO0DIR |= (1<<20); 	//SSEL is output
	FIO0SET = (1<<20); 		//SSEL high
	SSPCR1 |= (1<<1);		// enable SSP/SPI1	 
  
*/


#endif /* __UI_LED_H__ */
