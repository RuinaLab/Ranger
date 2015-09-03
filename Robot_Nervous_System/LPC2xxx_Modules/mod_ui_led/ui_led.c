/*
	led.c
	
	providing control of the LEDs on the UI board since 2009.
	
	Nicolas Williamson - July 2009
	Adapted from code by Emily Lee
*/

#include <includes.h>	

#define LED_BUSY (SSPSR & (1<<4))
#define LED_TNF (SSPSR & (1<<1))
#define LED_RNE (SSPSR & (1<<2))
#define SSEL_LO FIO0CLR = (1<<20);
#define SSEL_HI FIO0SET = (1<<20);

#define R 2  // red
#define G 1	 // green
#define B 0  // blue
#define NO_OP 0x2000  // no-op

void ui_led_init(void)
{
	while(!LED_TNF){}   // wait if transmit buffer is full
	SSEL_LO;
	SSPDR = 0x1001;		// configuration: run mode
	while(!LED_TNF){}   // wait if transmit buffer is full
	SSPDR = 0x1001;		// configuration: run mode
	while(LED_BUSY) {}	// wait if transfering data
	SSEL_HI;

	while(!LED_TNF){}   // wait if transmit buffer is full
	SSEL_LO;
	SSPDR = 0x0A01;				// turn off all LEDs, 16 bit
	while(!LED_TNF){}   // wait if transmit buffer is full
	SSPDR = 0x0A01;				// turn off all LEDs, 16 bit
	while(LED_BUSY) {}	// wait if transfering data
	SSEL_HI;
}

void ui_led_register_write(unsigned int device, unsigned int address, unsigned int w_data)
{
	// device 1 or 2 indicates first or second max6966 chip
	if (device == 1){
		while(!LED_TNF){}    // wait if transmit buffer is full
		SSEL_LO;        // SSEL1 low
		SSPDR = NO_OP;				   // dummy data for device 2
		while(!LED_TNF){}    // wait if transmit buffer is full
		SSPDR = (address << 8) | w_data; // data for device 1
		while(LED_BUSY){}	   // wait if transfering data
		SSEL_HI;        // SSEL1 high, latch data to devices
	} else if (device == 2){	
		while(!LED_TNF){}    // wait if transmit buffer is full
		SSEL_LO;        // SSEL1 low
		SSPDR = (address << 8) | w_data; // data for device 2
		while(!LED_TNF){}    // wait if transmit buffer is full
		SSPDR = NO_OP;				   // dummy data for device 1
		while(LED_BUSY){}	   // wait if transfering data
		SSEL_HI;        // SSEL1 high
	}
}

void ui_led_pwm(unsigned int ui_led_num, unsigned int color, unsigned int pwm)
{
	//PWM level should be between 3 and 254
	unsigned int dev, addr, tmp;
	
	if (pwm < 3){
		pwm = 1;
	} else if (pwm > 254){
		pwm = 0;
	}
	
	tmp = (ui_led_num * 3) + color;
	dev = (tmp / 9) + 1; //leds 0-2 are on device 1, and leds 3-5 are on device 2
	addr = tmp % 9;  

	ui_led_register_write(dev, addr, pwm);
}

void ui_led_rgb(unsigned int ui_led_num, unsigned int red_level, unsigned int green_level, unsigned int blue_level)
{
	//PWM level should be between 3 and 254
	ui_led_pwm(ui_led_num, R, red_level);
	ui_led_pwm(ui_led_num, G, green_level);
	ui_led_pwm(ui_led_num, B, blue_level);
}


