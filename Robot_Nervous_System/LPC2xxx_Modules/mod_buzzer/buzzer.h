/* 
	buzzer.h
	
	Nicolas Williamson - July 2009
*/

#ifndef __H_BUZZER__
#define __H_BUZZER__

void buzzer_on(void);
void buzzer_off(void);
void buzzer_set_frequency(int hz);
void buzzer_sine(int period);

/* HARDWARE SETUP

	// *******************************************************************************
	// Buzzer PWM Setup
	// *******************************************************************************
	PINSEL1 &= ~(3<<10);
	PINSEL1 |= (1<<10);  //PWM 5 on Pin P0.21 Enabled (bits 10/11)
	PWMPR = 0; //Will run at maximum rate
	PWMMR0 = 15000; 
	PWMMR5 = 0;
	PWMMCR = (1<<1); //Should set Timer Counter to reset upon reaching Match Register 0
	PWMTCR = (1<<0)|(1<<3); //Enables Timer Counter and PWM Mode.
	PWMPCR = (1<<13); //enable PWM5
	PWMLER = (1<<0)|(1<<5); //update PWMMR values
  
*/

#endif
