/**
	@file buzzer.c
	
  Control of the buzzer on the UI board.
  
  Example hardware/register setup of PWM for buzzer:
  @code
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
  @endcode
  
	@author Nicolas Williamson 
  @date July 2009
*/

#include <includes.h>
#include <math.h>

#define PI 3.14159265

int buzzer_hz = 4000; /**< The frequency of the buzzer. */
int buzzer_is_on = 0;
int buzzer_hz_init = 600;
int buzzer_hz_range = 300;
int	buzzer_period = 3000;

/**
  Turns the buzzer on.
*/
void buzzer_on(void){
	PWMMR0 = 60000000/buzzer_hz;
	PWMMR5 = PWMMR0/2;
	PWMLER = (1<<0)|(1<<5);
}

/**
  Turns the buzzer off.
*/
void buzzer_off(void){
	PWMMR5 = 0;
	PWMLER = (1<<5);
}

/**
  Sets the frequency of the buzzer.
  @param hz The new frequency of the buzzer.
*/
void buzzer_set_frequency(int hz){
	buzzer_hz = hz;
  if (buzzer_is_on){
    buzzer_on();
  }
}

void buzzer_sine(int period){
	int i;
	buzzer_period = period;
	for(i = 0; i < buzzer_period; i++){
		buzzer_hz = buzzer_hz_init + buzzer_hz_range * sin(buzzer_period * PI/180);
	}
	if (buzzer_is_on){
    	buzzer_on();
  	}
}
