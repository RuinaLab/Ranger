/*
	button.h
	
	Nicolas Williamson - July 2009
*/

#ifndef __H_BUTTON__
#define __H_BUTTON__

unsigned int button_pushed(unsigned int button);
unsigned int button_0(void);
unsigned int button_1(void);
unsigned int button_2(void);
unsigned int button_3(void);
unsigned int button_4(void);
unsigned int button_5(void);

/* HARDWARE SETUP

	// *******************************************************************************
	// Initialize Buttons
	// *******************************************************************************
	PINSEL0 &= ~(3<<24); //Button 1: P0.12 as GPIO
	PINSEL0 &= ~(3<<26); //Button 2: P0.13 as GPIO
	PINSEL1 &= ~(3<<24); //Button 3: P0.28 as GPIO
	PINSEL1 &= ~(3<<28); //Button 5: P0.30 as GPIO
	PINSEL2 &= ~(1<<3);  //Buttons 0 & 4: P1.18 and P1.16 GPIO (P1.16-25 GPIO)
	//set pins as inputs
	FIO0DIR &= ~(1<<12); 
	FIO0DIR &= ~(1<<13);
	FIO0DIR &= ~(1<<28);
	FIO0DIR &= ~(1<<30);
	FIO1DIR &= ~(1<<16);
	FIO1DIR &= ~(1<<18);
  
*/

#endif
