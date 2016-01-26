/*

	interrupts.h
	
	Nicolas Williamson - Dec 2009
	
*/

#ifndef __INTERRUPTS_H__
#define __INTERRUPTS_H__

typedef void(*ISR)(void);

//Public
void init_interrupts(void);
void FIQ_Handler(void) __irq;

//Private
void add_fiq(int num, ISR isr);

#endif
