/*

Timer module. Sets up timer interrupts.

Nic Williamson - March 2009
Cornell University

*/

#include "includes.h"

unsigned int on = 0;

void time_counter (void) __irq
{ 
  	if(on){
      FIO1SET_bit.P1_25 = 1;		 // Blue LED OFF
	  on = 0;

	}
	else{
	  FIO1CLR_bit.P1_25 = 1;		 // Blue LED ON
	  on = 1;
	}
	
  T0IR        = 1;    	// Clear interrupt flag
  VICVectAddr = 0;		// Dummy write to indicate end of interrupt service
}



