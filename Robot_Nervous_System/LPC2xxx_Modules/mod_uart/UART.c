/*
 *  UART.c
 *
 */

#include <includes.h>

//*******************************************************************************
// sendchar/getkey function	using UART0
//*******************************************************************************
/* This is necessary for printf function. 
 Make sure to include the right version of Retarget.c in the project workspace.*/

/* Send characters via Serial Port */
#define CR 0x0D
int sendchar (int ch){
	if (ch == '\n') {
		while (!(U0LSR & 0x20)){};
		U0THR = CR; /* output CR */
	}
	while (!(U0LSR & 0x20)){};
	return (U0THR = ch);	  	
}	
/* Read character from Serial Port   */
int getkey (void){                    
	while (!(U0LSR & 0x01)){};	
	return (U0RBR);
}

