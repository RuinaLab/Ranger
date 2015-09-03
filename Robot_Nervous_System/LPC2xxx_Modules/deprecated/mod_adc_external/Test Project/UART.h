/*
 *  UART.h
 *  
 *
 *  Created by Nicolas Williamson on 3/18/09.
 *  Copyright 2009 Cornell University. All rights reserved.
 *
 */

#ifndef UART_SETUP
#define UART_SETUP

/* HARDWARE SETUP: Put into Hardware_Setup.c setup_hardware() function

 // *******************************************************************************
 // Initialize UART0 for printf function
 // *******************************************************************************
 
 U0LCR=0x83;  // DLAB=1
 U0DLL=32;
 U0DLM=0;
 U0FCR|=1;    // FIFO
 U0LCR=0x3;   // DLAB=0
 PINSEL0_bit.P0_0 = 1;    // UART0 TXD
 PINSEL0_bit.P0_1 = 1;    // UART0 RXD

*/

//Function Headers
int sendchar (int ch);
int getkey (void);


#endif
