/*

	interrupts.c
	Sets up the vectored interrupts and the fast interrupt handler
	
	Nicolas Williamson - Dec 2009
	
*/

#include <includes.h>

VOID_VOID_F isrs[32];
unsigned int vic_nums[32];
unsigned int vect_count = 0;

void init_interrupts(void){
	// *************************************************
	// Interrupt enables
	// *************************************************
	VICIntEnable = 0;
	VICVectAddr  = 0;
	VICSoftIntClr = 0xffffffff;  	 //clear all interupts - only do this once

  // ************ PRIORITY 0 ******************
	//CAN2 RX
	VICVectAddr0 = (unsigned long)can_rx2_isr;
	VICVectCntl0 = (1 << 5) | VIC_CAN2RX;
	VICIntEnable |= (1 << VIC_CAN2RX);
  
	// ************ PRIORITY 1 ******************
	//Timer1, used for LCD, also used for scheduler
	VICVectAddr1 = (unsigned long)lcd_isr;
	VICVectCntl1 = (1<<5) | VIC_TIMER1;
	VICIntEnable = 1<<VIC_TIMER1;

  // ************ PRIORITY 2 ******************  
  //Timer0, used for RC receive
  VICVectAddr2 = (unsigned long)rcx_isr;
  VICVectCntl2 = (1<<5) | VIC_TIMER0;
  VICIntEnable = (1<<VIC_TIMER0);    
  
  // ************ PRIORITY 3 ******************
  VICVectAddr3 = (unsigned long)msimu_isr;	//Set address of interrupt service routine
  VICVectCntl3 = (1<<5) + VIC_UART1;	//Enable vectored interrupt slot 3 and set it to UART1 (interrupt source 7)
  VICIntEnable = (1<<VIC_UART1);		//Enable UART1 interrupt at vectored interrupt controller (VIC)

	// ************ PRIORITY 5 ******************
	//CAN2 RX
  //VICVectAddr5 = (unsigned long)can_rx1_isr;
  //VICVectCntl5 = 0x20 | VIC_CAN1RX;
  //VICIntEnable = 1 << VIC_CAN1RX;
	
	// ************ PRIORITY 6 ******************
	//CAN2 TX
  //VICVectAddr6 = (unsigned long)can_tx1_isr;
  //VICVectCntl6 = 0x20 | VIC_CAN1TX;
  //VICIntEnable = 1 << VIC_CAN1TX;
  
	// ************ PRIORITY 7 ******************
	//CAN2 RX
  //VICVectAddr7 = (unsigned long)can_rx2_isr;
  //VICVectCntl7 = 0x20 | VIC_CAN2RX;
	//VICIntEnable = 1 << VIC_CAN2RX;
	
	// ************ PRIORITY 8 ******************
	//CAN2 TX
//	VICVectAddr8 = (unsigned long)can_tx2_isr;
//	VICVectCntl8 = 0x20 | VIC_CAN2TX;
//	VICIntEnable = 1 << VIC_CAN2TX;
	
	// ************ PRIORITY 9 ******************
	//CAN ERRORS
	VICVectAddr9 = (unsigned long)can_error_isr;
	VICVectCntl9 = (1<<5) | VIC_CAN_AF;
	VICIntEnable = 1<<VIC_CAN_AF;	 
}

void FIQ_Handler(void) __irq
{
  //Empty FIQ handler - put code here if FIQ enabled.
}
