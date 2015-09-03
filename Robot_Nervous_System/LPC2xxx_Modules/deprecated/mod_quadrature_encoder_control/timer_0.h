
/* Header file for Timer_0.c */

#ifndef Timer_0
#define Timer_0

/* Hardware Setup - 
	// *******************************************************************************
	// Interrupt Handlers - Timer0
	// *******************************************************************************
    //Set up the Timer Counter 0 Interrupt
    //Used to recod length of time between encoder pulses.
		VICSoftIntClear = 0xffffffff;		 //clear all interupts - only do this once
		T0IR  = 1;                           //clear interrupts in Timer0
		T0MR0 = CPUSPEED/timespersec0;       // Match Register 0: 20 msec(50 Hz) with 48 MHz clock
		T0MCR = 3;                           // Match Control Reg: Interrupt(b0) and Reset(b1) on MR0
		T0TCR = 1;                           // Timer0 Enable
		T0IR  = 1;                           //clear interrupts in Timer0
	//FIQ Encoder Interrupt - use modified version of startup.s - as per instruction given in Realview Guide
		//VICIntSelect |= 1 << 4;			   // classify interrupt as FIQ
		//VICIntEnable |= 1 << 4;              // Enable Timer0 Interrupt bit 4 (1 sets the bit)
	//Non-FIQ Version - use default startup.s
		VICVectAddr1 = (unsigned long)TIMER0_ISR_Handler;   // Use slot 1, second highest vectored IRQ priority.
		VICVectCntl1 = 0x20 | 4;             // 0x20 is the interrupt enable bit, 0x04 is the TIMER0 channel number
		VICIntEnable |= 1 << 4;              // Enable Timer0 Interrupt bit 4 (1 sets the bit) 
*/
 

//Function Declarations
	//see "Quadrature_Encoder_Control.c" for more interrupt handlers, 
	//	namely "QEC_update_encoder_position" and "QEC_time_counter."


 #endif
