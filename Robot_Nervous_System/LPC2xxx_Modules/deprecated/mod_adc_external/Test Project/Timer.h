/* Header file for Timer.c */

/* Hardware Setup - (THIS SHOULD BE CHANGED BY SOMEONE WITH INTERRUPT KNOWLEDGE)

	// *******************************************************************************
	// Interrupt Handlers
	// *******************************************************************************
	
	// Timer Counter 0 Interrupt executes each 20ms @ 48 MHz CPU Clock
	// Increment counters timeval for general program use.	
	int timespersec = 2;
	VICSoftIntClear = 0xffffffff;		   //clear all interupts
	// Set up the Timer Counter 0 Interrupt
	// Used to blink the activity light
	T0MR0 = CPUSPEED/timespersec;        // Match Register 0: 20 msec(50 Hz) with 48 MHz clock
	T0MCR = 3;                           // Match Control Reg: Interrupt(b0) and Reset(b1) on MR0
	T0TCR = 1;                           // Timer0 Enable
	T0IR  = 1;                        //clear interrupts in Timer0
	VICVectAddr1 = (unsigned long)time_counter;   // Use slot 1, second highest vectored IRQ priority.
	VICVectCntl1 = 0x20 | 4;             // 0x20 is the interrupt enable bit, 0x04 is the TIMER0 channel number
	VICIntEnable = 0x00000010;
*/

//Function Declaration

void time_counter(void) __irq;

