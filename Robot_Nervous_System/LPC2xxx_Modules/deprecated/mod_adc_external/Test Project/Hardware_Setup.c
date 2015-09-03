/*
Hardware Setup File

Specific to LPC2194_01 motor control board on Ranger
Cornell University

Nicolas Williamson - March 2009
*/

#include "includes.h"

//Sets up board specific hardware stuff (including interrupts). MUST BE CALLED!!
int setup_hardware(void)
{
	int timespersec = 2;
	//*******************************************************************************
	// Initialize PLL
	//*******************************************************************************
	
	// PLLCFG: 0 pp mmmmm where pp=PSEL and mmmmm=MSEL.
	PLLCFG = MSEL | (PSEL<<5);
	// PLLCON: 000000 C E  C=connect, E=enable. Enable, wait for lock then C+E
	PLLCON = 0x00000001;
	// Give the connect sequence
	PLLFEED = 0x000000AA;
	PLLFEED = 0x00000055;
	while(!(PLLSTAT & 0x00000400)) ;	// Wait for PLL to lock (bit 10 is PLOCK)
	PLLCON = 0x00000003;	// Enable and Connect
	PLLFEED = 0x000000AA;
	PLLFEED = 0x00000055;
	VPBDIV = 0x00000001;
	
	
	//*******************************************************************************
	// Initialize UART0 for printf function
	//*******************************************************************************
	
	U0LCR=0x83;  // DLAB=1
	U0DLL=32;
	U0DLM=0;
	U0FCR|=1;    // FIFO
	U0LCR=0x3;   // DLAB=0
	PINSEL0_bit.P0_0 = 1;    // UART0 TXD
	PINSEL0_bit.P0_1 = 1;    // UART0 RXD
	
	
	//*******************************************************************************
	// Interrupt Handlers
	//*******************************************************************************
	// Timer Counter 0 Interrupt executes each 20ms @ 48 MHz CPU Clock
	// Increment counters timeval for general program use.	
	VICSoftIntClear = 0xffffffff;        // clear all interupts

	// Set up the Timer Counter 0 Interrupt
	// Used to blink the activity light
	T0MR0 = CPUSPEED/timespersec;        // Match Register 0: 20 msec(50 Hz) with 48 MHz clock
	T0MCR = 3;                           // Match Control Reg: Interrupt(b0) and Reset(b1) on MR0
	T0TCR = 1;                           // Timer0 Enable
	T0IR  = 1;                        //clear interrupts in Timer0
	VICVectAddr1 = (unsigned long)time_counter;   // Use slot 1, second highest vectored IRQ priority.
	VICVectCntl1 = (1<<5) | 4;             // 0x20 is the interrupt enable bit, 0x04 is the TIMER0 channel number
	VICIntEnable |=  0x000000010;

	PINSEL0_bit.P0_3 = 3;				// Set Pin3 to EINT1 (ADCBUSY line)
	EXTMODE |= (1<<1);						 // Set EINT1 (Bit 1) to be edge-sensitive
	EXTPOLAR |= 0x0;						// Set EINT1 to be sensitive on the falling-edge
	EXTINT = 1;							 // Clear all interrupt register flags
 	VICVectAddr0 = (unsigned long)AtoD;  // Use slot 0, the highest vectored IRQ priority.
 	VICVectCntl0 = (1<<5) | 15;             // (1<<5) is the interrupt enable bit, 15 is EINT1 interrupt
	VICIntEnable |= (1<<15); //Enable EINT1 (bit 15)
	
	// *******************************************************************************
	// OTHER
	// *******************************************************************************
	MAMCR_bit.MODECTRL=2;   // fully enable memory accelerator module
	SCS_bit.GPIO0M=1;       // enable high-speed GPIO for port0 (Fast GPIO)
	SCS_bit.GPIO1M=1;       // enable high-speed GPIO1 (Fast GPIO)	
	
	//On-board LED initialization
	FIO1DIR_bit.P1_23=1;   // set P1.23 to be output (Green LED)
	FIO1DIR_bit.P1_24=1;   // set P1.24 to be output (Red LED)
	FIO1DIR_bit.P1_25=1;   // set P1.25 to be output (Blue LED)
	FIO1SET_bit.P1_23=1; 	 // turn off green LED
	FIO1SET_bit.P1_24=1;	 // turn off red LED
	FIO1SET_bit.P1_25=1;	 // turn off blue LED
	
	
	// *******************************************************************************
	// Initialize SSP/SPI1 For External ADC
	// *******************************************************************************
	
	SSPCR1 = 0;					// Disable SSP to allow setting changes
	PCONP_bit.PCSPI1=0;   // power setting: disable SPI1
	PCONP_bit.PCSSP=1;    // power setting: enable SSP
	SSPCR0_bit.DSS= 0x7;     // data size: 8 bits
	SSPCR0_bit.FRF=0;     // SPI mode
	SSPCR0_bit.CPOL=1;    // sclk high when idle
	SSPCR0_bit.CPHA=1;    // sample on the second edge (rising edge)
	SSPCPSR_bit.CPSDVSR= 10; // prescale divider
	SSPCR0_bit.SCR= 2;    // bit frequency = PCLK/(CPSDVSR*(SCR+1)) = 2Mbps	
	PINSEL1 &= ~0x3FC;  // clear P0.17~P0.20;
	PINSEL1 |= 0x2A8;   // P0.17: SCK1, P0.18: MISO1, P0.19: MOSI1, P0.20: SSEL (auto SSEL1)
	IO0DIR_bit.P0_20=1;       // set manual SSEL1 as output
	IO0SET_bit.P0_20=1;       // SSEL1 high
	SSPCR1_bit.SSE=1;        // enable SSP/SPI1
	
	PINSEL0_bit.P0_12 = 0;  //ADC reset line set for GPIO operation
	PINSEL2 &= ~(1<<3);	  // set trace port for GPIO use (bit 3 = 0)
  						  //Should happen automatically at startup, but just in case...
						  //GPIO P1_16 is the ADC convert line
 	FIO0DIR_bit.P0_12=1;    // set P0_12 (ADC reset) to be a digital output
	FIO1DIR_bit.P1_16=1;    // set P1_16 (ADC convert) to be a digital output
	
	FIO0SET_bit.P0_12=1; 	 //set ADC reset line high
 	FIO1CLR_bit.P1_16=1; 	 //set ADC convert line low
	
	// Setup ADC registers

	ADC_external_register_write(3,(1<<2));  // two's comp output, autoreadback: MSB first, CCLK div = 1
	//ADC_external_register_write(6,0x0F);  //Temp; GPIO output
	ADC_external_register_write(7,0x00);  // Vref=2.5V, buffer and ref powered down (using external 5V reference)
	ADC_external_register_write(24,0x00); // SPI: MSB first, 3-wire mode, DIN stuff 
	while(SSPSR_bit.BSY){} //Wait for data to be sent
	while(SSPSR_bit.RNE){	 //Read returning dummy byte to clear out buffer.
    	ADC_external_read_buffer();
 	}	
	
	
	/*********DONE**********/
	
	return 0;
}

