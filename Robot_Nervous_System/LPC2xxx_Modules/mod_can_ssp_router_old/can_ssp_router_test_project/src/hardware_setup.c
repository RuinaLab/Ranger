#include <includes.h>



void timer0_isr(void) __irq {

  #ifdef DEBUG
    if (FIO0PIN & 1<<14)
    {
      VICIntEnClr = 0xFFFFFFFF; //Disable all interrupts before debugging
      VICVectAddr = 0;
      return;
    }
  #endif


  T0IR = 0xFFFFFFFF;  // Clear ALL Timer0 interrupt flags.
  schedule_tick();
  VICVectAddr = 0;    // Clear interrupt in VIC.  
}


///////////////////////////////////////////////////////////////////////////////////////////////
void setup_hardware(void){

  //Reset interrupt controller (mostly for proper debugger operation)
  VICIntEnable = 0;
  VICVectAddr  = 0;
  VICVectAddr  = 0;
  VICVectAddr  = 0;
  VICVectAddr  = 0;
  VICVectAddr  = 0;
  
  /*********************************************************************/
  /* Timer0 Initialization (except interrupts)                                            */
  /*********************************************************************/
  //initialize Timer0 for 1 ms ticks   
  T0TCR = 0;//enabled for counting
  T0PR = 0;//no prescaling
  //T0PR = 9;//no prescaling
  //TODO: Update this value for 10 MHz clock...currently for 12 MHz (already done?)
  T0MR0 = (60000-1);//1 mS
  T0CTCR = 0;//simple timer
  T0MCR = 0x00000003;//interrupt and reset on MR0 
  T0TCR = 1;//enabled for counting 
  /*********************************************
   * Put user initialization code below here.  *
   *********************************************/ 
  /***********************************************
   * Heartbeat Init Section
   * Set pin direction to output for driving LEDs
   * Set initial LED state
   ***********************************************/
   
//  SCS |= 1<<0;//Enable fast gpio on port 0.
//  SCS |= 1<<1;//Enable fast gpio on port 1.
  SCS = 3;    //Enable fast gpio on ports 0 and 1 (do not write reserved bits)
 
  FIO1DIR = 1<<23|1<<24|1<<25;
  FIO1SET = 1<<23|1<<24|1<<25; 

  /********* End Heartbeat Init Section ************/
	//SSP setup
  PCONP &= ~(1<<10);  // power setting: disable SPI1
  PCONP |= 1<<21;    	// power setting: enable SSP

  PINSEL1 &= ~0x3FC;  // clear P.017~P0.20;
  PINSEL1 |= 0x2A8;   // P0.17: SCK1, P0.18: MISO1, P0.19: MOSI1, P0.20: SSEL1

  SSPCR0 = 0x004F;     	// data size: 16 bits
  						// SPI mode
  						// CPOL=1: sclk high when idle
  						// CPHA=0: sample on the first edge (falling edge)
						// Serial Clock Rate = 

  SSPCPSR = 253; 			// prescale divider
  

  SSPCR1 = 0;        	// disable SSP/SPI1 
  SSPCR1 = 0;			//Master mode, no loopback mode
  SSPCR1 |= 1<<1;       // enable SSP/SPI1

  SSPIMSC = 0xF;		//Enable all ssp interrupt sources
//  SSPIMSC = 1<<3;		//enable transmit interrupts only
//test code - prime SSP interrupts
  SSPDR = 0;
 // ******************************************************
  //UART0 setup
  U0FCR = 0x1;    	// Enable FIFO
  U0FCR = 0x87;	// Clear FIFOs and set to generate interrupt when RX FIFO reaches 8 bytes.

  U0LCR=0x83;  //Step 1: DLAB=1, enable baud rate divisor adjustments
  			   //Set to 8 data bits, 1 stop bit, no parity

			   //Step 2: Set baud rate divisor
  U0DLL=25;		//Baud rate divisor for 115200 kbaud, with (1+0.3) fractional divider 
  U0DLM=0;		//and peripheral clock = 60MHz.

  U0LCR &= ~(1<<7); // Step 3: DLAB=0, disable baud rate adjustments, continue with other setup

  U0FDR = 3;		// DIVADDVAL = 3
  U0FDR |= 10<<4;	// MULTVAL = 10
  //U0IER_01 = 5;		// Enable receive data and receive line status interrupts
  
  //U0ACR = 0;	   	// Autobaud disabled
  U0TER = 0x80;	   	// Transmit enable
  PINSEL0 |= 1<<0;    // UART0 TXD active on MCU pin P0[0]
  PINSEL0 &= ~(1<<1); //
  PINSEL0 |= 1<<2;    // UART0 RXD	active on MCU pin P0[1]
  PINSEL0 &= ~(1<<3); // 

  //VICProtection = 0;		//Allow modification of VIC registers in user mode
  //VICVectAddr3 = (unsigned long)can_ssp_router_uart0_isr;	//Set address of interrupt service routine
  //VICVectCntl3 = (1<<5) + 6;	//Enable vectored interrupt slot 3 and set it to UART0 (interrupt source 6)
 // VICIntSelect &= ~(1<<6);	//Clear bit 6 - use IRQ interrupts for UART0, not FIQ.
  //VICIntEnable = 1<<6;		//Enable UART0 interrupt at vectored interrupt controller (VIC)


// ********************************************************************
  //On-board LED initialization
  
  //Set LED lines to GPIO
  PINSEL1 &= ~(1<<28); // Set P0[30] to GPIO (default at reset is AIN3)
  PINSEL1 &= ~(1<<29); //

  FIO0DIR |= 1<<30;   		// set P0[30] to be output (Green CAN1 LED)
  FIO1DIR |= 1<<16;   		// set P1[16] to be output (Red CAN1 LED)

  FIO0DIR |= 1<<2;   		// set P0[2] to be output (Green CAN2 LED)
  FIO0DIR |= 1<<3;   		// set P0[3] to be output (Red CAN2 LED)

  FIO1DIR |= 1<<19;   		// set P1[19] to be output (Green CAN3 LED)
  FIO1DIR |= 1<<18;   		// set P1[18] to be output (Red CAN3 LED)

  FIO0DIR |= 1<<10;   		// set P0[10] to be output (Green CAN4 LED)
  FIO0DIR |= 1<<11;   		// set P0[11] to be output (Red CAN4 LED)

  FIO1DIR |= 1<<23;   		// set P1[23] to be output (Green MCU LED)
  FIO1DIR |= 1<<24;   		// set P1[24] to be output (Red MCU LED)
  FIO1DIR |= 1<<25;   		// set P1[25] to be output (Blue MCU LED)

  FIO0SET |= 1<<30; 	 	// turn off green CAN1 LED
  FIO1SET |= 1<<16;	 		// turn off red CAN1 LED

  FIO0SET |= 1<<2; 	 		// turn off green CAN2 LED
  FIO0SET |= 1<<3;	 		// turn off red CAN2 LED

  FIO1SET |= 1<<19; 	 	// turn off green CAN3 LED
  FIO1SET |= 1<<18;	 		// turn off red CAN3 LED

  FIO0SET |= 1<<10; 	 	// turn off green CAN4 LED
  FIO0SET |= 1<<11;	 		// turn off red CAN4 LED

  FIO1SET |= 1<<23; 	 	// turn off green MCU LED
  FIO1SET |= 1<<24;	 		// turn off red MCU LED
  FIO1SET |= 1<<25;	 		// turn off blue MCU LED

  //CAN 3 and 4 transceiver standby initialization (MAX3051 RS pin)
  FIO0DIR |= 1<<8;   		// set P0[8] to be output (CAN 3 RS)
  FIO0DIR |= 1<<9;   		// set P0[9] to be output (CAN 4 RS)

  FIO0CLR |= 1<<8; 	 		// turn on CAN3 transceiver (high is standby)
  FIO0CLR |= 1<<9;	 		// turn on CAN4 transceiver (high is standby)

  //Control lines to LPC3250 module GPI lines - set to output
  FIO1DIR |= 1<<17;   		// set P1[8] to be output (LPC3250 GPI_0)
  FIO1DIR |= 1<<20;   		// set P1[20] to be output (LPC3250 GPI_2)
  FIO1DIR |= 1<<21;   		// set P1[21] to be output (LPC3250 GPI_5)
  FIO1DIR |= 1<<22;   		// set P1[22] to be output (LPC3250 GPI_6)

  //Control/interrupt lines from LPC3250 module - set to input or interrupt
 FIO0DIR &= ~(1<<15);		//Set P0[15] to input
 FIO0DIR &= ~(1<<16);		//Set P0[16] to input



  //Internal ADC pin setup - use AIN0 for battery current, and AIN1 for battery voltage
  PINSEL1 |= 1<<22;    // AIN0 active on MCU pin P0[27]
  PINSEL1 &= ~(1<<23); //
  PINSEL1 |= 1<<24;    // AIN1 active on MCU pin P0[28]
  PINSEL1 &= ~(1<<25); // 

 }
 /*
void timer0_isr(void) __irq {
  T0IR = 0xFFFFFFFF;  // Clear ALL Timer0 interrupt flags.
  schedule_tick();
  VICVectAddr = 0;    // Clear interrupt in VIC.  
}
*/
