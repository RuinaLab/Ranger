#include <mb_includes.h>

void mb_setup_hardware(void){

//unsigned long int temp = 0;

 /////////////////////////////////////////////////////////////////
  //Standard Timer 0 Config, 1 ms ticks

 // MS_CTRL = 1<<10; 	//TEST (temporarily disable SD card, to enable T0 match0 output)
  //MS_CTRL |= 7<<6;

  TIMCLK_CTRL1 |= 1<<2;	//Enable timer 0 clock (13 MHz peripheral clock)
  T0PR = 0;				//Divides input clock by PR (prescale register) + 1;
  T0MR0 = 13000 - 1;	//Timer counter will reset and interrupt at MR (match register) + 1 intervals

  //T0EMR |= 1<<4;	// Test code - toggle Match output 0.0 with each match event
  //T0EMR |= 1<<5;	// (frequency = 1/(2*timer period)

  T0CTCR = 	0;		// Timer mode - every PCLK rising edge increments prescale counter

  T0MCR = 3;		//Interrupt and reset on match register 0
  T0CCR = 0;		//Disable capture registers and their interrupts
  T0TCR = 1<<1;		//Reset counter
  T0TCR = 1<<0;		//Enable counter
  	
  MIC_ER |= 1<<16;		//Standard Timer/Counter 0 interrupt enable
  MIC_APR &= ~(1<<16);	//Standard Timer/Counter 0 is active low level
  MIC_ATR &= ~(1<<16);	//Standard Timer/Counter 0 interrupt is level-actuated
  
/////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////
  //Standard Timer 1 Config, free-run 
  TIMCLK_CTRL1 |= 1<<3;	//Enable timer 1 clock (13 MHz peripheral clock)
  T1PR = 0;				//Divides input clock by PR (prescale register) + 1;

  T1CTCR = 	0;		// Timer mode - every PCLK rising edge increments prescale counter

 
  T1CCR = 0;		//Disable capture registers and their interrupts
  T1TCR = 1<<1;		//Reset counter
  T1TCR = 0;		//Bring reset bit low
 
/////////////////////////////////////////////////////////////////


  
  /***********************************************
   * Heartbeat Init Section
   * Set pin direction to output for driving LEDs
   * Set initial LED state
   ***********************************************/
  P2_DIR_SET = 1<<25;		//Set GPIO_0 to output (goes to LPC2194 P0[15])
  P2_DIR_SET = 1<<26;		//Set GPIO_1 to output (goes to LPC2194 P0[16])
  
  /********* End Heartbeat Init Section ************/


  LCD_CTRL = 0;		//Disable LCD (would overwrite other pin functions if on)

//////////////////////////////////////////////////////////////////////////////
//SSP1 setup

 // SSP_CTRL |= 0x32;		  //Enable SSP1 DMA connection and clock

  

  P_MUX_SET = 1<<8;		// Set SPI2_CLK/SCK1 pin to SCK1
  P_MUX_SET = 1<<6;		// Set SPI2_DATIN/MISO1 pin to MISO1
  P_MUX_SET = 1<<5;		// Set SPI2_DATIO/MOSI1 pin to MOSI1
  P2_MUX_SET = 1<<4;	// Set GPIO_04 to SSEL1
  //Note - may be overridden by LCD, if enabled.

  SSP_CTRL = 1<<1;  // enable ssp1 clock
  SSP_CTRL |= 1<<4;	// enable ssp1 tx dma
  SSP_CTRL |= 1<<5; // enable ssp1 rx dma
  
  SSP1CR0 = 0x4F;    // data size: 16 bits
  					 // SPI mode
  		             // CPOL = 1 (sclk high when idle)
					 // CPHA = 0 (sample on the falling edge)
  					 // Serial clock rate divider = 0 (divide by 1)

  SSP1CPSR = 12;     // prescale divider

  SSP1CR1 = 0;		//Clear all bits. Enable bit must be zero
  SSP1CR1 = 1<<2;	//Set slave mode, no loopback mode or slave output disable.	
 // SSP1CR1 |= 1<<0;	//Set  loopback mode test
  //SSP1CR1 &= ~(1<<2); //Set master mode test

 // SSP1IMSC = 7;	//Enable interrupts on overrun, receive time-out, and RX FIFO > half full
  SSP1IMSC = 0;	//for DMA - disable all ssp1 interrupts

 SSP1DMACR = 3;	 //Enable transmit and receive FIFO for DMA
  	
//  SSP1CR1 |= 1<<1;   // enable SSP/SPI1. Should be last step of SSP setup

  
  //SSP1 Interrupt controller setup
  //disable ssp1 interrupts for DMA use
//  MIC_APR |= 1<<21;	//Set SSP1 interrupt to active high	(do this _FIRST_)
//  MIC_ITR |= 1<<21;	//Set SSP1 interrupt to FIQ
//  MIC_ER |= 1<<21;	//Enable SSP1 interrupt (then enable interrupts)


 /////////////////////////////////////////////////////////////////
  //DMA setup


DMACLK_CTRL = 1;	//Enable DMA clocks

GPDMA_CONFIG = 1;  //Enable DMA controller. Set bus master 0 and 1 to little-endian

//clear any leftover interrupts
GPDMA_INT_TCCLR = 0xFF;
GPDMA_INT_ERR_CLR = 0xFF;


SSP1CR1 |= 1<<1;   // enable SSP/SPI1. Should be last step of SSP setup

//Set up GPDMA interrupts
MIC_ITR = (MIC_ITR | 1<<28) & (~(1<<12 | 1<<2));	//Set GPDMA interrupt to FIQ
MIC_ATR = (MIC_ATR & ~(1<<28)) & (~(1<<12 | 1<<2));	//Set GPDMA interrupt to level-sense
MIC_APR = (MIC_APR | 1<<28) & (~(1<<12 | 1<<2));	//Set GPDMA interrupt to active high
MIC_ER  = (MIC_ER  | 1<<28) & (~(1<<12 | 1<<2));	//Enable GPDMA interrupt

mb_csr_gpdma_rx0_isr();	//Start DMA receive process





 
/////////////////////////////////////////////////////////////////
  //UART5 setup
  UART_CTRL = 0; 	// default pin configuration of U3, U5, and U6
  UART_CLKMODE |= 1 << 8;	//Clock on mode for UART 5. (Autoclock is bit 9)
  U5CLK = 0x1071;		// fractional rate pre-divider: X/Y = 16/113; use 13 MHz peripheral clock,
  						// to yield 115.2 kbaud.
  UART_LOOP &= ~(1<<4);		//Turn off loopback mode for UART 5

  U5FCR = 0x1;  // Enable FIFO
  U5FCR = 0x8F;	// Clear FIFOs, set FIFO control bit, generate interrupt when RX FIFO reaches 48 bytes.
  				// Generate interrupt when TX FIFO reaches 0 bytes.

  U5LCR=0x83;  //Step 1: DLAB=1, enable baud rate divisor adjustments
  			   //Set to 8 data bits, 1 stop bit, no parity

			   //Step 2: Set baud rate divisor
  U5DLL=1;		//Just using fractional rate pre-divider; no DLL, DLM needed.
  U5DLM=0;		//and peripheral clock = 60MHz.

  U5LCR &= ~(1<<7); // Step 3: DLAB=0, disable baud rate adjustments, continue with other setup
  U5IER = 0;		// No interrupts yet


 }

// Interrupt service routine for LPC3250 standard timer 0
void mb_timer0_isr(void)
{
  T0IR = 0xFF;	//Clear Timer0 interrupts
  
//  T1TC = 0; // **** TEST CODE **** reset timer 1
  mb_schedule_tick();
}
