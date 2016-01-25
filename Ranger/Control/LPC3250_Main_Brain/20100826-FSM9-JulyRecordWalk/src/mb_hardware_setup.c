#include <mb_includes.h>

void mb_setup_hardware(void){

//unsigned long int temp = 0;

 /////////////////////////////////////////////////////////////////
 //Standard Timer 0 Config, 1.5 ms ticks (slower than 1 mS sync from ARM7)
 //Will be adjusted to 1 mS if timer runs in local mode without
 //synchronization from the ARM7 SSP SSEL line.
 //Adjustment occurs in Timer0 isr.
 //Adjusted to 1.5 mS again when sync pulses resume.
 //This readjustment occurs in a9_clock_tick, called from SSP SSEL isr

  TIMCLK_CTRL1 |= 1<<2;	//Enable timer 0 clock (13 MHz peripheral clock)
  T0PR = 0;				//Divides input clock by PR (prescale register) + 1;
  T0MR0 = 13000 - 1;	//Timer counter will reset and interrupt at MR (match register) + 1 intervals

  T0CTCR = 	0;		// Timer mode - every PCLK rising edge increments prescale counter

  T0MCR = 3;		//Interrupt and reset on match register 0
  T0CCR = 0;		//Disable capture registers and their interrupts
  T0TCR = 1<<1;		//Reset counter 
/////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////
//Standard Timer 1 Config, free-run 

  TIMCLK_CTRL1 |= 1<<3;	//Enable timer 1 clock (13 MHz peripheral clock)
  T1PR = 0;				//Divides input clock by PR (prescale register) + 1;

  T1CTCR = 	0;		// Timer mode - every PCLK rising edge increments prescale counter

 
  T1CCR = 0;		//Disable capture registers and their interrupts
  T1TCR = 1<<1;		//Reset counter
//  T1TCR = 1<<0;		//Bring reset bit low, start counter
  T1TCR = 0;  // **** TEST CODE ****

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
  P_MUX_SET = 1<<8;		// Set SPI2_CLK/SCK1 pin to SCK1
  P_MUX_SET = 1<<6;		// Set SPI2_DATIN/MISO1 pin to MISO1
  P_MUX_SET = 1<<5;		// Set SPI2_DATIO/MOSI1 pin to MOSI1
  P2_MUX_SET = 1<<4;	// Set GPIO_04 to SSEL1
//  P2_MUX_SET = 0;	// Set GPIO_04 to GPIO (note: not a robust bit setting)
  P2_DIR_CLR = 1<<29; //Set GPIO_4 to input

  //Note - may be overridden by LCD, if enabled.

  SSP_CTRL = 1<<1;  // enable ssp1 clock
  SSP_CTRL |= 1<<4;	// enable ssp1 tx dma
  SSP_CTRL |= 1<<5; // enable ssp1 rx dma
  
  SSP1CR0 = 0xCF;    // data size: 16 bits
  					 // SPI mode
  		             // CPOL = 1 (sclk high when idle)
					 // CPHA = 0 (sample on the falling edge)
  					 // Serial clock rate divider = 0 (divide by 1)

  SSP1CPSR = 12;  // Prescale divider. The LPC3250 user manual says to set this to 12 (minimum)
                  // in slave mode, so that's what we did. It works at master clock rates from the slowest
                  // to about 15 MHz, in simple tests. Or was that 20 MHz? It definitely did not work at 30
                  // MHz. We are generally using lower speeds due to limitations on the ARM7 end.
                  // But the important point is that this can be left at 12 for pretty much any data rate.

  SSP1CR1 = 0;		//Clear all bits. Enable bit must be zero
  SSP1CR1 = 1<<2;	//Set slave mode, no loopback mode or slave output disable.	
 
  SSP1IMSC = 0;	//for DMA, disable all ssp1 interrupts

 //SSP1DMACR = 3;	 //Enable transmit and receive FIFO for DMA //moved to software setup,
  	
  SSP1CR1 = (unsigned char)((SSP1CR1 | (1<<1)) & (0xF));   // enable SSP/SPI1
  
  
 /////////////////////////////////////////////////////////////////
 //DMA setup

DMACLK_CTRL = 1;	//Enable DMA clocks

GPDMA_CONFIG = 1;  //Enable DMA controller. Set bus master 0 and 1 to little-endian

//clear any leftover interrupts
GPDMA_INT_TCCLR = 0xFF;
GPDMA_INT_ERR_CLR = 0xFF;

#ifdef BT_HSU2_BLUETOOTH
/////////////////////////////////////////////////////////////////
  UART_CTRL = 0;    //Should be the default. Enable HSU2 functions, instead of U3 modem control lines.
                    //Also affects U6.
  
  //HS-UART2 setup for BlueTooth serial communication to the Main Brain
  HSU2_CTRL =   
      (3<<0)  //transmitter FIFO trigger level of 16 bytes
    | (0<<2)  //receiver FIFO trigger level of 1 bytes (for occasional incoming packets - using a trigger level
              //of one byte prevents data from sitting around in the buffer forever.
    | (1<<5)  //enable transmit interrupt (for DMA use, not by the interrupt controller)
    | (1<<6) //enable receiver interrupt (for DMA use, not by the interrupt controller)
    | (20<<9)  //Sampling point offset (default is 20)
    | (1<<14) //Enable CTS flow control
    | (1<<15) //HCTS is inverted
    | (0<<16) //Disable receiver timeout interrupt for DMA
    | (1<<18) //Enable RTS flow control; pin is connected to U2_HRTS
    | (2<<19) //Set RTS flow control trigger level to 32
    | (1<<21) //HRTS is inverted
    ;
            
  HSU2_RATE = 1;    //PERIPH_CLK/14/(1+1) = 13,000,000/14/2 = 464.29 Kbaud
//  HSU2_RATE = 96;    //PERIPH_CLK/14/(96+1) = 13,000,000/14/97 = 9600 baud
              
  HSU2_IIR = (1<<5) | (1<<4) | (1<<3);  //Clear overrun, break, and framing error interrupts 
/////////////////////////////////////////////////////////////////
#endif  //BT_HSU2_BLUETOOTH

#ifdef BT_HSU1_SERIAL_PORT
/////////////////////////////////////////////////////////////////
  //HS-UART1 setup (for non-BlueTooth serial communication to the Main Brain)
  HSU1_CTRL =   
      (3<<0)  //transmitter FIFO trigger level of 16 bytes
    | (0<<2)  //receiver FIFO trigger level of 1 bytes (for occasional incoming packets - using a trigger level
              //of one byte prevents data from sitting around in the buffer forever.
    | (1<<5)  //enable transmit interrupt (for DMA use, not by the interrupt controller)
    | (1<<6) //enable receiver interrupt (for DMA use, not by the interrupt controller)
              //Don't enable the receive time-out interrupt with DMA
    | (20<<9);  //Sampling point offset (default is 20)

//  HSU1_RATE = 96;    //PERIPH_CLK/14/(96+1) = 13,000,000/14/97 = 9600 baud            
//  HSU1_RATE = 7;    //PERIPH_CLK/14/(7+1) = 13,000,000/14/8 = 116.07 Kbaud
//  HSU1_RATE = 0;    //PERIPH_CLK/14/(0+1) = 13,000,000/14/1 = 928.6 Kbaud
  HSU1_RATE = 1;    //PERIPH_CLK/14/(1+1) = 13,000,000/14/2 = 464.3 Kbaud
            
  HSU1_IIR = (1<<5) | (1<<4) | (1<<3);  //Clear overrun, break, and framing error interrupts 
  
  HSU1_TX = 0;    // **** TEST CODE **** start data transfer?
/////////////////////////////////////////////////////////////////
#endif  //BT_HSU1_SERIAL_PORT
 
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
