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
  asched_tick();
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
  /* Power control setup - turn on clocks for peripherals in use                                            */
  /*********************************************************************/ 
  
  //First check that SPI interrupts are disabled.
  S1SPCR = 0;
  
  PCONP =
  (1<<1)    //Timer0
  | (1<<2)  //Timer1
  | (1<<3)  //UART0
  | (1<<4)  //UART1
  | (1<<5)  //PWM0
  | (1<<12) //ADC internal
  | (1<<13) //CAN1
  | (1<<14) //CAN2
  | (1<<15) //CAN3
  | (1<<16) //CAN4
  | (1<<21) //SSP
  ;

  /*********************************************************************/
  /* Timer0 Initialization (except interrupts)                                            */
  /*********************************************************************/
  //initialize Timer0 for 1 ms ticks 
    
  T0TCR = 0;//enabled for counting
  T0PR = 0;//no prescaling
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
   
  SCS |= 1<<0;//Enable fast gpio on port 0.
  SCS |= 1<<1;//Enable fast gpio on port 1.
 
  // Set up MCU LED IO lines
  FIO1DIR = 1<<23|1<<24|1<<25;
  FIO1SET = 1<<23|1<<24|1<<25; 
  FIO1CLR = 1<<25;
  FIO1SET = 1<<23|1<<24;

  /********* End Heartbeat Init Section ************/
  
    /*********************************************************************/
  /* Timer1 Initialization                                            */
  /*********************************************************************/
  //initialize Timer1 for full-clock (60MHz for now) free-run counting
     
  T1TCR = 0;//counting disabled
  T1PR = 0;//no prescaling
  T1MR0 = (12000-1);//200 uS
  //(intended to allow complete read of receive buffer)
  T1CTCR = 0;//simple timer
  T1MCR = 0x00000003;//interrupt and reset on MR0 
//  T1TCR = 1;//enabled for counting 

  //////////////////////////////////////////////////////////////////////////////////  
	//SSP setup
//  PCONP = PCONP & (~(1<<10)) & PCONP_RBM;  // power setting: disable SPI1
//  PCONP = (PCONP | 1<<21) & PCONP_RBM;    	// power setting: enable SSP

  PINSEL1 &= ~0x3FC;  // clear P.017~P0.20;
  //Automatic control of slave select 1 (SSEL1)
  PINSEL1 |= 0x2A8;   // P0.17: SCK1, P0.18: MISO1, P0.19: MOSI1, P0.20: SSEL1
  
  //Manual control of slave select (SSEL) (i.e., via GPIO)
  //  PINSEL1 |= 0xA8;   // P0.17: SCK1, P0.18: MISO1, P0.19: MOSI1, P0.20: GPIO
  //  FIO0DIR |= 1<<20; //P0.20 is output
  //  FIO0SET = 1<<20;  //Set high
  
  SSPCR0 = 0xCF;     	// data size: 16 bits
  						// SPI mode
  						// CPOL=1: sclk high when idle
  						// CPHA=1: sample on the first edge (falling edge)
						  // Serial Clock Rate = 

  SSPCPSR = 20; 			// prescale divider - 3 Mbits/sec
  

  SSPCR1 = 0;        	// disable SSP/SPI1 
  SSPCR1 = 0;			//Master mode, no loopback mode
  SSPCR1 = ((SSPCR1 | (1<<1)) & ~(0xF<<4));   // enable SSP/SPI1

//  SSPIMSC = 0xF;		//Enable all ssp interrupt sources
//  SSPIMSC = 1<<3;		//enable transmit interrupts only

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

  FIO0SET = 1<<30; 	 	// turn off green CAN1 LED
  FIO1SET = 1<<16;	 		// turn off red CAN1 LED

  FIO0SET = 1<<2; 	 		// turn off green CAN2 LED
  FIO0SET = 1<<3;	 		// turn off red CAN2 LED

  FIO1SET = 1<<19; 	 	// turn off green CAN3 LED
  FIO1SET = 1<<18;	 		// turn off red CAN3 LED

  FIO0SET = 1<<10; 	 	// turn off green CAN4 LED
  FIO0SET = 1<<11;	 		// turn off red CAN4 LED

  FIO1SET = 1<<23; 	 	// turn off green MCU LED
  FIO1SET = 1<<24;	 		// turn off red MCU LED
  FIO1SET = 1<<25;	 		// turn off blue MCU LED

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
  
  
  
  // *******************************************************************************
  // CAN1 Initialization                                             
  // *******************************************************************************  
//  PCONP = (PCONP | 1<<13) & PCONP_RBM;  //Turn on CAN1 peripheral clock
 
  PINSEL1 &= ~(3<<18);
  PINSEL1 |= 1<<18;  // enable CAN controller 1 pin RD1 (TD1 not PINSELable)

  C1MOD = 1;
//  C1MOD |= (1<<2); //Self-test mode: Transmit always considered successful, even without ACK
  //  C1CMR = 1<<1; //Abort transmission?  Because maybe putting the controller into reset mode isn't enough?
  C1CMR = (1<<3)|(1<<2)|(1<<1);
  C1GSR = 0;
  C1IER = 0;
  //C1BTR = 0;              // default value for BTR
  C1BTR = (0<<23)|(2<<20)|(5<<16)|(1<<14)|(0<<10)|(1<<0);
  //  C2BTR_bit.BRP = 1;            // osc. freq. = 60MHz  assume VPB = 60MHz
  //  C2BTR_bit.SJW = 1;            // Synchronization Jump Width
  //  C2BTR_bit.TSEG1 = 5;          // Time Segment1 = 5+1 = 6
  //  C2BTR_bit.TSEG2 = 2;          // Time Segment2 = 2+1 = 3
  //  C2BTR_bit.SAM = 0;            // sample once, baud rate = 6MHz

  // default values
  C1TFI1 = 0;
  C1TFI2 = 0;
  C1TFI3 = 0;
  C1TID1 = 0;
  C1TID2 = 0;
  C1TID3 = 0;
  C1TDA1 = 0;
  C1TDA2 = 0;
  C1TDA2 = 0; 
  C1TDB1 = 0;
  C1TDB2 = 0;
  C1TDB3 = 0;

  // acceptance filter: bypass
  AFMR = 1<<1;
  
//  CAN1RS is hardwired to ground on B10A
//  if(B2A_OVERRIDE){
//  FIO1DIR|=1<<18;//Enable CAN tranceiver
//    FIO1CLR= 1<<18;
// }

  C1IER = 1<<0; //Turn on Rx Interrupt
//  C1IER |= ((1<<1) | (1<<9)) & 0x07FF; //Turn on Tx 1 and Tx 2 interrupts
  C1IER |=  ((1<<1)) & 0x07FF; //Turn on Tx 1 interrupts
  C1IER |= ((1<<2)|(1<<3)|(1<<5)|(1<<7))&0x7FF; //Turn on CAN error interrupts
//  C1IER |= (1<<2)|(1<<3)|(1<<4)|(1<<5)|(1<<6)|(1<<7); //Turn on CAN error interrupts
//  C1IER |= (1<<6); //Arbitration Lost interrupt
  C1GSR = 0;

  C1CMR = 0x0E;
  C1MOD = 0;
  // *******************************************************************************
  // CAN2 Initialization                                             
  // *******************************************************************************  
 // PCONP = (PCONP | 1<<14) & PCONP_RBM;  //Turn on CAN2 peripheral clock
  
  PINSEL1 &= ~(3<<14);
  PINSEL1 |= 1<<14;  // enable CAN controller 2 pin RD2 
  PINSEL1 &= ~(3<<16);
  PINSEL1 |= 1<<16;  // enable CAN controller 2 pin TD2

  C2MOD = 1;
//  C1MOD |= (1<<2); //Self-test mode: Transmit always considered successful, even without ACK
//  C1CMR = 1<<1; //Abort transmission?  Because maybe putting the controller into reset mode isn't enough?
  C2CMR = (1<<3)|(1<<2)|(1<<1);
  C2GSR = 0;
  C2IER = 0;
//  C1BTR = 0;              // default value for BTR
  C2BTR = (0<<23)|(2<<20)|(5<<16)|(1<<14)|(0<<10)|(1<<0);
//  C2BTR_bit.BRP = 1;            // osc. freq. = 60MHz  assume VPB = 60MHz
//  C2BTR_bit.SJW = 1;            // Synchronization Jump Width
//  C2BTR_bit.TSEG1 = 5;          // Time Segment1 = 5+1 = 6
//  C2BTR_bit.TSEG2 = 2;          // Time Segment2 = 2+1 = 3
//  C2BTR_bit.SAM = 0;            // sample once, baud rate = 6MHz

//  default values
  C2TFI1 = 0;
  C2TFI2 = 0;
  C2TFI3 = 0;
  C2TID1 = 0;
  C2TID2 = 0;
  C2TID3 = 0;
  C2TDA1 = 0;
  C2TDA2 = 0;
  C2TDA2 = 0; 
  C2TDB1 = 0;
  C2TDB2 = 0;
  C2TDB3 = 0;

//  C1IER = 1<<0; //Turn on Rx Interrupt
//  C1IER |= 1<<1; //Turn on Tx 1 interrupt
//  C1GSR = 0;//clear error counters

// acceptance filter: bypass
//  AFMR_bit.AccBP=1;
//  AFMR_bit.AccOff=0;
  AFMR = 1<<1;

//  CAN2RS is hardwired to ground on B10A
//  if(B2A_OVERRIDE){
//    FIO1DIR|=1<<19;//Enable CAN tranceiver
//    FIO1CLR= 1<<19;
//  }

  C2IER = 1<<0; //Turn on Rx Interrupt
//  C2IER |=  ((1<<1) | (1<<9)) & 0x07FF; //Turn on Tx 1 and Tx 2 interrupts
  C2IER |=  ((1<<1)) & 0x07FF; //Turn on Tx 1 interrupts
  C2IER |= ((1<<2)|(1<<3)|(1<<5)|(1<<7))&0x7FF; //Turn on CAN error interrupts
//  C1IER |= (1<<2)|(1<<3)|(1<<4)|(1<<5)|(1<<6)|(1<<7); //Turn on CAN error interrupts
//  C1IER |= (1<<6); //Arbitration Lost interrupt
  C2GSR = 0;

  C2CMR = 0x0E;
  C2MOD = 0;


  // *******************************************************************************
  // CAN3 Initialization                                             
  // *******************************************************************************  
 // PCONP = (PCONP | 1<<15) & PCONP_RBM;  //Turn on CAN3 peripheral clock
  
  PINSEL1&=~(3<<10);
  PINSEL1 |= 2<<10;     // enable CAN controller 3 pin RD3
  PINSEL1&=~(3<<12);
  PINSEL1 |= 1<<12;     // enable CAN controller 3 pin TD3
  
  C3MOD = 1;
//  C1MOD |= (1<<2); //Self-test mode: Transmit always considered successful, even without ACK
  //  C1CMR = 1<<1; //Abort transmission?  Because maybe putting the controller into reset mode isn't enough?
  C3CMR = (1<<3)|(1<<2)|(1<<1);
  C3GSR = 0;
  C3IER = 0;
  //C1BTR = 0;              // default value for BTR
  C3BTR = (0<<23)|(2<<20)|(5<<16)|(1<<14)|(0<<10)|(1<<0);
  //  C2BTR_bit.BRP = 1;            // osc. freq. = 60MHz  assume VPB = 60MHz
  //  C2BTR_bit.SJW = 1;            // Synchronization Jump Width
  //  C2BTR_bit.TSEG1 = 5;          // Time Segment1 = 5+1 = 6
  //  C2BTR_bit.TSEG2 = 2;          // Time Segment2 = 2+1 = 3
  //  C2BTR_bit.SAM = 0;            // sample once, baud rate = 6MHz

  // default values
  C3TFI1 = 0;
  C3TFI2 = 0;
  C3TFI3 = 0;
  C3TID1 = 0;
  C3TID2 = 0;
  C3TID3 = 0;
  C3TDA1 = 0;
  C3TDA2 = 0;
  C3TDA2 = 0; 
  C3TDB1 = 0;
  C3TDB2 = 0;
  C3TDB3 = 0;

  // acceptance filter: bypass
  AFMR = 1<<1;
  FIO0DIR|=1<<8;//Enable CAN tranceiver
  FIO0CLR= 1<<8;

  C3IER = 1<<0; //Turn on Rx Interrupt
//  C3IER |=  ((1<<1) | (1<<9)) & 0x07FF; //Turn on Tx 1 and Tx 2 interrupts
  C3IER |=  ((1<<1)) & 0x07FF; //Turn on Tx 1 interrupts
  C3IER |= ((1<<2)|(1<<3)|(1<<5)|(1<<7))&0x7FF; //Turn on CAN error interrupts
//  C1IER |= (1<<2)|(1<<3)|(1<<4)|(1<<5)|(1<<6)|(1<<7); //Turn on CAN error interrupts
//  C1IER |= (1<<6); //Arbitration Lost interrupt
  C3GSR = 0;

  C3CMR = 0x0E;
  C3MOD = 0;
  
  
  // *******************************************************************************
  // CAN4 Initialization                                             
  // *******************************************************************************  
//  PCONP = (PCONP | 1<<16) & PCONP_RBM;  //Turn on CAN4 peripheral clock

  PINSEL0&=~(3<<24);
  PINSEL0 |= 3<<24;     // enable CAN controller 4 pin RD4
  PINSEL0&=~(3<<26);
  PINSEL0 |= 3<<26;     // enable CAN controller 4 pin TD4

  C4MOD = 1;
//  C1MOD |= (1<<2); //Self-test mode: Transmit always considered successful, even without ACK
//  C1CMR = 1<<1; //Abort transmission?  Because maybe putting the controller into reset mode isn't enough?
  C4CMR = (1<<3)|(1<<2)|(1<<1);
  C4GSR = 0;    //set error counters to zero
  C4IER = 0;
//  C1BTR = 0;              // default value for BTR
  C4BTR = (0<<23)|(2<<20)|(5<<16)|(1<<14)|(0<<10)|(1<<0);
//  C2BTR_bit.BRP = 1;            // osc. freq. = 60MHz  assume VPB = 60MHz
//  C2BTR_bit.SJW = 1;            // Synchronization Jump Width
//  C2BTR_bit.TSEG1 = 5;          // Time Segment1 = 5+1 = 6
//  C2BTR_bit.TSEG2 = 2;          // Time Segment2 = 2+1 = 3
//  C2BTR_bit.SAM = 0;            // sample once, baud rate = 6MHz

//  default values
  C4TFI1 = 0;
  C4TFI2 = 0;
  C4TFI3 = 0;
  C4TID1 = 0;
  C4TID2 = 0;
  C4TID3 = 0;
  C4TDA1 = 0;
  C4TDA2 = 0;
  C4TDA2 = 0; 
  C4TDB1 = 0;
  C4TDB2 = 0;
  C4TDB3 = 0;

// acceptance filter: bypass
//  AFMR_bit.AccBP=1;
//  AFMR_bit.AccOff=0;
  AFMR = 1<<1;


  FIO0DIR|=1<<9;//Enable CAN tranceiver
  FIO0CLR= 1<<9;  

  C4IER = 1<<0; //Turn on Rx Interrupt
//  C4IER |=  ((1<<1) | (1<<9)) & 0x07FF; //Turn on Tx 1 and Tx 2 interrupts
  C4IER |=  ((1<<1)) & 0x07FF; //Turn on Tx 1 interrupts
  C4IER |= ((1<<2)|(1<<3)|(1<<5)|(1<<7))&0x7FF; //Turn on CAN error interrupts
//  C1IER |= (1<<2)|(1<<3)|(1<<4)|(1<<5)|(1<<6)|(1<<7); //Turn on CAN error interrupts
//  C1IER |= (1<<6); //Arbitration Lost interrupt
  C4GSR = 0;

  C4CMR = 0x0E; //Abort transmission, release receive buffer, clear data overrun.
  C4MOD = 0;     //Put CAN controller in normal operating mode




 }
