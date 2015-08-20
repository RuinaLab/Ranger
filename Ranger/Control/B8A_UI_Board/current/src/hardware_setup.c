#include <includes.h>

void init_hardware(void){

  VICIntEnable = 0;

///////////////////////////////////////////////////////////////////
//Peripheral power control
//Turn on needed peripheral blocks here first
//PCONP - Enable peripheral power and clocks
  PCONP = 0
  | (1<<1)  //Enable Timer0
  | (1<<2)  //Enable Timer1
  | (1<<3)  //Enable UART0
  | (1<<4)  //Enable UART1
  | (1<<5)  //Enable PWM0
//| (1<<7)  //Enable I2C
  | (1<<8)  //Enable SPI0
//| (1<<9)  //Enable RTC (Real time clock)
//| (1<<10) //Enable SPI1
//| (1<<11) //Enable EMC (external memory controller) LPC21XX have no external memory
  | (1<<12) //Enable internal ADC (analog-to-digital converter) Note: clear PDN in ADCR before clearing this bit
#ifdef USE_CAN1
  | (1<<13) //Enable CAN controller 1
#endif
#ifdef USE_CAN2
| (1<<14) //Enable CAN controller 2
#endif
#ifdef USE_CAN3
| (1<<15) //Enable CAN controller 3
#endif
#ifdef USE_CAN4
| (1<<16) //Enable CAN controller 4
#endif
  | (1<<21) //Enable SSP0 controller (must disable SPI1 to use this) Bit 21!!!!!
  ;
/////////////////////////////////////////////////////////////////////////////////////


  SCS = (1<<0)|(1<<1);       // enable high-speed GPIO0 (bit 0) and GPIO1 (bit 1) (Fast GPIO)
  
  /*********************************************
  * Put user initialization code below here.  *
  *********************************************/   

  // ***********************************************
  // Heartbeat Init Section
  // ***********************************************
  //On-board LED initialization
  PINSEL2 &= ~(1<<3);   // set trace port for GPIO use (bit 3 = 0)
  FIO1DIR |= (1<<23);   // set P1.23 to be output (Green LED)
  FIO1DIR |= (1<<24);   // set P1.24 to be output (Red LED)
  FIO1DIR |= (1<<25);   // set P1.25 to be output (Blue LED)
  mcu_led_all_off(); 

  //Sync LED initialization
  PINSEL1 &= ~(3<<0);  // Set to fast GPIO
  FIO0DIR |= (1<<16);   // Set to output
  
  // *******************************************************************************
	// Initialize SSP/SPI1 for LED Control
	// *******************************************************************************
	PCONP &= ~(1<<10);		// power setting: disable SPI1	(bit 10)
	PCONP |= (1<<21);		// power setting: enable SSP (bit 21!!!!!!!!!!!!!!!!!!!!!!)
	SSPCR1 = 0;				// Disable SSP to allow setting changes
	SSPCR0 = 0;
	SSPCR0 |= (15<<0);		// data size: 16 bits (bits 0-3 = 15 means 16bit data)
	SSPCR0 &= ~(3<<4);		// SPI mode (bits 4-5 = 0)
	SSPCR0 &= ~(1<<6);		//sclk low when idle - Motorola format
	SSPCR0 &= ~(1<<7);		//sample on the first edge (rising edge)
//	SSPCPSR &= ~(3<<0);		//clear prescale divider
	SSPCPSR |= 2;		// prescale divider
	SSPCR0 |= (1<<8);		// bit frequency = PCLK/(CPSDVSR*(SCR+1)) = 15 Mbits/sec
	PINSEL1 &= ~0x3FC;		// clear P0.17~P0.20;
	PINSEL1 |= (2<<2)|(2<<4)|(2<<6);		// P0.17: SCK1, P0.18: MISO1, P0.19: MOSI1, P0.20: Manual SSEL 
//	PINSEL1 |= (2<<2)|(2<<4)|(2<<6)|(2<<8);	// P0.17: SCK1, P0.18: MISO1, P0.19: MOSI1, P0.20: auto SSEL
	FIO0DIR |= (1<<20); 	//SSEL is output
	FIO0SET = (1<<20); 		//SSEL high
	SSPCR1 |= (1<<1);		// enable SSP/SPI1	
  
	// *******************************************************************************
	// Initialize Buttons
	// *******************************************************************************
	PINSEL0 &= ~(3<<24); //Button 1: P0.12 as GPIO
	PINSEL0 &= ~(3<<26); //Button 2: P0.13 as GPIO
	PINSEL1 &= ~(3<<24); //Button 3: P0.28 as GPIO
	PINSEL1 &= ~(3<<28); //Button 5: P0.30 as GPIO
	PINSEL2 &= ~(1<<3);  //Buttons 0 & 4: P1.18 and P1.16 GPIO (P1.16-25 GPIO)
	//set pins as inputs
	FIO0DIR &= ~(1<<12); 
	FIO0DIR &= ~(1<<13);
	FIO0DIR &= ~(1<<28);
	FIO0DIR &= ~(1<<30);
	FIO1DIR &= ~(1<<16);
	FIO1DIR &= ~(1<<18);
  
  // *******************************************************************************
	// Initialize Microstrain IMU over UART1
	// *******************************************************************************
  U1LCR=0x83;  //Step 1: DLAB=1, enable baud rate adjustments
  //Set to 8 data bits, 1 stop bit, no parity
	//Step 2: Set baud rate divisor and fractional baud rate divisors
 // U1DLL=25;		//Baud rate divisor for 115200 kbaud, with (1+0.3) fractional divider 
 // U1DLM=0;		//and peripheral clock = 60MHz.
//  U1FDR = 3;		// DIVADDVAL = 3
//  U1FDR |= 10<<4;	// MULTVAL = 10

	//Step 2: Set baud rate divisor and fractional baud rate divisors
  U1DLL=6;		//Baud rate divisor for 460800 baud, with (1+0.356) fractional divider 
  U1DLM=0;		//and peripheral clock = 60MHz.
  U1FDR = 5;		// DIVADDVAL = 5
  U1FDR |= 14<<4;	// MULTVAL = 14

  //U1FCR = 0x81;    	// Enable receive and transmit FIFO; interrupt on receive at 8 bytes.
  U1FCR = 0x1;		// Enable FIFO
  U1FCR = 0x87;		//Clear FIFOs and set to generate interrupt when RX FIFO reaches 8 bytes.
  U1MCR = 0;	   	// Loopback mode, CTS and RTS flow control disabled.
  U1ACR = 0;	   	// Autobaud disabled
  U1TER = 0x80;	   	// Transmit enabled
  U1LCR &= ~(1<<7); // Step 3: DLAB=0, disable baud rate adjustments
  PINSEL0 |= 1<<16;    // UART1 TXD active on MCU pin P0[8]
  PINSEL0 &= ~(1<<17); //
  PINSEL0 |= 1<<18;    // UART1 RXD	active on MCU pin P0[9]
  PINSEL0 &= ~(1<<19); //
  U1IER = (1<<0); // enable RBR interrupts
  U1ACR = (1<<8) | (1<<9); //clear the corresponding interrupt in the U1IIR

	// *******************************************************************************
	// Buzzer PWM Setup
	// *******************************************************************************
	PINSEL1 &= ~(3<<10);
	PINSEL1 |= (1<<10);  //PWM 5 on Pin P0.21 Enabled (bits 10/11)
	PWMPR = 0; //Will run at maximum rate
	PWMMR0 = 100000; 
	PWMMR5 = 0;
	PWMMCR = (1<<1); //Should set Timer Counter to reset upon reaching Match Register 0
	PWMTCR = (1<<0)|(1<<3); //Enables Timer Counter and PWM Mode.
	PWMPCR = (1<<13); //enable PWM5
	PWMLER = (1<<0)|(1<<5); //update PWMMR values

	// *******************************************************************************
	// Initialize LCD
	// *******************************************************************************
	PINSEL0 &= ~(0xFFFF); //Pins P0.0-P0.7 GPIO (D0-D7 on LCD controller)
	PINSEL0 &= ~(((unsigned int)3)<<30); //P0.15 GPIO (RS)
	PINSEL0 &= ~(3<<20); //P0.10 GPIO (R/~W)
	PINSEL0 &= ~(3<<22); //P0.11 GPIO (EN)
	FIO0DIR |= 0x8CFF; //set above pins as output
	FIO0MASK &= ~(0xFF); //zero fiomask
	//Timer stuff
	T1PR = 0;//no prescaling
	T1MR0 = (60000000/(SCHED_SPEED * 1000)) - 1;//Match Register for scheduler interrupts
	T1TCR = 1;//enabled for counting
	T1CTCR = 0;//simple timer
	T1MCR = (1)|(1<<1);//interrupt (bit0) and reset (bit1) on MR0

  // *******************************************************************************
  // CAN1 Initialization                                             
  // *******************************************************************************    
  #ifdef USE_CAN1  
  // (No transceiver standby line for UI board CAN1)
  C1MOD = 1 & ~C1MOD_RB;
  C1CMR = (1<<3)|(1<<2)|(1<<1);
  C1GSR = 0 & ~C1GSR_RB;
  C1IER = 0 & ~C1IER_RB;
  //C1BTR = 0;              // default value for BTR
  C1BTR = (0<<23)|(2<<20)|(5<<16)|(1<<14)|(0<<10)|(1<<0)
  		& ~C1BTR_RB;
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

  // Enable CAN1 interrupts
  C1IER = 0
  | (1<<0)    //Enable receive interrupt
  | (0<<1)    //Disable transmit interrupt for buffer 1
  | (1<<2)    //Enable error warning interrupt
  | (1<<3)    //Enable data overrun interrupt
  | (0<<4)    //Disable wake-up interrupt
  | (1<<5)    //Enable error passive interrupt
  | (0<<6)    //Disable arbitration lost interrupt
  | (1<<7)    //Enable bus error interrupt
  | (0<<8)    //Disable ID ready interrupt
  | (0<<9)    //Disable transmit interrupt for buffer 2
  | (0<<10)   //Disable transmit interrupt for buffer 3
  ;

  C1GSR = 0;
  C1CMR = 0x0E;
  C1MOD = 0;
  #endif

  // *******************************************************************************
  // CAN2 Initialization                                             
  // ******************************************************************************* 
  FIO1DIR|=1<<19; //Set CAN standby control line to output
  FIO1SET= 1<<19; //Disable CAN tranceiver
  #ifdef USE_CAN2 
  PINSEL1 &= ~(3<<14);
  PINSEL1 |= 1<<14;  // enable CAN controller 2 pin RD2 
  PINSEL1 &= ~(3<<16);
  PINSEL1 |= 1<<16;  // enable CAN controller 2 pin TD2
  C2MOD = 1;
  C2CMR = (1<<3)|(1<<2)|(1<<1);
  C2GSR = 0;
  C2IER = 0;

  C2BTR = (0<<23)|(2<<20)|(5<<16)|(1<<14)|(0<<10)|(1<<0);
//  C2BTR_bit.BRP = 1;            // osc. freq. = 60MHz  assume VPB = 60MHz
//  C2BTR_bit.SJW = 1;            // Synchronization Jump Width
//  C2BTR_bit.TSEG1 = 5;          // Time Segment1 = 5+1 = 6
//  C2BTR_bit.TSEG2 = 2;          // Time Segment2 = 2+1 = 3
//  C2BTR_bit.SAM = 0;            // sample once, baud rate = 6MHz

  //default values
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

  //acceptance filter: bypass
  AFMR = 1<<1;

  //Enable CAN2 interrupts
  C2IER = 0
  | (1<<0)    //Enable receive interrupt
  | (0<<1)    //Disable transmit interrupt for buffer 1
  | (1<<2)    //Enable error warning interrupt
  | (1<<3)    //Enable data overrun interrupt
  | (0<<4)    //Disable wake-up interrupt
  | (1<<5)    //Enable error passive interrupt
  | (0<<6)    //Disable arbitration lost interrupt
  | (1<<7)    //Enable bus error interrupt
  | (0<<8)    //Disable ID ready interrupt
  | (0<<9)    //Disable transmit interrupt for buffer 2
  | (0<<10)   //Disable transmit interrupt for buffer 3
  ;

  C2GSR = 0;
  C2CMR = 0x0E;
  FIO1CLR= 1<<19; //Enable CAN tranceiver
  C2MOD = 0;
#endif

  // *******************************************************************************
  // RC Receive Setup
  // *******************************************************************************
  PINSEL1 &= ~(3<<12); //p0.22 clear
  PINSEL1 &= ~(3<<22); //p0.27 clear
  PINSEL1 &= ~(3<<0); //p0.16  clear 
  PINSEL1 &= ~(3<<26); //p0.29 clear 
  PINSEL1 |= (2<<12); //p0.22 cap0
  PINSEL1 |= (2<<22); //p0.27 cap1
  PINSEL1 |= (3<<0); //p0.16 cap2 
  PINSEL1 |= (2<<26); //p0.29 cap3 

  T0IR = 0xFF;
  T0CCR |= (1<<0)|(1<<1)|(1<<2); //cap0 interrupt on RE (rising edge) and FE (falling edge)
  T0CCR |= (1<<3)|(1<<4)|(1<<5); //cap1 interrupt on RE (rising edge) and FE (falling edge)
  T0CCR |= (1<<6)|(1<<7)|(1<<8); //cap2 interrupt on RE (rising edge) and FE (falling edge)
  T0CCR |= (1<<9)|(1<<10)|(1<<11); //cap3 interrupt on RE (rising edge) and FE (falling edge)

  T0MR0 = (60000000/100)-1; //10ms       
  T0MCR = (1<<0)|(1<<1); // Match Control Reg: Interrupt(b0) and Reset(b1) on MR0;
  T0TCR = 1;   // Timer0 Enable
}

