#include <includes.h>

void init_hardware(void){

  VICIntEnable = 0;

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
  FIO1SET = (1<<24)|(1<<23)|(1<<25);

  // *******************************************************************************
  // Initialize SSP/SPI1 For External ADC
  // *******************************************************************************
  PCONP &= ~(1<<10);  	// power setting: disable SPI1  (bit 10)
  PCONP |= (1<<21);  	// power setting: enable SSP (bit 21!!!!!!!!!!!!!!!!!!!!!!)
  SSPCR1 = 0;  	  	// Disable SSP to allow setting changes
  SSPCR0 = 0x00000000;
  SSPCR0 = (15<<0)  	// data size: 16 bits (bits 0-3 = 15 means 16bit data)
  | (1<<6)			// sclk high when idle
	| (1<<7)		  	// sample on the second edge (rising edge)
	| (2<<8);		  	// bit frequency = PCLK/(CPSDVSR*(SCR+1)) =
//  SSPCR0 &= ~(3<<4);  	// SPI mode (bits 4-5 = 0)
//  SSPCPSR &= ~(3<<0);  	//clear prescale divider
  SSPCPSR = (2<<0);  	// prescale divider  
  PINSEL1 &= ~0x3FC;  	// clear P0.17~P0.20;
//  PINSEL1 |= (2<<2)|(2<<4)|(2<<6);  	// P0.17: SCK1, P0.18: MISO1, P0.19: MOSI1, P0.20: Manual SSEL 
  PINSEL1 |= (2<<2)|(2<<4)|(2<<6)|(2<<8);  // P0.17: SCK1, P0.18: MISO1, P0.19: MOSI1, P0.20: auto SSEL
//  FIO0DIR |= (1<<20); //SSEL is output
//  FIO0CLR = (1<<20); //SSEL low
  SSPCR1 = (1<<1);  	// enable SSP/SPI1   
  PINSEL0 &= ~(3<<24);  //ADC reset line set for GPIO operation (P0.12 bits 24/25)
  PINSEL2 &= ~(1<<3);    // set trace port for GPIO use (bit 3 = 0)
  //Should happen automatically at startup, but just in case...
  //GPIO P1_16 is the ADC convert line
  FIO0DIR |= (1<<12);    // set P0_12 (ADC reset) to be a digital output
  FIO1DIR |= (1<<16);    // set P1_16 (ADC convert) to be a digital output  
  FIO0SET = (1<<12);    //set ADC reset line high
  FIO1CLR = (1<<16);    //set ADC convert line low
  // adc_external interrupts
  PINSEL0 &=~(3<<6);
  PINSEL0 |= (3<<6);  	  	// Set Pin3 to EINT1 (ADCBUSY line)
  EXTMODE = (EXTMODE|(1<<1)) & ~EXTMODE_RB;  	  	  // Set EINT1 (Bit 1) to be edge-sensitive
  EXTPOLAR &= ~(1<<1)&0x0F;  	  	// Set EINT1 to be sensitive on the falling-edge
  EXTINT = 0x0F;//1;

  // *******************************************************************************
  // Motor Controller Setup
  // *******************************************************************************
  //Motor control initialization
  //FIO0DIR |= (1<<9);   	// set P0.9 to be output (PWM A)
  //FIO0DIR |= (1<<7);     // set P0.7 to be output (PWM B)
  FIO0DIR |= (1<<11);     // set P0.11 to be output (Low side enable A)
  FIO1DIR |= (1<<17);     // set P1.17 to be output (Low side enable B)
  FIO0DIR |= (1<<6);      // set P0.6 to be output (Watchdog timer) 
  //FIO0CLR = (1<<9);  	// turn off PWM A output
  //FIO0CLR = (1<<7);  	// turn off PWM B output
  FIO0SET = (1<<11);  	// turn on low side A enable output
  FIO1SET = (1<<17);  	// turn on low side B enable output
  FIO0CLR = (1<<6);  	// turn off watchdog timer output
  //Initializes the PWM registers and PWM2 and PWM6 outputs
  //Pin Function Select
  PINSEL0 &= ~(3<<14);
  PINSEL0 |= (2<<14);  //PWM 2 on Pin P0.7 Enabled (bits 14/15)
  PINSEL0 &= ~(3<<18);
  PINSEL0 |= (2<<18);  //PWM 6 on Pin P0.9 Enabled (bits 18/19)
  //PWM Prescale Register. Controls number of PClK cycles per Timer Counter Cycle
  PWMPR= 0; //Will run at maximum rate
  //PWM Match Registers
  //PWM Match Register 0. Resets Timer Counter and sets all PWM outputs to 1.
  PWMMR0= 600; //100 kHz PWM frequency (60000000/100000)
  PWMMR1= 0;
  PWMMR2= 0;
  PWMMR3= 0;
  PWMMR4= 0;
  PWMMR5= 0;
  PWMMR6= 0;
  //PWM Match Control Register
  PWMMCR= (PWMMCR|0x2) & ~PWMMCR_RB; //Should set Timer Counter to reset upon reaching Match Register 0
  //PWM Timer Control Register.
  PWMTCR= (PWMTCR|0x1) // Enable Timer Counter
  		| (PWMTCR|0x8) // Enable PWM Mode
  		& ~PWMTCR_RB; 
               //Must Occur after Initialization of Match Register 0.
  //PWM Control Register. Enables individual PWM types and outputs.
  PWMPCR = (PWMPCR|0x4400) & ~PWMPCR_RB; //Should mean the same thing as next 4 lines (PWM Control Register)
  //PWMSEL2= 0; //Enables Single Edge PWM 2 Control
  //PWMSEL6= 0; //Enables Single Edge PWM 6 Control
  //PWMENA2= 1; //PWM 2 Output Enabled
  //PWMENA6= 1; //PWM 6 Output Enabled
  //PWM Latch Enable Register. Updates Match values.
  //Each bit directly relates to a Match Register
  //PWMLER= 0;  //Latch Register to zero
  PWMMR2= 0;       // out of 600  -ve  	
  PWMMR6= 0;       // out of 600  +ve  	  	 
  // Latch enable reg automatically cleared once PWM match values are set
  PWMLER= (PWMLER|0x45) & ~PWMLER_RB;  //Enable update of PWM match registers 0, 2 and 6.  0x45 --> bit0, bit2, bit6

  // *******************************************************************************
  // Initialize Internal ADC 
  // *******************************************************************************
  //Burst mode setup (old)
  //ADCR = 0x0021C700|(ADCI_CH0_READ)|(ADCI_CH1_READ << 1)|(ADCI_CH2_READ << 2)|(ADCI_CH3_READ << 3);
  //Interrupt driven setup - CLKDIV = 199; bit21 is ADC enabled; 
  //Which channels to read from; if you want to read from a channel, set its value to 1
  #define ADCI_CH0_READ /*hello?*/
  #define ADCI_CH1_READ /*fix*/
  #define ADCI_CH2_READ /*me!*/
  #define ADCI_CH3_READ /*(please)*/
  ADCR = (1<<21)|(199<<8) & ~ADCR_RB;
  //Set Pins to be AINx if reading from that channel
  if (ADCI_CH0_READ) {
    PINSEL1 &= ~(3<<22);
    PINSEL1 |= (1<<22);
  }  //AIN0 on pin P0.27
  if (ADCI_CH1_READ) {
    PINSEL1 &= ~(3<<24);
    PINSEL1 |= (1<<24);
  }  //AIN1 on pin P0.28 
  if (ADCI_CH2_READ) {
    PINSEL1 &= ~(3<<26);
    PINSEL1 |= (1<<26);
  }  //AIN2 on pin P0.29
  if (ADCI_CH3_READ) {
    PINSEL1 &= ~(3<<28);
    PINSEL1 |= (1<<28);
  }  //AIN3 on pin P0.30  
  //adci interrupts for conversion completions
  ADINTEN = (1<<8)//(ADCI_CH0_READ)|(ADCI_CH1_READ << 1)|(ADCI_CH2_READ << 2)|(ADCI_CH3_READ << 3)
  		& ~ADINTEN_RB;  

/*  // *******************************************************************************
  // UART Init Section
  // *******************************************************************************
  //Set P0.0 to TXD0 and P0.1 to RXD0 
  PINSEL0 &= ~(0xF);
  PINSEL0 |= 0x5;  
  U0LCR = (1<<7);//DLAB = 1 to enable baud rate changes
  //Rate config from Jason for 115.2k, with 1+0.3 frac divider and pclk = 60MHz
  U0DLL = 25;
  U0DLM = 0;
  U0FDR = 3;//DIVADDVAL = 3
  U0FDR |= 10<<4;//MULTVAL = 10;

  U0LCR = (0<<7)|(3<<0);//DLAB = 0 to disable baud rate changes, Wordsize = 3 for 8 bit words  
  U0IER = (1<<1);//1: Enable THRE interrupt  
  IO0DIR |= 1<<10; //Unassert SHDN_L on uart level shifter
  IO0SET = 1<<10;*/

  // *******************************************************************************
  // CAN1 Initialization                                             
  // *******************************************************************************  
  PINSEL1 &= ~(3<<18);
  PINSEL1 |= 1<<18;  // enable CAN controller 1 pin RD1 (TD1 not PINSELable)
//  FIO0DIR |= 1<<25;  //Set 0.25 CAN RD1 To output...wait, why?
  C1MOD = 1 & ~C1MOD_RB;
//  C1MOD |= (1<<2); //Self-test mode: Transmit always considered successful, even without ACK
  //  C1CMR = 1<<1; //Abort transmission?  Because maybe putting the controller into reset mode isn't enough?
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
  FIO1DIR|=1<<18;//Enable CAN tranceiver
  FIO1CLR= 1<<18;

  C1IER = 1<<0; //Turn on Rx Interrupt
  C1IER |= (1<<1)&0x07FF; //Turn on Tx 1 interrupt
  C1IER |= ((1<<2)|(1<<7))&0x7FF //Turn on CAN error interrupts
  		& ~C1IER_RB;
//  C1IER |= (1<<2)|(1<<3)|(1<<4)|(1<<5)|(1<<6)|(1<<7); //Turn on CAN error interrupts
//  C1IER |= (1<<6); //Arbitration Lost interrupt
  C1GSR = 0;

  C1CMR = 0x0E;
  C1MOD = 0;
  
  // *******************************************************************************
  // CAN2 Initialization                                             
  // *******************************************************************************  
  PINSEL1 &= ~(3<<14);
  PINSEL1 |= 1<<14;  // enable CAN controller 2 pin RD2 
  PINSEL1 &= ~(3<<16);
  PINSEL1 |= 1<<16;  // enable CAN controller 2 pin TD2
//  FIO0DIR |= 1<<25;  //Set 0.25 CAN RD1 To output...wait, why?
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
//  IO1DIR|=1<<18;//Enable CAN tranceiver
//  IO1CLR= 1<<18;
  FIO1DIR|=1<<19;//Enable CAN tranceiver
  FIO1CLR= 1<<19;

  C2IER = 1<<0; //Turn on Rx Interrupt
  C2IER |= (1<<1)&0x07FF; //Turn on Tx 1 interrupt
  C2IER |= ((1<<2)|(1<<7))&0x7FF; //Turn on CAN error interrupts
//  C1IER |= (1<<2)|(1<<3)|(1<<4)|(1<<5)|(1<<6)|(1<<7); //Turn on CAN error interrupts
//  C1IER |= (1<<6); //Arbitration Lost interrupt
  C2GSR = 0;

  C2CMR = 0x0E;
  C2MOD = 0;
  
  // *******************************************************************************
	// SPI0 Setup for Absolute Encoder 
	// *******************************************************************************
	//Set Pinsel bits for controlling SPI0:
  PINSEL0 &= ~(3<<8); //clear pin P0.4
  PINSEL0 &= ~(3<<10); //clear pin P0.5 
  PINSEL0 &= ~(3<<26); //clear pin P0.5
	PINSEL0 |= (1<<8);		//SCKO for SPI0 (clock for data transfer)
	PINSEL0 |= (1<<10);	    //MISO for SPI0 (master in slave out data)
	PINSEL0 |= (0<<26);		//set P0_13 to GPIO -- set pin to output
	FIO0DIR |= (1<<13);		//this pin controls multiplexers (SN74LVC1G3157) via the SN74LVC1G07 level shitfer
	    	                   		//when P0_13 is low J3 encoder can be read via SPI
									//when   ''  is high J9 encoder can be read 
	//Set Up SPI Register:
	S0SPCR |= (1<<2) | (0<<8) | (1<<9) | (1<<10) | (1<<11); 	//recieve 14 bits of data
	S0SPCR |= (1<<4);				//set clock polarity to active low
	S0SPCR |= (1<<5);				//activate master mode
	S0SPCCR = 60;	//set data transfer rate; PCLK / S0SPCCR = 60MHz / 60 = 1MHz

  // *******************************************************************************
  // QEC Setup
  // *******************************************************************************
  //Enable Capture Interrupt For Falling and Rising Edges:
  PINSEL1 &= ~(3<<12);
  PINSEL1 &= ~(3<<0); //Clear bits 1:0 (P0.16)
  PINSEL1 &= ~(3<<26); //Clear bits 27:26 (P0.29)
  PINSEL1 &= ~(3<<22);
  PINSEL1 |= (2<<12);    // set pin 0.22 to capture 0.0  
  PINSEL1 |= (2<<22);    // set pin 0.27 to capture 0.1
  PINSEL1 |= (3<<0); //Set P0.16 to Timer0 Capture 0.2
  PINSEL1 |= (2<<26); //Set P0.29 to Timer0 Capture 0.3
  //Set up the Timer Counter 0 Interrupt
  //Used to recod length of time between encoder pulses.
  T0IR  = 0xFF;                           //clear interrupts in Timer0
  //Enable Capture registers
  //Encoder 1 - J3
  T0CCR |= (1<<0)|(1<<1)|(1<<2); //Cap0.0 on rising and falling edges
  T0CCR |= (1<<3)|(1<<4)|(1<<5); //Cap0.1 on rising and falling edges
  //Encoder 2 - J9
  T0CCR |= (1<<6)|(1<<7)|(1<<8);//|(1<<9)|(1<<10)|(1<<11); //Cap0.2 on rising and falling edges
  T0CCR |= (1<<9)|(1<<10)|(1<<11); //Cap0.3 on rising and falling edges
  T0TCR = 1;                           // Timer0 Enable  

  // *******************************************************************************
  // Limit Switch Setup                                            
  // *******************************************************************************  
  PINSEL0 &= ~(3<<0); //set P0.0 to GPIO
  PINSEL0 &= ~(3<<2); //set P0.1 to GPIO
  FIO0DIR &= ~(1<<0); //set P0.0 to input
  FIO0DIR &= ~(1<<1); //set P0.1 to input


  // *******************************************************************************
  // SCHEDULE Setup
  // *******************************************************************************
  //NOTE: Scheduler uses same timer as QEC... it just uses the overflow capabilities
  T0MR0 = 60000000/(SCHED_SPEED * 1000);       // Match Register 0
  T0MCR = 3;                           // Match Control Reg: Interrupt(b0) and Reset(b1) on MR0
  
  
/* ________________________________________________________
  //initialize Timer1 for 1 ms ticks for SCHEDULER
  //edited by Nic to use timer 1; previously used timer 0 but conflicted with QEC module
  T1PR = 0;//no prescaling
  T1MR0 = 60000000/(SCHED_SPEED*1000);//Match Register for scheduler interrupts
  T1TCR = 1;//enabled for counting
  T1CTCR = 0;//simple timer
  T1MCR = (1)|(1<<1);//interrupt (bit0) and reset (bit1) on MR0*/
}
