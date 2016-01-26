#include <includes.h>

void init_hardware(void){

  VICVectAddr = 0;
  VICVectAddr = 0;
  VICVectAddr = 0;
  VICVectAddr = 0;
  VICVectAddr = 0;
  VICVectAddr = 0;
  VICIntEnable = 0;

///////////////////////////////////////////////////////////////////
//Peripheral power control
//Turn on needed peripheral blocks here first
//PCONP - Enable peripheral power and clocks
  PCONP = 0
  | (1<<1)  //Enable Timer0
  | (1<<2)  //Enable Timer1
  | (1<<3)  //Enable UART0
//  | (1<<4)  //Enable UART1
//  | (1<<5)  //Enable PWM0
  | (1<<7)  //Enable I2C
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
//  | (1<<21) //Enable SSP0 controller (must disable SPI1 to use this) Bit 21!!!!!
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
  FIO1SET = (1<<24)|(1<<23)|(1<<25);


  // *******************************************************************************
  // Initialize Internal ADC 
  // *******************************************************************************
  //Burst mode setup (old)
  //ADCR = 0x0021C700|(ADCI_CH0_READ)|(ADCI_CH1_READ << 1)|(ADCI_CH2_READ << 2)|(ADCI_CH3_READ << 3);
  //Interrupt driven setup - CLKDIV = 199; bit21 is ADC enabled; 
  //Which channels to read from; if you want to read from a channel, set its value to 1
  #define ADCI_CH0_READ 0
  #define ADCI_CH1_READ 1
  #define ADCI_CH2_READ 0
  #define ADCI_CH3_READ 0
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

  // *******************************************************************************
  // I2C Initialize 
  //******************************************************************************* 
  PINSEL0 &=~(3<<4);
	PINSEL0 &=~(3<<6);
	PINSEL0 |=1<<4;
	PINSEL0 |=1<<6;

	//set up appropriate data rate
	//I2SCLH defines the number of pclk cycles for SCL high, I2SCLL defines the number of pclk cycles for SCL low.
	//see p 175 for more details. Color sensor max is 100KHz
	I2SCLH = 300;
	I2SCLL = 300;
	
	//turn i2onset
	//make i2conset look like MSB: - 1 0 0 0 0 - - :LSB
	I2CONCLR = 0x6C; // sets AA to 0, SI to 0 STA to 0, I2EN to 0
	I2CONSET = 0x40; //sets i2onset to -10000-- 

  // *******************************************************************************
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
  IO0SET = 1<<10;

  // *******************************************************************************
  // CAN1 Initialization                                             
  // *******************************************************************************
  FIO1DIR|=1<<18; //Set transceiver control line to output
  FIO1SET= 1<<18; //Disable transceiver
  
  #ifdef USE_CAN1  
  PINSEL1 &= ~(3<<18);
  PINSEL1 |= 1<<18;  // enable CAN controller 1 pin RD1 (TD1 not PINSELable)
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
  FIO1CLR = 1<<18; //Enable transceiver
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

// acceptance filter: bypass
  AFMR = 1<<1;

  // Enable CAN2 interrupts
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
  // QDC Setup
  // *******************************************************************************
  //Enable Capture Interrupt For Falling and Rising Edges:
  PINSEL1 &= ~(3<<12);
  PINSEL1 &= ~(3<<0);     //Clear bits 1:0 (P0.16)
  PINSEL1 &= ~(3<<26);    //Clear bits 27:26 (P0.29)
  PINSEL1 &= ~(3<<22);
  PINSEL1 |= (2<<12);     // set pin 0.22 to capture 0.0  
  PINSEL1 |= (2<<22);     // set pin 0.27 to capture 0.1
  PINSEL1 |= (3<<0);      //Set P0.16 to Timer0 Capture 0.2
  PINSEL1 |= (2<<26);     //Set P0.29 to Timer0 Capture 0.3
  //Set up the Timer Counter 0 Interrupt
  //Used to record length of time between encoder pulses.
  T0IR  = 0xFF;                           //clear interrupts in Timer0

  //Cycle time for count and reset = (MR0 + 1)/Clock freq
  //= 30000/60000000 = 0.0005 sec. = 0.5 mS
  //T0MR0 = 29999;       
  T0MR0 = 3243243; //i2c modification to avoid 120 Hz light aliasing sinusoid     
  T0MCR = (1<<0)|(1<<1); // Match Control Reg: Interrupt(b0) and Reset(b1) on MR0;
  T0TCR = 1;   // Timer0 Enable 

  //Enable capture register interrupts (CAP0 and CAP2 in both directions, for 2X
  //pulse counting and timing.
  T0CCR = 0
  | (1<<0)      //CAP0.0 rising edge IE
  | (1<<2)      //CAP0.0 capture interrupt enable
  | (1<<6)      //CAP0.2 rising edge IE
  | (1<<8)      //CAP0.2 capture interrupt enable
  ;

  // *******************************************************************************
  // Timer 1 setup for scheduler
  // *******************************************************************************

  //Set up the Timer Counter 1 Interrupt
  T1IR  = 0xFF;                           //clear interrupts in Timer1
  T1MR0 = (60000000/(SCHED_SPEED * 1000))-1;       
  T1MCR = (1<<0)|(1<<1); // Match Control Reg: Interrupt(b0) and Reset(b1) on MR0;
  T1TCR = 1;   // Timer1 Enable   
}

void timer1_isr(void) __irq
{
  asched_tick();
  T1IR = 0xFF;
  VICVectAddr = 0; //Interrupt Acknowledged
}
