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
  MCU_LED_ALL_OFF;  

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
  // SCHEDULE Setup
  // *******************************************************************************
  //Expect timestamp every ms
  PWMMCR |= (1<<0) & ~PWMMCR_RB; //Interrupt from match 0
  
}

void init_interrupts(void){
	// *************************************************
	// Interrupt enables
	// *************************************************
	VICIntEnable = 0;
	VICVectAddr  = 0;
	VICSoftIntClr = 0xffffffff;  	 //clear all interupts - only do this once

  // ************ PRIORITY 2 ******************  
	//PWM TIMER FOR SCHEDULE
	VICVectAddr2 = (unsigned long)schedule_isr;
	VICVectCntl2 = (1<<5) | VIC_PWM0; // (1<<5) is the interrupt enable bit, 8 is PWM interrupt
	VICIntEnable |= (1<<VIC_PWM0); //Enable PWM (bit8)
	
	// ************ PRIORITY 4 ******************
	//CAN1 TX
	VICVectAddr4 = (unsigned long)can_tx1_isr;
	VICVectCntl4 = 0x20 | VIC_CAN1TX;
	VICIntEnable = 1 << VIC_CAN1TX; 
	
	// ************ PRIORITY 5 ******************
	//CAN1 RX
	VICVectAddr5 = (unsigned long)can_rx1_isr;
	VICVectCntl5 = 0x20 | VIC_CAN1RX;
	VICIntEnable = 1 << VIC_CAN1RX;
	
	// ************ PRIORITY 6 ******************
	//CAN2 RX
	VICVectAddr6 = (unsigned long)can_rx2_isr;
	VICVectCntl6 = 0x20 | VIC_CAN2RX;
	VICIntEnable = 1 << VIC_CAN2RX;
	
	// ************ PRIORITY 7 ******************
	//CAN2 TX
	VICVectAddr7 = (unsigned long)can_tx2_isr;
	VICVectCntl7 = 0x20 | VIC_CAN2TX;
	VICIntEnable = 1 << VIC_CAN2TX;
	
	// ************ PRIORITY 8 ******************
	//CAN ERRORS
	VICVectAddr8 = (unsigned long)can_error_isr;
	VICVectCntl8 = (1<<5) | VIC_CAN_AF;
	VICIntEnable = 1<<VIC_CAN_AF;	 
   
}
