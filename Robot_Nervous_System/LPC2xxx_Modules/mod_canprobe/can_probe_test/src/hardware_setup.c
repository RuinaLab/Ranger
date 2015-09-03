#include <includes.h>

void setup_hardware(void){

	VICIntEnable = 0;

	MAMCR |= (1<<1);		// fully enable memory accelerator module
	SCS = (1<<0)|(1<<1);       // enable high-speed GPIO0 (bit 0) and GPIO1 (bit 1) (Fast GPIO)

  /*********************************************************************/
  /* Timer0 Initialization                                             */
  /*********************************************************************/
  //initialize Timer0 for 1 ms ticks
//  T0PR = 0;//no prescaling
  //T0PR = 9;//no prescaling
  //TODO: Update this value for 10 MHz clock...currently for 12 MHz (already done?)
//  T0MR0 = (60000-1);//1 mS
//  T0TCR = 1;//enabled for counting
//  T0CTCR = 0;//simple timer
//  T0MCR = 0x00000003;//interrupt and reset on MR0
  
//  VICVectAddr0 = (unsigned long)Timer0_ISR;
//  VICVectCntl0 = 0x20 | VIC_TIMER0; /* Timer1 Interrupt */
//  VICIntEnable = 1 << VIC_TIMER0;   /* Enable Timer1 Interrupt */
  
  /*********************************************
   * Put user initialization code below here.  *
   *********************************************/ 
  /***********************************************
   * Heartbeat Init Section
   * Set pin direction to output for driving LEDs
   * Set initial LED state
   ***********************************************/
 
  FIO1DIR = 1<<23|1<<24|1<<25;
  FIO1SET = 1<<23|1<<24|1<<25; 
  FIO1CLR = 1<<25;
  FIO1CLR = 1<<24;
  FIO1SET = 1<<23;

  /********* End Heartbeat Init Section ************/
  /***********************************************
   * UART Init Section
   * 
   * 
   ***********************************************/
 
  //Set P0.0 to TXD0 and P0.1 to RXD0 
  PINSEL0 &= ~(0xF);
  PINSEL0 |= 0x5;
 
  U0LCR = (1<<7);//DLAB = 1 to enable baud rate changes
  //Rate config from Jason for 115.2k, with 1+0.3 frac divider and pclk = 60MHz
  U0DLL = 25;
  U0DLM = 0;
  U0FDR = 3;//DIVADDVAL = 3
  U0FDR |= 10<<4;//MULTVAL = 10;
  //
  
  U0LCR = (0<<7)|(3<<0);//DLAB = 0 to disable baud rate changes, Wordsize = 3 for 8 bit words
  
//  U0IER = (1<<1);//1: Enable THRE interrupt
  
  FIO0DIR |= 1<<10; //Unassert SHDN_L on uart level shifter
  FIO0SET = 1<<10;

  
//  VICVectAddr1 = (unsigned long)uarti_isr;
//  VICVectCntl1 = 0x20 | VIC_UART0; /* Timer1 Interrupt */
//  VICIntEnable = 1 << VIC_UART0;   /* Enable Timer1 Interrupt */


  /********* End Heartbeat Init Section ************/
 
/*  
  PINSEL1 |= 1<<18;     // enable CAN controller 1 pin RD1 (TD1 not PINSELable)
  C1MOD = 1;
  C1CMR = 0;
  C1GSR = 0;
  C1IER = 0;
  //C1BTR = 0;              // default value for BTR
  C1BTR = (0<<23)|(2<<20)|(5<<16)|(1<<14)|(0<<10)|(1<<0);
//  C2BTR_bit.BRP = 0;            // osc. freq. = 60MHz  assume VPB = 60MHz
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
  

  //C2MOD_bit.STM = 1;    // self test mode
  //C2MOD_bit.TM = 0;      // test mode

  //C2MOD_bit.RM = 0;
  //C1MOD |= 1<<2;//STM
  
  
  // acceptance filter: bypass
//  AFMR_bit.AccBP=1;
//  AFMR_bit.AccOff=1;
  AFMR = 1<<1;
  //AFMR |= 1<<0;
  
  IO1DIR|=1<<18;
  IO1CLR= 1<<18;
  
  VICVectAddr1 = (unsigned long)can_probe_can1rx_isr;
  VICVectCntl1 = 0x20 | VIC_CAN1RX;
  VICIntEnable = 1 << VIC_CAN1RX;
  
  VICVectAddr4 = (unsigned long)  can_error_isr;
  VICVectCntl4 = 0x20 | VIC_CAN_AF;
  VICIntEnable = 1 << VIC_CAN_AF;
  
  C1IER |= 1<<0; //Turn on RX interrupt
  C1IER |= 0xFC; //Turn on Error interrupts
  
  C1GSR = 0;//clear error counters
  
  C1MOD = 0;//Turn off RM
  
  */
  
  /*********************************************************************/
  /* CAN1 Initialization                                             */
  /*********************************************************************/  
  PINSEL1&=~(3<<18);
  PINSEL1 |= 1<<18;     // enable CAN controller 1 pin RD1 (TD1 not PINSELable)
//  FIO0DIR |= 1<<25;//Set 0.25 CAN RD1 To output...wait, why?
  
  C1MOD = 1;
//  C1CMR = 1<<1; //Abort transmission?  Because maybe putting the controller into reset mode isn't enough?
  C1CMR = 0;
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
  
 
//  C1IER |= 1<<1; //Turn on Tx 1 interrupt
  C1IER |= 1<<0; //Turn on RX interrupt
  C1IER |= 0xFC; //Turn on Error interrupts
  
//  C1GSR = 0;//clear error counters
  C1MOD = 0;//Turn off RM
  
  // acceptance filter: bypass
  //  AFMR_bit.AccBP=1;
  //  AFMR_bit.AccOff=0;
  AFMR = 1<<1;
//  IO1DIR|=1<<18;//Enable CAN tranceiver
//  IO1CLR= 1<<18;

  FIO1DIR|=1<<18;//Enable CAN tranceiver
  FIO1CLR= 1<<18;

  /*********************************************************************/
  /* CAN2 Initialization                                             */
  /*********************************************************************/  
  PINSEL1&=~(3<<14);
  PINSEL1 |= 1<<14;     // enable CAN controller 2 pin RD2
  PINSEL1&=~(3<<16);
  PINSEL1 |= 1<<16;     // enable CAN controller 2 pin TD2
//  FIO0DIR |= 1<<25;//Set 0.25 CAN RD1 To output...wait, why?
  
  C2MOD = 1;
//  C1CMR = 1<<1; //Abort transmission?  Because maybe putting the controller into reset mode isn't enough?
  C2CMR = 0;
  C2GSR = 0;
  C2IER = 0;
  //C1BTR = 0;              // default value for BTR
  C2BTR = (0<<23)|(2<<20)|(5<<16)|(1<<14)|(0<<10)|(1<<0);
  //  C2BTR_bit.BRP = 1;            // osc. freq. = 60MHz  assume VPB = 60MHz
  //  C2BTR_bit.SJW = 1;            // Synchronization Jump Width
  //  C2BTR_bit.TSEG1 = 5;          // Time Segment1 = 5+1 = 6
  //  C2BTR_bit.TSEG2 = 2;          // Time Segment2 = 2+1 = 3
  //  C2BTR_bit.SAM = 0;            // sample once, baud rate = 6MHz

   // default values
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
  
 
//  C1IER |= 1<<1; //Turn on Tx 1 interrupt
  C2IER |= (1<<0)&0x7FF; //Turn on RX interrupt
  C2IER |= (0xFC)&0x7FF; //Turn on Error interrupts
  
//  C1GSR = 0;//clear error counters
  C2MOD = 0;//Turn off RM
  
  // acceptance filter: bypass
  //  AFMR_bit.AccBP=1;
  //  AFMR_bit.AccOff=0;
  AFMR = 1<<1;
//  IO1DIR|=1<<18;//Enable CAN tranceiver
//  IO1CLR= 1<<18;

  FIO1DIR|=1<<19;//Enable CAN tranceiver
  FIO1CLR= 1<<19;


  /*********************************************************************/
  /* CAN3 Initialization                                             */
  /*********************************************************************/  
  PINSEL1&=~(3<<10);
  PINSEL1 |= 2<<10;     // enable CAN controller 3 pin RD3
  PINSEL1&=~(3<<12);
  PINSEL1 |= 1<<12;     // enable CAN controller 3 pin TD3
//  FIO0DIR |= 1<<25;//Set 0.25 CAN RD1 To output...wait, why?
  
  C3MOD = 1;
//  C1CMR = 1<<1; //Abort transmission?  Because maybe putting the controller into reset mode isn't enough?
  C3CMR = 0;
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
  
 
//  C1IER |= 1<<1; //Turn on Tx 1 interrupt
  C3IER |= 1<<0; //Turn on RX interrupt
  C3IER |= 0xFC; //Turn on Error interrupts
  
//  C1GSR = 0;//clear error counters
  C3MOD = 0;//Turn off RM
  
  // acceptance filter: bypass
  //  AFMR_bit.AccBP=1;
  //  AFMR_bit.AccOff=0;
  AFMR = 1<<1;
//  IO1DIR|=1<<18;//Enable CAN tranceiver
//  IO1CLR= 1<<18;

  FIO0DIR|=1<<8;//Enable CAN tranceiver
  FIO0CLR= 1<<8;

  /*********************************************************************/
  /* CAN4 Initialization                                             */
  /*********************************************************************/  
  PINSEL0&=~(3<<24);
  PINSEL0 |= 3<<24;     // enable CAN controller 4 pin RD4
  PINSEL0&=~(3<<26);
  PINSEL0 |= 3<<26;     // enable CAN controller 4 pin TD4
//  FIO0DIR |= 1<<25;//Set 0.25 CAN RD1 To output...wait, why?
  
  C4MOD = 1;
//  C1CMR = 1<<1; //Abort transmission?  Because maybe putting the controller into reset mode isn't enough?
  C4CMR = 0;
  C4GSR = 0;
  C4IER = 0;
  //C1BTR = 0;              // default value for BTR
  C4BTR = (0<<23)|(2<<20)|(5<<16)|(1<<14)|(0<<10)|(1<<0);
  //  C2BTR_bit.BRP = 1;            // osc. freq. = 60MHz  assume VPB = 60MHz
  //  C2BTR_bit.SJW = 1;            // Synchronization Jump Width
  //  C2BTR_bit.TSEG1 = 5;          // Time Segment1 = 5+1 = 6
  //  C2BTR_bit.TSEG2 = 2;          // Time Segment2 = 2+1 = 3
  //  C2BTR_bit.SAM = 0;            // sample once, baud rate = 6MHz

   // default values
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
  
 
//  C1IER |= 1<<1; //Turn on Tx 1 interrupt
  C4IER |= (1<<0)&0x7FF; //Turn on RX interrupt
  C4IER |= (0xFC)&0x7FF; //Turn on Error interrupts
  
//  C1GSR = 0;//clear error counters
  C4MOD = 0;//Turn off RM
  
  // acceptance filter: bypass
  //  AFMR_bit.AccBP=1;
  //  AFMR_bit.AccOff=0;
  AFMR = 1<<1;
//  IO1DIR|=1<<18;//Enable CAN tranceiver
//  IO1CLR= 1<<18;

  FIO0DIR|=1<<9;//Enable CAN tranceiver
  FIO0CLR= 1<<9;  
}

void Timer0_ISR(void) __irq {
  T0IR = 0xFFFFFFFF;  // Clear ALL Timer0 interrupt flags.
  schedule_tick();
  VICVectAddr = 0;    // Clear interrupt in VIC.  
}
