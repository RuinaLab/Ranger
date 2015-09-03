#include <includes.h>

void setup_hardware(void){

	MAMCR |= (1<<1);		// fully enable memory accelerator module
	SCS |= (1<<0)|(1<<1);       // enable high-speed GPIO0 (bit 0) and GPIO1 (bit 1) (Fast GPIO)

	
	/*********************************************
	* Put user initialization code below here.  *
	*********************************************/ 	

	// ***********************************************
	// Heartbeat Init Section
	// ***********************************************
	//On-board LED initialization
	PINSEL2 &= ~(1<<3);	 // set trace port for GPIO use (bit 3 = 0)
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
	//	
	U0LCR = (0<<7)|(3<<0);//DLAB = 0 to disable baud rate changes, Wordsize = 3 for 8 bit words	
	U0IER = (1<<1);//1: Enable THRE interrupt	
	IO0DIR |= 1<<10; //Unassert SHDN_L on uart level shifter
	IO0SET = 1<<10;

	// *******************************************************************************
	// CAN Initialization                                             
	// *******************************************************************************  
	PINSEL1 &= ~(3<<18);
	PINSEL1 |= 1<<18;	// enable CAN controller 1 pin RD1 (TD1 not PINSELable)
//	FIO0DIR |= 1<<25;	//Set 0.25 CAN RD1 To output...wait, why?
	C1MOD = 1;
//	C1MOD |= (1<<2); //Self-test mode: Transmit always considered successful, even without ACK
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
	
	C1IER  =0;

//	C1IER = 1<<0; //Turn on Rx Interrupt
//	C1IER |= 1<<1; //Turn on Tx 1 interrupt
	//  C1GSR = 0;//clear error counters

	// acceptance filter: bypass
	//  AFMR_bit.AccBP=1;
	//  AFMR_bit.AccOff=0;
	AFMR = 1<<1;
	//  IO1DIR|=1<<18;//Enable CAN tranceiver
	//  IO1CLR= 1<<18;
	FIO1DIR|=1<<18;//Enable CAN tranceiver
	FIO1CLR= 1<<18;
	
	// *******************************************************************************
	// SCHEDULE Setup
	// *******************************************************************************
	//initialize Timer1 for 1 ms ticks for SCHEDULER
	//edited by Nic to use timer 1; previously used timer 0 but conflicted with QEC module
	T1PR = 0;//no prescaling
	T1MR0 = 60000000/(SCHED_SPEED*1000);//Match Register for scheduler interrupts
	T1TCR = 1;//enabled for counting
	T1CTCR = 0;//simple timer
	T1MCR = (1)|(1<<1);//interrupt (bit0) and reset (bit1) on MR0

}

void Timer1_ISR(void) __irq 
{
	T1IR = 0xFFFFFFFF;  // Clear ALL Timer1 interrupt flags.
	schedule_tick();
	VICVectAddr = 0;    // Clear interrupt in VIC.  
}
