#include <includes.h>


#define SCHEDULE_TICK_DIVIDER 1 //number of system jiffies (should be 1 mS) per time slot
#define SCHEDULE_TICKS_PER_MS 1 //number of ticks per millisecond

const TASK_PTR schedule[]={
   (TASK_PTR)&task_heartbeat, (TASK_PTR)NULL,
   (TASK_PTR)&task_heartbeat, (TASK_PTR)NULL,
   (TASK_PTR)&task_heartbeat, (TASK_PTR)NULL,
   (TASK_PTR)&task_heartbeat, (TASK_PTR)NULL,
   (TASK_PTR)&task_heartbeat, (TASK_PTR)NULL,
   (TASK_PTR)&task_heartbeat, (TASK_PTR)NULL,
   (TASK_PTR)&task_heartbeat, (TASK_PTR)NULL,
   (TASK_PTR)&task_heartbeat, (TASK_PTR)NULL,
   (TASK_PTR)&task_heartbeat, (TASK_PTR)NULL,
   (TASK_PTR)&task_heartbeat, (TASK_PTR)NULL,
   (TASK_PTR)NULL
   };

void setup_software(void){
  /*********************************************
   * Scheduler Initialization.  Do not modify. *
   *********************************************/ 
  schedule_init(schedule,SCHEDULE_TICK_DIVIDER,SCHEDULE_TICKS_PER_MS);
 
  /*********************************************
   * Put user initialization code below here.  *
   *********************************************/ 
  set_heartbeat_period_ms(250);

  csr_init_can_rings();   //initialize CAN ring buffer (4 tx, 1 rx)
  
  csr_routing_table_init(); // **** TEST CODE set up test route
  
//  csr_enable_chan(CHAN_CAN1);
 // csr_enable_chan(CHAN_CAN2);
////////////////////////////////////////////////////////////////////
//Enable interrupts
////////////////////////////////////////////////////////////////////

  VICIntEnable = 0;
  VICVectAddr  = 0;
  VICSoftIntClr = 0xffffffff;  	 //clear all interupts - only do this once
  
  
  //SSP and associated Timer1 interrupt setup FIQ
  VICIntSelect |= 1 << 11;    //Enable FIQ interrupts for SSP
  VICIntSelect |= 1 << 5;     //Enable FIQ interrupts for Timer1
  VICIntEnable = 1 << 11;     // Enable SSP interrupt
  VICIntEnable = 1 << 5;      // Enable Timer1 interrupt
  T1TCR = 1;                  //Start Timer1
  
  
  
  // ************ PRIORITY 0 ******************
  //Timer0 interrupt setup
  VICVectAddr0 = (unsigned long)timer0_isr;
  VICVectCntl0 = 0x20 | 4; /* Timer0 Interrupt */
  VICIntEnable = 1 << 4;   /* Enable Timer0 Interrupt */
  
  // ************ PRIORITY 1 ******************
  //UART (uart_int)
//  VICVectAddr1 = (unsigned long)uarti_isr;
//  VICVectCntl1 = 0x20 | VIC_UART1;
//  VICIntEnable = 1 << VIC_UART1;

  // ************ PRIORITY 2 ******************
  //CAN1 RX
  VICVectAddr2 = (unsigned long)can_rx1_isr;
  VICVectCntl2 = 0x20 | VIC_CAN1RX;
  VICIntEnable = 1 << VIC_CAN1RX;

  // ************ PRIORITY 3 ******************
  //CAN1 TX
  VICVectAddr3 = (unsigned long)csr_can1_tx_isr;
  VICVectCntl3 = 0x20 | VIC_CAN1TX;
  VICIntEnable = 1 << VIC_CAN1TX; 

  // ************ PRIORITY 4 ******************
  //CAN2 RX
  VICVectAddr4 = (unsigned long)can_rx2_isr;
  VICVectCntl4 = 0x20 | VIC_CAN2RX;
  VICIntEnable = 1 << VIC_CAN2RX;

  // ************ PRIORITY 5 ******************
  //CAN2 TX
  VICVectAddr5 = (unsigned long)csr_can2_tx_isr;
  VICVectCntl5 = 0x20 | VIC_CAN2TX;
  VICIntEnable = 1 << VIC_CAN2TX;


//  if(!B2A_OVERRIDE) 
//  {
  
    // ************ PRIORITY 6 ******************
    //CAN3 RX
    VICVectAddr6 = (unsigned long)can_rx3_isr;
    VICVectCntl6 = 0x20 | VIC_CAN3RX;
    VICIntEnable = 1 << VIC_CAN3RX;
  
    // ************ PRIORITY 7 ******************
    //CAN3 TX
//    VICVectAddr7 = (unsigned long)can_tx3_isr;
    VICVectAddr7 = (unsigned long)csr_can3_tx_isr;
    VICVectCntl7 = 0x20 | VIC_CAN3TX;
    VICIntEnable = 1 << VIC_CAN3TX; 
  
    // ************ PRIORITY 8 ******************
    //CAN4 RX
    VICVectAddr8 = (unsigned long)can_rx4_isr;
    VICVectCntl8 = 0x20 | VIC_CAN4RX;
    VICIntEnable = 1 << VIC_CAN4RX;
  
    // ************ PRIORITY 9 ******************
    //CAN4 TX
    VICVectAddr9 = (unsigned long)csr_can4_tx_isr;
    VICVectCntl9 = 0x20 | VIC_CAN4TX;
    VICIntEnable = 1 << VIC_CAN4TX;


  //UART0 interrupt setup
  //VICVectCntl3 = (1<<5) + 6;	//Enable vectored interrupt slot 3 and set it to UART0 (interrupt source 6)
 // VICIntSelect &= ~(1<<6);	//Clear bit 6 - use IRQ interrupts for UART0, not FIQ.
  //VICIntEnable = 1<<6;		//Enable UART0 interrupt at vectored interrupt controller (VIC)
 // }
}
