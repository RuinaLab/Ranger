#include <includes.h>


#define SCHEDULE_TICK_DIVIDER 1 //number of system jiffies (should be 1 mS) per time slot
#define SCHEDULE_TICKS_PER_MS 1 //number of ticks per millisecond

#define RUN_EVERY_LINE (TASK_PTR)&b10a_update_can_leds, (TASK_PTR)&b10a_update_mcu_leds, (TASK_PTR)&error_update, (TASK_PTR)&hb_beat_count
const TASK_PTR schedule[]=
  {
    RUN_EVERY_LINE, (TASK_PTR)&error_send_next, (TASK_PTR)NULL,
    RUN_EVERY_LINE, (TASK_PTR)NULL,
    RUN_EVERY_LINE, (TASK_PTR)NULL,
    RUN_EVERY_LINE, (TASK_PTR)NULL,
    RUN_EVERY_LINE, (TASK_PTR)NULL,
    RUN_EVERY_LINE, (TASK_PTR)NULL,
    RUN_EVERY_LINE, (TASK_PTR)NULL,
    RUN_EVERY_LINE, (TASK_PTR)NULL,
    RUN_EVERY_LINE, (TASK_PTR)NULL,
    RUN_EVERY_LINE, (TASK_PTR)NULL,
    (TASK_PTR)NULL
  };
  
//////////////////////////////////////////////////////////////////////////////////////////////

/*
// ********************************CAN*********************************
#define CAN_RX_FRAME_BUF_LEN 64
CAN_RING can_rx_ring;
CAN_FRAME can_rx_frame_buf[CAN_RX_FRAME_BUF_LEN];

#define CAN_TX_FRAME_BUF_LEN 8
CAN_RING can_tx_ring1;
CAN_FRAME can_tx_frame_buf1[CAN_TX_FRAME_BUF_LEN];
CAN_RING can_tx_ring2;
CAN_FRAME can_tx_frame_buf2[CAN_TX_FRAME_BUF_LEN];
CAN_RING can_tx_ring3;
CAN_FRAME can_tx_frame_buf3[CAN_TX_FRAME_BUF_LEN];
CAN_RING can_tx_ring4;
CAN_FRAME can_tx_frame_buf4[CAN_TX_FRAME_BUF_LEN];

//////////////////////////////////////////////////////////////////////////////////////////////
*/
  
void heartbeat_blink_blue(void)
{
  b10a_mcu_blue_led_blink(50);
}

void setup_software(void)
{
  /*********************************************
   * Scheduler Initialization.  Do not modify. *
   *********************************************/ 
  asched_init(schedule,SCHEDULE_TICK_DIVIDER);
  
 /*********************************************
  * Put user initialization code below here.  *
  *********************************************/
  error_init(router_error_transmit, asched_get_timestamp);
  hb_init(500, &heartbeat_blink_blue, asched_get_timestamp);

  //Zero out global variables. Buffer indexes can be corrupted during debugger operation
  csr_global_variable_init();
   
    
//set_heartbeat_period_ms(250);

  // Initialize CAN router board functions and buffers
  router_data_nexus_init();
  
  csr_routing_table_init(); // initialize routing table to default values
  
//can_rx_acceptance_filter_init();
  
//csr_enable_chan(CHAN_CAN1);
//csr_enable_chan(CHAN_CAN2);
 
// *******************************************
// Set up CAN
// *******************************************
  
  // Select functions to be called when can_isr receieves a CAN frame, transmits a frame, or has an error.
  // Functions must take the CAN channel number (1 - 4) as an argument.
  // To do: make the error callback function more detailed.
  can_init_status_callback(&b10a_can_packet_count,   // Count received frame on a given CAN channel
                                &b10a_can_packet_count,   // Count transmitted frame on a given CAN channel
                                &b10a_blink_red_can_led); // Error on a given CAN channel
/*
  can_rx_set_descriptors(NULL, NULL);
  can_ring_init(&can_rx_ring, can_rx_frame_buf, CAN_RX_FRAME_BUF_LEN);

  // ********* CAN1 ***********
  can_ring_init(&can_tx_ring1, can_tx_frame_buf1, CAN_TX_FRAME_BUF_LEN);
  can_tx_set_chan_cfg(CHAN_CAN1, (volatile unsigned long *)0xE0044000, &can_tx_ring1);
  can_rx_set_chan_cfg(CHAN_CAN1, (volatile unsigned long *)0xE0044000, &can_rx_ring, CAN_DISPATCH_MANUAL);

   // ******** CAN2 ***********
  can_ring_init(&can_tx_ring2,can_tx_frame_buf2,CAN_TX_FRAME_BUF_LEN);
  can_tx_set_chan_cfg(CHAN_CAN2, (volatile unsigned long *)0xE0048000, &can_tx_ring2);
  can_rx_set_chan_cfg(CHAN_CAN2, (volatile unsigned long *)0xE0048000, &can_rx_ring,CAN_DISPATCH_MANUAL);

   // ******** CAN3 ***********
  can_ring_init(&can_tx_ring3, can_tx_frame_buf3, CAN_TX_FRAME_BUF_LEN);
  can_tx_set_chan_cfg(CHAN_CAN3, (volatile unsigned long *)0xE004C000, &can_tx_ring3);
  can_rx_set_chan_cfg(CHAN_CAN3, (volatile unsigned long *)0xE004C000, &can_rx_ring,CAN_DISPATCH_MANUAL);

   // ******** CAN4 ***********
  can_ring_init(&can_tx_ring4, can_tx_frame_buf4, CAN_TX_FRAME_BUF_LEN);
  can_tx_set_chan_cfg(CHAN_CAN4, (volatile unsigned long *)0xE0050000, &can_tx_ring4);
  can_rx_set_chan_cfg(CHAN_CAN4,(volatile unsigned long *)0xE0050000,&can_rx_ring,CAN_DISPATCH_MANUAL);

  csr_init_can_rx_ring(&can_rx_ring);
  */
  /*
  // *******************************************
  // Set up CAN/SSP Router
  // *******************************************
  csr_init(&can_rx_ring);
  
  csr_enable_chan(CHAN_SSP);
  csr_enable_chan(CHAN_CAN1);
  csr_enable_chan(CHAN_CAN2);
  csr_enable_chan(CHAN_CAN3);
  csr_enable_chan(CHAN_CAN4);
*/

  //Synchronize with ARM9 board. Execution will wait here until two ready codes
  //have been successfully exchanged with the ARM9. This function should be called
  //immediately prior to enabling of the interrupts.
  //csr_synchronize_arm9(); 
 
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
//  VICVectAddr3 = (unsigned long)csr_can1_tx_isr;
  VICVectAddr3 = (unsigned long)can_tx1_isr;
  VICVectCntl3 = 0x20 | VIC_CAN1TX;
  VICIntEnable = 1 << VIC_CAN1TX; 

  // ************ PRIORITY 4 ******************
  //CAN2 RX
  VICVectAddr4 = (unsigned long)can_rx2_isr;
  VICVectCntl4 = 0x20 | VIC_CAN2RX;
  VICIntEnable = 1 << VIC_CAN2RX;

  // ************ PRIORITY 5 ******************
  //CAN2 TX
 // VICVectAddr5 = (unsigned long)csr_can2_tx_isr;
  VICVectAddr5 = (unsigned long)can_tx2_isr;
  VICVectCntl5 = 0x20 | VIC_CAN2TX;
  VICIntEnable = 1 << VIC_CAN2TX;
  
    // ************ PRIORITY 6 ******************
    //CAN3 RX
    VICVectAddr6 = (unsigned long)can_rx3_isr;
    VICVectCntl6 = 0x20 | VIC_CAN3RX;
    VICIntEnable = 1 << VIC_CAN3RX;
  
    // ************ PRIORITY 7 ******************
    //CAN3 TX
    VICVectAddr7 = (unsigned long)can_tx3_isr;
   // VICVectAddr7 = (unsigned long)csr_can3_tx_isr;
    VICVectCntl7 = 0x20 | VIC_CAN3TX;
    VICIntEnable = 1 << VIC_CAN3TX; 
  
    // ************ PRIORITY 8 ******************
    //CAN4 RX
    VICVectAddr8 = (unsigned long)can_rx4_isr;
    VICVectCntl8 = 0x20 | VIC_CAN4RX;
    VICIntEnable = 1 << VIC_CAN4RX;
  
    // ************ PRIORITY 9 ******************
    //CAN4 TX
  //  VICVectAddr9 = (unsigned long)csr_can4_tx_isr;
    VICVectAddr9 = (unsigned long)can_tx4_isr;    
    VICVectCntl9 = 0x20 | VIC_CAN4TX;
    VICIntEnable = 1 << VIC_CAN4TX;
    
    // ************ PRIORITY 10 ******************
    //CAN common (for error handling)
    VICVectAddr10 = (unsigned long)can_error_isr;
    VICVectCntl10 = 0x20 | 19;
    VICIntEnable = 1 << 19;


  //UART0 interrupt setup
  //VICVectCntl3 = (1<<5) + 6;	//Enable vectored interrupt slot 3 and set it to UART0 (interrupt source 6)
 // VICIntSelect &= ~(1<<6);	//Clear bit 6 - use IRQ interrupts for UART0, not FIQ.
  //VICIntEnable = 1<<6;		//Enable UART0 interrupt at vectored interrupt controller (VIC)
 // }
}
