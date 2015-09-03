#include <includes.h>


#define SCHEDULE_TICK_DIVIDER 1 //number of system jiffies (should be 1 mS) per time slot
#define SCHEDULE_TICKS_PER_MS 1 //number of ticks per millisecond

//Set up ring name and buffer for SSP transmit
#define	CSR_SSP_TX_FRAME_BUF_LEN 128
CAN_RING csr_ssp_tx_ring;
CAN_FRAME csr_ssp_tx_frame_buf[CSR_SSP_TX_FRAME_BUF_LEN];

//Set up ring name and buffer for CAN receive
#define RX_FRAME_BUF_LEN 64
CAN_RING rx_ring;
CAN_FRAME rx_frame_buf[RX_FRAME_BUF_LEN];

//Set up 4 ring names and buffers for CAN transmit
#define TX_FRAME_BUF_LEN 16
CAN_RING tx1_ring;
CAN_FRAME tx1_frame_buf[TX_FRAME_BUF_LEN];
CAN_RING tx2_ring;
CAN_FRAME tx2_frame_buf[TX_FRAME_BUF_LEN];
CAN_RING tx3_ring;
CAN_FRAME tx3_frame_buf[TX_FRAME_BUF_LEN];
CAN_RING tx4_ring;
CAN_FRAME tx4_frame_buf[TX_FRAME_BUF_LEN];

const TASK_PTR schedule[]={
   (TASK_PTR)&hb_beat, (TASK_PTR)NULL,
   (TASK_PTR)&hb_beat, (TASK_PTR)NULL,
   (TASK_PTR)&hb_beat, (TASK_PTR)NULL,
   (TASK_PTR)&hb_beat, (TASK_PTR)NULL,
   (TASK_PTR)&hb_beat, (TASK_PTR)NULL,
   (TASK_PTR)&hb_beat, (TASK_PTR)NULL,
   (TASK_PTR)&hb_beat, (TASK_PTR)NULL,
   (TASK_PTR)&hb_beat, (TASK_PTR)NULL,
   (TASK_PTR)&hb_beat, (TASK_PTR)NULL,
   (TASK_PTR)&hb_beat, (TASK_PTR)NULL,
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
//  set_heartbeat_period_ms(250);

  //initialize SSP ring buffer
  can_ring_init(&csr_ssp_tx_ring,csr_ssp_tx_frame_buf,CSR_SSP_TX_FRAME_BUF_LEN);	//SSP main transmit	ring buffer
  csr_ssp_tx_ring_ptr_set(&csr_ssp_tx_ring);

  //initialize CAN ring buffers
  can_ring_init(&rx_ring,rx_frame_buf,RX_FRAME_BUF_LEN);
  can_rx_set_chan_cfg(CHAN_CAN1, (volatile unsigned long *)0xE0044000, &rx_ring, CAN_DISPATCH_MANUAL); //CAN controller 1
  can_rx_set_chan_cfg(CHAN_CAN2, (volatile unsigned long *)0xE0048000, &rx_ring, CAN_DISPATCH_MANUAL); //CAN controller 2
  can_rx_set_chan_cfg(CHAN_CAN3, (volatile unsigned long *)0xE004C000, &rx_ring, CAN_DISPATCH_MANUAL); //CAN controller 3
  can_rx_set_chan_cfg(CHAN_CAN4, (volatile unsigned long *)0xE0050000, &rx_ring, CAN_DISPATCH_MANUAL); //CAN controller 4
 
  can_ring_init(&tx1_ring,tx1_frame_buf,TX_FRAME_BUF_LEN);				//CAN controller 1 transmit
  can_ring_init(&tx2_ring,tx2_frame_buf,TX_FRAME_BUF_LEN);				//CAN controller 2 transmit
  can_ring_init(&tx3_ring,tx3_frame_buf,TX_FRAME_BUF_LEN);				//CAN controller 3 transmit
  can_ring_init(&tx4_ring,tx4_frame_buf,TX_FRAME_BUF_LEN);				//CAN controller 4 transmit

  can_tx_set_chan_cfg(CHAN_CAN1, (volatile unsigned long *)0xE0044000, &tx1_ring); //CAN controller 1
  can_tx_set_chan_cfg(CHAN_CAN2, (volatile unsigned long *)0xE0048000, &tx2_ring); //CAN controller 2
  can_tx_set_chan_cfg(CHAN_CAN3, (volatile unsigned long *)0xE004C000, &tx3_ring); //CAN controller 3
  can_tx_set_chan_cfg(CHAN_CAN4, (volatile unsigned long *)0xE0050000, &tx4_ring); //CAN controller 4

////////////////////////////////////////////////////////////////////
//Enable interrupts
////////////////////////////////////////////////////////////////////

  #ifdef DEBUG
    while (FIO0PIN & 1<<14)
    {
      FIO0CLR = 1<<24;   //Turn on MCU red LED
    }
  #endif
  
  //Timer0 interrupt setup
  VICVectAddr0 = (unsigned long)timer0_isr;
  VICVectCntl0 = 0x20 | 4; /* Timer1 Interrupt */
  VICIntEnable = 1 << 4;   /* Enable Timer1 Interrupt */

/*
  //SSP interrupt setup IRQ
  VICVectAddr2 = (unsigned long)csr_ssp_isr;
  VICVectCntl2 = 0x20 | 11; // SSP Interrupt, set to level 2
  VICIntEnable = 1 << 11;   // Enable SSP interrupt
  */
  
  //SSP interrupt setup FIQ
  VICIntSelect |= 1 << 11;   //Enable FIQ interrupts for SSP
  VICIntEnable = 1 << 11;   // Enable SSP interrupt

  //UART0 interrupt setup
  //VICVectCntl3 = (1<<5) + 6;	//Enable vectored interrupt slot 3 and set it to UART0 (interrupt source 6)
 // VICIntSelect &= ~(1<<6);	//Clear bit 6 - use IRQ interrupts for UART0, not FIQ.
  //VICIntEnable = 1<<6;		//Enable UART0 interrupt at vectored interrupt controller (VIC)
}
