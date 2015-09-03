#include <includes.h>

#define CAN_PROBE_RING_SIZE 8
CAN_FRAME  rxring_buf[CAN_PROBE_RING_SIZE];
CAN_RING   rxring;

void setup_software(void){
  /*********************************************
   * Scheduler Initialization.  Do not modify. *
   *********************************************/ 
//  schedule_init();
 
  /*********************************************
   * Put user initialization code below here.  *
   *********************************************/ 
  set_heartbeat_period_ms(250);

  can_ring_init(&rxring,rxring_buf,CAN_PROBE_RING_SIZE);
  can_probe_set_ring(&rxring);
  
  can_rx_set_chan_cfg(CHAN_CAN1,(volatile unsigned long *)0xE0044000, &rxring, CAN_DISPATCH_MANUAL);
  can_rx_set_chan_cfg(CHAN_CAN2,(volatile unsigned long *)0xE0048000, &rxring, CAN_DISPATCH_MANUAL);
  can_rx_set_chan_cfg(CHAN_CAN3,(volatile unsigned long *)0xE004C000, &rxring, CAN_DISPATCH_MANUAL);
  can_rx_set_chan_cfg(CHAN_CAN4,(volatile unsigned long *)0xE0050000, &rxring, CAN_DISPATCH_MANUAL);
  
//  VICVectAddr1 = (unsigned long)can_tx_isr;
//  VICVectCntl1 = 0x20 | VIC_CAN1TX;
//  VICIntEnable = 1 << VIC_CAN1TX;
  
  VICVectAddr2 = (unsigned long)can_rx1_isr;
  VICVectCntl2 = 0x20 | VIC_CAN1RX;
  VICIntEnable = 1 << VIC_CAN1RX;  
  
  VICVectAddr3 = (unsigned long)can_rx2_isr;
  VICVectCntl3 = 0x20 | VIC_CAN2RX;
  VICIntEnable = 1 << VIC_CAN2RX;
  
  VICVectAddr4 = (unsigned long)can_rx3_isr;
  VICVectCntl4 = 0x20 | VIC_CAN3RX;
  VICIntEnable = 1 << VIC_CAN3RX;  
  
  VICVectAddr5 = (unsigned long)can_rx4_isr;
  VICVectCntl5 = 0x20 | VIC_CAN4RX;
  VICIntEnable = 1 << VIC_CAN4RX;
  
//  VICVectAddr0 = (unsigned long)timer0_isr;
//  VICVectCntl0 = 0x20 | 4; /* Timer1 Interrupt */
//  VICIntEnable = 1 << 4;   /* Enable Timer1 Interrupt */
  
  VICVectAddr6 = (unsigned long)can_error_isr;
  VICVectCntl6 = 0x20 | VIC_CAN_AF;
//  VICIntEnable = 1 << VIC_CAN_AF;
  
  
}
