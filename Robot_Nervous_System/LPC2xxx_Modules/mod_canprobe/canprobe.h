#ifndef  __CANPROBE_H__
#define  __CANPROBE_H__

void can_probe_set_ring(CAN_RING * ring);
void can_probe_spit(void);
/*
int can_probe_push(CAN_FRAME * frame);
int can_probe_pop(CAN_FRAME * frame);
void can_probe_tx_next_frame(void);
void can_probe_go(void);
__irq void can_probe_can1rx_isr (void);
__irq void can_error_isr(void);
*/

#endif /* __CANPROBE_H__ */
