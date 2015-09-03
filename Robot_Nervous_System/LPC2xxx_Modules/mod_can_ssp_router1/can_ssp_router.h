#ifndef  __CAN_SSP_ROUTER_H__
#define  __CAN_SSP_ROUTER_H__

//__irq void csr_ssp_isr(void);

void csr_routing_table_init(void);
void csr_init_can_rings(void);
void csr_ssp_isr(void); //Set up for FIQ interrupts
void csr_route(void);
unsigned short int csr_route_frame(CAN_FRAME * frame);
unsigned short int csr_push_ssp_frame(CAN_FRAME * frame);
unsigned short int csr_pop_ssp_frame(CAN_FRAME * frame);
unsigned short int csr_can1_tx_push_frame(CAN_FRAME * frameptr);
void csr_can1_tx_isr(void) __irq;
unsigned short int csr_can2_tx_push_frame(CAN_FRAME * frameptr);
void csr_can2_tx_isr(void) __irq;
unsigned short int csr_can3_tx_push_frame(CAN_FRAME * frameptr);
void csr_can3_tx_isr(void) __irq;
unsigned short int csr_can4_tx_push_frame(CAN_FRAME * frameptr);
void csr_can4_tx_isr(void) __irq;

#endif// __CAN_SSP_ROUTER_H__

