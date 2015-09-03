#ifndef  __CAN_SSP_ROUTER_H__
#define  __CAN_SSP_ROUTER_H__

#define SSP 4

//__irq void csr_ssp_isr(void);

void csr_ssp_isr(void); //Set up for FIQ interrupts
void csr_route(void);
void csr_ssp_tx_ring_ptr_set(CAN_RING * ring);
unsigned short int csr_route_frame(CAN_FRAME * frame);
unsigned short int csr_pop_ssp_frame(CAN_FRAME * frame);
void csr_ssp_prime(void);

#endif// __CAN_SSP_ROUTER_H__

