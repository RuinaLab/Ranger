#ifndef CSR_H
#define CSR_H

typedef __packed struct csr_routing_entry {
  int destinations : 5;
  int source       : 3;
} CSR_ROUTE;

void csr_init(CAN_RING * can_rx_ring);
void csr_add_routes(short can_id,int destinations);
void csr_set_routes(short can_id,int destinations);
void csr_enable_chan(CAN_CHANNEL chan);
void csr_route_now(void);
void csr_route_frame(CAN_FRAME * frame);

//void csr_ssp_init(CAN_RING * ring);
unsigned short int csr_ssp_pop_frame(CAN_FRAME * frame);
unsigned short int csr_ssp_push_frame(CAN_FRAME * frame);
void csr_ssp_timer0_isr(void) __irq;

#endif /* CSR_H */
