#include <includes.h>

CSR_ROUTE   csr_routes[2048];
CAN_RING  * csr_can_rx_ring;
int         csr_chan_enable[5];

void csr_init(CAN_RING * can_rx_ring) {
  int i;

  csr_can_rx_ring = can_rx_ring;
  csr_chan_enable[0] = 0;
  csr_chan_enable[1] = 0;
  csr_chan_enable[2] = 0;
  csr_chan_enable[3] = 0;
  csr_chan_enable[4] = 0;

  for(i = 0; i < 2048; i++) {
    csr_routes[i].source       = 0;
    csr_routes[i].destinations = 1<<CHAN_SSP;
  }
}

void csr_set_routes(short can_id,int destinations){
  csr_routes[can_id].destinations = destinations;
}

void csr_add_routes(short can_id,int destinations){
  csr_routes[can_id].destinations |= destinations;
}

void csr_enable_chan(CAN_CHANNEL chan){
  csr_chan_enable[chan] = 1;
}

void csr_route_now(void) {
  CAN_FRAME frame;

  if(csr_can_rx_ring && (can_ring_pop(csr_can_rx_ring,&frame) == 0)){
    csr_route_frame(&frame);
  }

  if(csr_ssp_pop_frame(&frame) == 0){
    csr_route_frame(&frame);
  }
}


void csr_route_frame(CAN_FRAME * frame) {
   int destinations;
   int rtr;
   int src;

  if(frame->rtr) {
    csr_routes[frame->addr].destinations |= 1<<(frame->chan);
  } else {
    csr_routes[frame->addr].source = (frame->chan);
  }
  
  destinations = csr_routes[frame->addr].destinations;
  rtr          = frame->rtr;
  src          = frame->chan;

  if((src != CHAN_SSP) && csr_chan_enable[CHAN_SSP] && ((destinations & (1<<CHAN_SSP)) || rtr)){
    frame->chan = CHAN_SSP;
    csr_ssp_push_frame(frame);
  }
  
  if((src != CHAN_CAN1) && csr_chan_enable[CHAN_CAN1] && ((destinations & (1<<CHAN_CAN1)) || rtr)){
    frame->chan = CHAN_CAN1;
    can_transmit_frame(frame);
  }
  
  if((src != CHAN_CAN2) && csr_chan_enable[CHAN_CAN2] && ((destinations & (1<<CHAN_CAN2)) || rtr)){
    frame->chan = CHAN_CAN2;
    can_transmit_frame(frame);
  }
  
  if((src != CHAN_CAN3) && csr_chan_enable[CHAN_CAN3] && ((destinations & (1<<CHAN_CAN3)) || rtr)){
    frame->chan = CHAN_CAN3;
    can_transmit_frame(frame);
  }
  
  if((src != CHAN_CAN4) && csr_chan_enable[CHAN_CAN4] && ((destinations & (1<<CHAN_CAN4)) || rtr)){
    frame->chan = CHAN_CAN4;
    can_transmit_frame(frame);
  }
  
}

