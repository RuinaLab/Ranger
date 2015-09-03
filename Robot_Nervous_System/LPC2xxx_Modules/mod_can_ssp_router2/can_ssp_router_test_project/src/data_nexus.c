#include <includes.h>

///////////////////////////////////////////////////////////////////////////////////////////////////
// Define ring buffers for data flow in the CAN-SSP router board

// CAN receive: The four CAN controllers share a single receive ring buffer
const unsigned short can_rx_frame_buf_len = 64;
CAN_RING can_rx_ring;
CAN_FRAME can_rx_frame_buf[can_rx_frame_buf_len];

// CAN transmit: Each of the four controllers has its own transmit ring buffer
const unsigned short can_tx_frame_buf_len = 8;
CAN_RING can_tx_ring1;
CAN_FRAME can_tx_frame_buf1[can_tx_frame_buf_len];
CAN_RING can_tx_ring2;
CAN_FRAME can_tx_frame_buf2[can_tx_frame_buf_len];
CAN_RING can_tx_ring3;
CAN_FRAME can_tx_frame_buf3[can_tx_frame_buf_len];
CAN_RING can_tx_ring4;
CAN_FRAME can_tx_frame_buf4[can_tx_frame_buf_len];

// Set up ring name and buffer for CAN-SSP
// outer board transmit (Error codes, battery voltage, system power, bus loading, etc.)
const unsigned short router_tx_frame_buf_len = 8;
CAN_RING router_tx_ring;
CAN_FRAME router_tx_frame_buf[router_tx_frame_buf_len];

///////////////////////////////////////////////////////////////////////////////////////////////////

void router_data_nexus_init(void)
{
  
  // Initialize CAN descriptor lists (not used in router board)
  can_rx_set_descriptors(NULL, NULL);

  // Initialize common CAN receive ring
  can_ring_init(&can_rx_ring, can_rx_frame_buf, can_rx_frame_buf_len);

  // Initialize individual CAN transmit rings, and set channel configurations
  // CAN dispatch is "manual" - incoming frames go to the receive ring buffer

  // ********* CAN1 ***********
  can_ring_init(&can_tx_ring1, can_tx_frame_buf1, can_tx_frame_buf_len);
  can_tx_set_chan_cfg(CHAN_CAN1, (volatile unsigned long *)0xE0044000, &can_tx_ring1);
  can_rx_set_chan_cfg(CHAN_CAN1, (volatile unsigned long *)0xE0044000, &can_rx_ring, CAN_DISPATCH_MANUAL);

   // ******** CAN2 ***********
  can_ring_init(&can_tx_ring2,can_tx_frame_buf2, can_tx_frame_buf_len);
  can_tx_set_chan_cfg(CHAN_CAN2, (volatile unsigned long *)0xE0048000, &can_tx_ring2);
  can_rx_set_chan_cfg(CHAN_CAN2, (volatile unsigned long *)0xE0048000, &can_rx_ring,CAN_DISPATCH_MANUAL);

   // ******** CAN3 ***********
  can_ring_init(&can_tx_ring3, can_tx_frame_buf3, can_tx_frame_buf_len);
  can_tx_set_chan_cfg(CHAN_CAN3, (volatile unsigned long *)0xE004C000, &can_tx_ring3);
  can_rx_set_chan_cfg(CHAN_CAN3, (volatile unsigned long *)0xE004C000, &can_rx_ring,CAN_DISPATCH_MANUAL);

   // ******** CAN4 ***********
  can_ring_init(&can_tx_ring4, can_tx_frame_buf4, can_tx_frame_buf_len);
  can_tx_set_chan_cfg(CHAN_CAN4, (volatile unsigned long *)0xE0050000, &can_tx_ring4);
  can_rx_set_chan_cfg(CHAN_CAN4,(volatile unsigned long *)0xE0050000,&can_rx_ring,CAN_DISPATCH_MANUAL);

  // Initialize ring buffer for router board transmit (error codes, voltage, power, CAN bus load, etc.)
  can_ring_init(&router_tx_ring, router_tx_frame_buf, router_tx_frame_buf_len);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Take incoming data from SSP receive buffer and CAN receive buffer.
//Call csr_route_frame to distribute it to the correct locations
void route_frames(void)
{
  CAN_FRAME frame;

  //pop one CAN frame pointer from ssp; route if available
  if (!csr_pop_ssp_frame(&frame))
  {
    // **** TEST CODE ****  line below commented out for testing.
    csr_route_frame(&frame);
  }

  //pop one CAN frame from CAN receive buffer; route if available
  if (!can_ring_pop(&can_rx_ring, &frame))
  {
// **** TEST CODE ****  line below should be commented out for bench testing with interconnected CAN buses.
    csr_route_frame(&frame);
  }

  //pop one CAN frame from router board transmit buffer; route if available
  if (!can_ring_pop(&router_tx_ring, &frame))
  {
    csr_route_frame(&frame);
  }
  
}

// Build error frame, and push it onto the router board ring buffer for transmission to main brain
 void router_error_transmit(void)
 {
   CAN_FRAME frame;
   
   //Build CSR error frame
   frame.addr = ID_ERROR_CSR;
   frame.dlc = 8;
   frame.rtr = 0;
   frame.chan = CHAN_ROUTER;
   frame.payload.w.w1 = error_get_info();
   frame.payload.w.w2 = error_get_time();

  //Push frame onto router board (csr) ring buffer
   can_ring_push(&router_tx_ring, &frame);
 }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
