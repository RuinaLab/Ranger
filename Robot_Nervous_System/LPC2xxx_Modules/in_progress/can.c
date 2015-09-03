/**
	can.c
	
*/
#include <includes.h>

// ************************************************************************************
// TRANSMIT
// ************************************************************************************
// CAN transmit frame count global variables:
volatile unsigned short can_tx_frame_count_1;
volatile unsigned short can_tx_frame_count_2;
volatile unsigned short can_tx_frame_count_3;
volatile unsigned short can_tx_frame_count_4;
CAN_TX_CHAN_CFG  can_tx_chan_cfgs[5];   //Five elements, index 1 to 4

// Set up CAN_TX_CHAN_CFG for channel chan; stalled is initialized to 1 to kickstart transmit process
void can_tx_set_chan_cfg(CAN_CHANNEL chan,volatile unsigned long * base_addr, CAN_RING * tx_ring){
  can_tx_chan_cfgs[chan].base_addr = base_addr;
  can_tx_chan_cfgs[chan].ring      = tx_ring;
  can_tx_chan_cfgs[chan].stalled   = 1;
}
// Basic can_transmit function, calls multi-argument can_transmit_alt
int can_transmit(CAN_FRAME_DESC * frame_desc){
  return can_transmit_alt(frame_desc,frame_desc->chan, frame_desc->rtr);
}
// CAN transmit function; has options for CAN channel and rtr (remote transmission request)
int can_transmit_alt(CAN_FRAME_DESC * frame_desc, CAN_CHANNEL chan, char rtr){
  CAN_LAYOUT  layout = frame_desc->frame_layout;
  CAN_FRAME   frame;
  int         dlc;

// Set data length code of transmitted frame, based on rtr bit or CAN_LAYOUT
  if(rtr){
    dlc = 0;
  }else{
    switch(layout){
      case CAN_LAYOUT_D:
      case CAN_LAYOUT_FF:
      case CAN_LAYOUT_II:
      case CAN_LAYOUT_FI:
      case CAN_LAYOUT_ISS:
        dlc = 8;
        break;
      default:
        //Unrecognized layout, throw error and abort.
        return 1;
    }
  }
 
  // Use getter functions to fill in payload data bytes of transmitted frame.
  // Function pointers are from the frame descriptor in the first argument
  if(rtr) {
    dlc = 0;
  } else {
    switch(layout){
      //WARNING!!! not for the faint of heart.  Here we are casting function pointers
      //to another type of function pointer, and then we call it.  If this doesn't
      //make sense, read about function pointers and casting.
      case CAN_LAYOUT_D:
        frame.payload.d.d1 = ((CAN_TX_GETTER_DOUBLE)frame_desc->ptr1)();
        dlc = 8;
        break;
      case CAN_LAYOUT_FF:
        frame.payload.f.f1 = ((CAN_TX_GETTER_FLOAT)frame_desc->ptr1)();
        frame.payload.f.f2 = ((CAN_TX_GETTER_FLOAT)frame_desc->ptr2)();
        dlc = 8;
        break;
      case CAN_LAYOUT_II:
        frame.payload.i.i1 = ((CAN_TX_GETTER_INT)frame_desc->ptr1)();
        frame.payload.i.i2 = ((CAN_TX_GETTER_INT)frame_desc->ptr2)();
        dlc = 8;
  	  break;
      case CAN_LAYOUT_FI:
        frame.payload.f.f1 = ((CAN_TX_GETTER_FLOAT)frame_desc->ptr1)();
        frame.payload.i.i2 = ((CAN_TX_GETTER_INT)frame_desc->ptr2)();
        dlc = 8;
      break;
      case CAN_LAYOUT_ISS:
        frame.payload.i.i1 = ((CAN_TX_GETTER_INT  )frame_desc->ptr1)();
        frame.payload.s.s3 = ((CAN_TX_GETTER_SHORT)frame_desc->ptr2)();
        frame.payload.s.s4 = ((CAN_TX_GETTER_SHORT)frame_desc->ptr3)();
        dlc = 8;
  	  break;
      default:
        //Unrecognized layout, throw error and abort.
        return 1;
    }
  }
  // Fill in the rest of the CAN_FRAME elements
  // Note that the channel number chan comes from the frame descriptor here, not the argument chan.
  // This may or may not be useful. *********
  // So the chan argument does nothing at this point.
  frame.addr = frame_desc->addr;
  frame.chan = frame_desc->chan;
  frame.dlc  = dlc;
  frame.rtr  = rtr;
  
  //The frame is now fully populated with Address, Data Length, and Payload
  //and is therefore ready to be sent out.
  return can_ring_push(can_tx_chan_cfgs[chan].ring, &frame); //returns 0 if successful, 1 if buffer is full and can't add to list
  //return can_transmit_frame(&frame);
}

static void can_add_frame_count(int chan){
  switch (chan){
    case 1: can_tx_frame_count_1++; break;
    case 2: can_tx_frame_count_2++; break;
    case 3: can_tx_frame_count_3++; break;
    case 4: can_tx_frame_count_4++; break;
    default: break;
  }
}

int can_tx_now(CAN_CHANNEL chan){
  volatile unsigned long* base = can_tx_chan_cfgs[chan].base_addr;
  CAN_FRAME frame;
  int sent = 0;
  if (CAN_REG(base, CAN_SR) & (1<<2)) // buffer 1 is available
  {   
    can_ring_pop(can_tx_chan_cfgs[chan].ring, &frame);
    CAN_REG(base, CAN_TFI1) = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
    CAN_REG(base, CAN_TID1) = frame.addr;   //get CAN_ID
    CAN_REG(base, CAN_TDA1) = frame.payload.w.w1;   //load first data word
    CAN_REG(base, CAN_TDB1) = frame.payload.w.w2;   //load second data word	
    CAN_REG(base, CAN_CMR) = (1<<0) | (1<<5);     // Select buffer 1, request transmission  
    can_add_frame_count(chan);
    sent++;
  }
  else if (CAN_REG(base, CAN_SR) & (1<<10)) // buffer 2 is available
  {
    can_ring_pop(can_tx_chan_cfgs[chan].ring, &frame);
    CAN_REG(base, CAN_TFI2) = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
    CAN_REG(base, CAN_TID2) = frame.addr;   //get CAN_ID
    CAN_REG(base, CAN_TDA2) = frame.payload.w.w1;   //load first data word
    CAN_REG(base, CAN_TDB2) = frame.payload.w.w2;   //load second data word	
    CAN_REG(base, CAN_CMR) = (1<<0) | (1<<6);     // Select buffer 2, request transmission  
    can_add_frame_count(chan);
    sent++;
  }
  return sent;
}

// ************************************************************************************
// RECEIVE
// ************************************************************************************
//CAN_FRAME_DESC   ** can_rx_descriptors;
//CAN_FRAME_DESC   ** can_rx_rtr_descriptors;
//CAN_RX_CHAN_CFG     can_rx_chan_cfgs[5];
//Set up CAN receive buffers (version 2)
//#define CAN_RX_BUFFER_SIZE  16
//volatile CAN_FRAME can_rx_buffer[CAN_RX_BUFFER_SIZE];
//volatile short int can_rx_index1 = 0;
//volatile short int can_rx_index2 = 0;

//void can_rx_set_chan_cfg(CAN_CHANNEL chan,volatile unsigned long * base_addr, CAN_RING * rx_ring, CAN_DISPATCH_MODE mode){
//  CAN_RX_CHAN_CFG * cfg = &can_rx_chan_cfgs[chan];
//  
//  cfg->base_addr     = base_addr;
//  cfg->dispatch_mode = mode;
//  cfg->ring          = rx_ring; 
//}
//
//void can_rx_set_descriptors(CAN_FRAME_DESC ** rx_descriptors,CAN_FRAME_DESC ** rtr_descriptors){
//  can_rx_descriptors     = rx_descriptors;
//  can_rx_rtr_descriptors = rtr_descriptors;
//}
//
////void can_rx1_fiq(void){can_rx_fiq(CHAN_CAN1);}
////void can_rx2_fiq(void){can_rx_fiq(CHAN_CAN2);}
////void can_rx3_fiq(void){can_rx_fiq(CHAN_CAN3);}
////void can_rx4_fiq(void){can_rx_fiq(CHAN_CAN4);}
////Read CAN rx controller buffer, create CAN frame and push onto receive ring buffer
//void can_rx1_fiq(void)
//{
//  short int temp_index = can_rx_index1;
//
//  if (++temp_index == CAN_RX_BUFFER_SIZE) {temp_index = 0;}
//  if (temp_index == can_rx_index2)
//  {
//    //Buffer full, new data will be lost; don't advance write index
//    error_occurred_fiq(ERROR_CAN1_RX_BUF_OF);  
//    C1CMR = 1<<2; //Release CAN data buffer
//    return;
//  }
////  ++can_rx_frame_count_1; // Count received frame
//
//  can_rx_buffer[temp_index].chan            = CHAN_CAN1;
//  can_rx_buffer[temp_index].addr            = C1RID;
//  can_rx_buffer[temp_index].dlc             = (C1RFS>>16)&0xF;
//  can_rx_buffer[temp_index].rtr             = (C1RFS>>30)&0x1;
//  can_rx_buffer[temp_index].payload.w.w1    = C1RDA;
//  can_rx_buffer[temp_index].payload.w.w2    = C1RDB;
//   
//  C1CMR = 1<<2; //Release CAN data buffer 
//
//  can_rx_index1 = temp_index;   //update index1
//}
//
//unsigned short can_rx_pop_frame(CAN_FRAME * frameptr)
//{
//  volatile short int temp_index1 = can_rx_index1;
//  volatile short int temp_index2 = can_rx_index2;
//
//  if (temp_index2 != temp_index1)
//  {
//    *frameptr = can_rx_buffer[temp_index2];
//    if (++temp_index2 >= CAN_RX_BUFFER_SIZE) 
//    {
//      temp_index2 = 0;
//    }
//    can_rx_index2 = temp_index2;  // Update read index
//    return 0; // Frame popped successfully
//  }
//  else
//  {
//    return 1; // Buffer empty
//  }
//}
//
//void can_rx_dispatch_all(void)
//{
//  CAN_FRAME frame;
//  while (!can_rx_pop_frame(&frame))
//  {
//    can_rx_dispatch_frame(&frame);
//  }
//}
//
//void can_rx_dispatch_frame(CAN_FRAME * frame){
//  CAN_FRAME_DESC  * frame_desc = 0;
//  CAN_LAYOUT        layout;
//  int               expected_dlc;
//  int               i;
//  CAN_FRAME_DESC ** search_list;
//  
//  search_list = (frame->rtr) ? can_rx_rtr_descriptors : can_rx_descriptors;
//  if(search_list == NULL) {
//    return; //Error, list not valid.
//  } 
//  i = 0;
//  while(1){
//    frame_desc = search_list[i];
//    if((frame_desc == 0) ||                               //if we have reached the end of the rx descriptor list, or
//      ((frame_desc->addr&0x7FF) == (frame->addr&0x7FF))) {//if we have found an address match
//      break;
//    }
//    i++;
//  }
//
//  if(frame_desc == 0) {return;} //Throw error.  This shouldn't have gotten through the acceptance filter
//  
//  //frame_desc points now to the descriptor for the received frame
//  //now, verify that the correct data length field value has been received  
//  layout = frame_desc->frame_layout;
//
//  if(frame->rtr) {
//    expected_dlc = 0;
//  } else {
//    switch(layout){
//      case CAN_LAYOUT_D:
//      case CAN_LAYOUT_FF:
//      case CAN_LAYOUT_II:
//      case CAN_LAYOUT_FI:
//  	  case CAN_LAYOUT_ISS:
//        expected_dlc =  8;
//        break;
//      default:
//        //this packet uses an unrecognized layout
//        expected_dlc = -1;
//        break;
//    };
//  }
//  if(expected_dlc == -1){
//    //handle unrecognized frame layout exception
//    return;
//  }
//  if((expected_dlc == 8) &&(frame->dlc < 8) ||
//     (expected_dlc != frame->dlc)) {
//    //handle dlc mismatch exception
//    return;
//  }
//
//  //dispatch payload contents per frame layout based on frame descriptor
//  if(frame->rtr == 0){
//    switch(layout){
//      case CAN_LAYOUT_D:
//        ((CAN_RX_SETTER_DOUBLE)frame_desc->ptr1)(frame->payload.d.d1);
//        break;
//      case CAN_LAYOUT_FF:
//        ((CAN_RX_SETTER_FLOAT)frame_desc->ptr1)(frame->payload.f.f1);
//        ((CAN_RX_SETTER_FLOAT)frame_desc->ptr2)(frame->payload.f.f2);
//        break;
//      case CAN_LAYOUT_II:
//        ((CAN_RX_SETTER_INT)frame_desc->ptr1)(frame->payload.i.i1);
//        ((CAN_RX_SETTER_INT)frame_desc->ptr2)(frame->payload.i.i2);
//        break;
//      case CAN_LAYOUT_FI:
//        ((CAN_RX_SETTER_FLOAT)frame_desc->ptr1)(frame->payload.f.f1);
//        ((CAN_RX_SETTER_INT)  frame_desc->ptr2)(frame->payload.i.i2);
//        break;
//  	  case CAN_LAYOUT_ISS:
//        ((CAN_RX_SETTER_INT  )frame_desc->ptr1)(frame->payload.i.i1);
//        ((CAN_RX_SETTER_SHORT)frame_desc->ptr2)(frame->payload.s.s3);
//        ((CAN_RX_SETTER_SHORT)frame_desc->ptr3)(frame->payload.s.s4);	  
//        break;	  
//      default:
//        //this packet uses an unrecognized layout, throw an error
//        break;
//    };
//  } else {
//    can_transmit(frame_desc);
//  }
//  //YAY!!!
//  //Now we're done, this frame has been fully dispatched!
//}
//
//void can_rx_acceptance_filter_init(void)
//{
//  unsigned short i, j, high_id;
//  unsigned long addr, chan, test_value;
//  CAN_FRAME_DESC  * frame_desc = 0;
//  volatile unsigned long * afram_ptr = &AFRAM;
//  
//  // Set to bypass mode before modifying acceptance filter registers and RAM
//  AFMR = 
//      (0 << 0)    // Set acceptance filter to off
//    | (1 << 1)    // Set acceptance filter to bypass mode
//    | (0 << 2);   // FullCAN is off
//    
// 
//  // Add CAN_IDs and channel numbers to acceptance filter RAM
//  // (In whatever order the rx frame descriptors are in)
//  i = 0;
//  j = 0;
//  high_id = 0;
//  while(1)
//  {
//    frame_desc = can_rx_descriptors[i];
//    
//    if (frame_desc != NULL)
//    {
//      if (!high_id)
//      {
//        addr = frame_desc->addr;
//        chan = frame_desc->chan;
//        
//        afram_ptr[j] = (addr << 16) | (chan - 1) << 29; // Add lower-index CAN_ID and channel number
//        high_id = 1;                                    // Wait for next iteration for higher-value CAN_ID
//      }
//      else  //high_id
//      {
//        addr = frame_desc->addr;
//        chan = frame_desc->chan;
//       
//        afram_ptr[j] |= (addr | ((chan - 1) << 13));           // Add higher-index CAN_ID and channel number
//        ++j;                                             // Increment AFRAM address index
//        if (j >= 0x200)
//        {
//          //AFRAM is full. Could add code to give an error if it was also not the last CAN_ID in the list.
//          break;
//        }
//        high_id = 0;                                // Start again with the low id
//      }
//      ++i;        // increment the descriptors counter
//    }
//    else if ((frame_desc == 0) && (i == 0)) // handle case of empty list 
//    {
//      break;
//    }
//    else  //End of list
//    {
//      if (high_id)
//      {
//        afram_ptr[j] |= 0x67FF; // Add dummy higher-value CAN_ID. This is the highest value possible if up to 4 CAN supported
//        j++;                    // Finish with j pointing to AFRAM location above table.          
//      }
//      break;
//    }    
//  }
//  
//  // Modified bubble sort of the AFRAM entries; they need to be in ascending order, including both channel and id.
//  // Count up until reaching a pair of values out of order, then switch them and start again from 0.
//  // Stop when the end of the list is reached with no pairs out of order.
//  i = 0;
//  while (1)
//  {
//    test_value = 0xFFFF & (afram_ptr[i >> 1] >> ((~i & 1) << 4));
//    if (((i+1) >> 1) >= j)
//    {
//      break;  //sorting done
//    }
//    else if (test_value > (0xFFFF & (afram_ptr[(i+1) >> 1] >> ((~(i+1) & 1) << 4))))
//    {
//      // Pair is out of order, swap values
//      if (i & 1)  // i is odd and i+1 is even - swap adjacent 16-bit words of successive 32-bit words
//      {
//        afram_ptr[i >> 1] &= 0xFFFF0000;      // Clear bottom 16 bits of lower 32-bit AFRAM word
//        afram_ptr[i >> 1] |=  afram_ptr[(i+1) >> 1] >> 16;  // Move the upper 16 bits of the upper 32-bit AFRAM word
//                                                            // down by 16, then move to the lower 32-bit AFRAM word
//        afram_ptr[(i+1) >> 1] &= 0xFFFF;      // Clear top 16 bits of upper 32-bit AFRAM word
//        afram_ptr[(i+1) >> 1] |= (test_value << 16);      // Move bottom 16 bits of lower 32-bit AFRAM word to the top 16
//                                                    // bits of the upper AFRAM word.
//        
//      }
//      else    // i is even and i+1 is odd - swap 16-bit words in same 32-bit word
//      {
//        afram_ptr[i >> 1] <<= 16;         // Move low-ID halfword to upper 16 bits
//        afram_ptr[i >> 1] |= test_value;  // Move the high-ID halfword into the lower 16 bits
//      } 
//      i = 0;  // search from start of array again
//    }
//    else if (test_value == (0xFFFF & (afram_ptr[(i+1) >> 1] >> (((i+1) & 1) << 4))))
//    {
//      //error - no two IDs should be identical. Annoying to clean up this case, however. For later if needed.
//      ++i;  // give error message and ignore. Some IDs may not work
//    }
//    else
//    {
//      ++i;  // not out of order yet - increment and check next pair
//    }
//  }
//  
//  
//  SFF_sa = 0;    //Set start index of standard individual identifiers in acceptance filter RAM
//  
//  SFF_GRP_sa = j << 2; //Set start index of standard group identifiers; also indicates end of preceding section
//  
//  EFF_sa = SFF_GRP_sa;     //Set start index of extended individual identifiers; also indicates end of preceding section
//  
//  EFF_GRP_sa = SFF_GRP_sa; //Set start index of extended group identifiers; also indicates end of preceding section
//  
//  ENDofTable = SFF_GRP_sa; //Address after last active element of acceptance filter. Max is 0x800.
//    
//  // Activate acceptance filter, once setup is done
//  AFMR = 
//      (0 << 0)    // Set acceptance filter to on
//    | (0 << 1)    // Set acceptance filter to normal mode
//    | (0 << 2);   // FullCAN is off
//    
//}

//// ************************************************************************************
//// CAN RING BUFFERS
//// ************************************************************************************
//void can_ring_init(CAN_RING * ring, CAN_FRAME * frame_buf, int buf_len){
//  ring->buf = frame_buf;
//  ring->buf_len = buf_len;
//  ring->in_idx = 0;
//  ring->out_idx = 0;
//}
//
//int can_ring_push(CAN_RING * ring, CAN_FRAME * frame) {
//  int next_in_idx = ring->in_idx;
//  if (++next_in_idx == ring->buf_len) {next_in_idx = 0;}
//  if (next_in_idx == ring->out_idx) {return 1;} //buffer full
//  ring->buf[next_in_idx] = *frame;
//  ring->in_idx = next_in_idx;
//  return 0;
//}
//
//int can_ring_pop(CAN_RING * ring, CAN_FRAME * frame){
//  if (ring->out_idx != ring->in_idx){
//    *frame = ring->buf[ring->out_idx];
//    if (ring->out_idx == (ring->buf_len) - 1){
//      ring->out_idx = 0;
//    } else {
//      ++ring->out_idx;
//    }
//    return 0;   //frame popped successfully
//  }
//  return 1;   //buffer empty
//}

//// ************************************************************************************
//// FRAME DESCRIPTOR FUNCTIONS
//// ************************************************************************************
//void   can_rx_setter_double_dummy(double d){ }
//void   can_rx_setter_float_dummy(float f){ }
//void   can_rx_setter_int_dummy(int i){ }
//void   can_rx_setter_short_dummy(short s){}
//double can_tx_getter_double_dummy(void) { return 0; }
//float  can_tx_getter_float_dummy(void) { return 0; }
//int    can_tx_getter_int_dummy(void) { return 0; }
//short  can_tx_getter_short_dummy(void) {return 0;}
////TX DESCRIPTOR SETTING FUNCTIONS
//void can_set_tx_descriptor_d(CAN_FRAME_DESC* frame_desc,int addr,CAN_CHANNEL chan,
//  CAN_TX_GETTER_DOUBLE g_d1
//  ){
//  frame_desc->addr = addr;
//  frame_desc->chan = chan;
//  frame_desc->rtr  = 0;
//  frame_desc->frame_layout = CAN_LAYOUT_D;
//  frame_desc->ptr1 = (CAN_VV_PTR)g_d1;
//}
//
//void can_set_tx_descriptor_ff(CAN_FRAME_DESC* frame_desc,int addr,CAN_CHANNEL chan,
//  CAN_TX_GETTER_FLOAT g_f1,CAN_TX_GETTER_FLOAT g_f2
//  ){
//  frame_desc->addr = addr;
//  frame_desc->chan = chan;
//  frame_desc->rtr  = 0;
//  frame_desc->frame_layout = CAN_LAYOUT_FF;
//  frame_desc->ptr1 = (CAN_VV_PTR)g_f1;
//  frame_desc->ptr2 = (CAN_VV_PTR)g_f2;
//}
//void can_set_tx_descriptor_ii(CAN_FRAME_DESC* frame_desc,int addr,CAN_CHANNEL chan,
//  CAN_TX_GETTER_INT g_i1,CAN_TX_GETTER_INT g_i2
//  ){
//  frame_desc->addr = addr;
//  frame_desc->chan = chan;
//  frame_desc->rtr  = 0;
//  frame_desc->frame_layout = CAN_LAYOUT_II;
//  frame_desc->ptr1 = (CAN_VV_PTR)g_i1;
//  frame_desc->ptr2 = (CAN_VV_PTR)g_i2;
//}
//
//void can_set_tx_descriptor_fi(CAN_FRAME_DESC* frame_desc,int addr,CAN_CHANNEL chan,
//  CAN_TX_GETTER_FLOAT g_f1,CAN_TX_GETTER_INT g_i1
//  ){
//  frame_desc->addr = addr;
//  frame_desc->chan = chan;
//  frame_desc->rtr  = 0;
//  frame_desc->frame_layout = CAN_LAYOUT_FI;
//  frame_desc->ptr1 = (CAN_VV_PTR)g_f1;
//  frame_desc->ptr2 = (CAN_VV_PTR)g_i1;
//}
//
//void can_set_tx_descriptor_iss(CAN_FRAME_DESC* frame_desc,int addr,CAN_CHANNEL chan,
//  CAN_TX_GETTER_INT g_i1,CAN_TX_GETTER_SHORT g_s1, CAN_TX_GETTER_SHORT g_s2
//  ){
//  frame_desc->addr = addr;
//  frame_desc->chan = chan;
//  frame_desc->rtr  = 0;
//  frame_desc->frame_layout = CAN_LAYOUT_ISS;
//  frame_desc->ptr1 = (CAN_VV_PTR)g_i1;
//  frame_desc->ptr2 = (CAN_VV_PTR)g_s1;
//  frame_desc->ptr3 = (CAN_VV_PTR)g_s2;
//}
//
//void can_set_tx_descriptor_rtr(CAN_FRAME_DESC* frame_desc,int addr,CAN_CHANNEL chan){
//  frame_desc->addr = addr;
//  frame_desc->chan = chan;
//  frame_desc->rtr  = 1;
//}
//
////RX DESCRIPTOR SETTING FUNCTIONS
//void can_set_rx_descriptor_d(CAN_FRAME_DESC* frame_desc,int addr,CAN_CHANNEL chan,
//  CAN_RX_SETTER_DOUBLE s_d1
//  ){
//  frame_desc->chan = chan;
//  frame_desc->addr = addr;
//  frame_desc->frame_layout = CAN_LAYOUT_D;
//  frame_desc->ptr1 = (CAN_VV_PTR)s_d1;  
//}
//void can_set_rx_descriptor_ff(CAN_FRAME_DESC* frame_desc,int addr,CAN_CHANNEL chan,
//  CAN_RX_SETTER_FLOAT s_f1,CAN_RX_SETTER_FLOAT s_f2
//  ){
//  frame_desc->chan = chan;
//  frame_desc->addr = addr;
//  frame_desc->frame_layout = CAN_LAYOUT_FF;
//  frame_desc->ptr1 = (CAN_VV_PTR)s_f1;
//  frame_desc->ptr2 = (CAN_VV_PTR)s_f2;
//}
//
//void can_set_rx_descriptor_ii(CAN_FRAME_DESC* frame_desc,int addr,CAN_CHANNEL chan,
//  CAN_RX_SETTER_INT s_i1,CAN_RX_SETTER_INT s_i2
//  ){
//  frame_desc->chan = chan;
//  frame_desc->addr = addr;
//  frame_desc->frame_layout = CAN_LAYOUT_II;
//  frame_desc->ptr1 = (CAN_VV_PTR)s_i1;
//  frame_desc->ptr2 = (CAN_VV_PTR)s_i2;
//}
//
//void can_set_rx_descriptor_fi(CAN_FRAME_DESC* frame_desc,int addr,CAN_CHANNEL chan,
//  CAN_RX_SETTER_FLOAT s_f1,CAN_RX_SETTER_INT s_i1
//  ){
//  frame_desc->chan = chan;
//  frame_desc->addr = addr;
//  frame_desc->frame_layout = CAN_LAYOUT_FI;
//  frame_desc->ptr1 = (CAN_VV_PTR)s_f1;
//  frame_desc->ptr2 = (CAN_VV_PTR)s_i1;
//}
//
//void can_set_rx_descriptor_iss(CAN_FRAME_DESC* frame_desc,int addr,CAN_CHANNEL chan,
//  CAN_RX_SETTER_INT s_i1,CAN_RX_SETTER_SHORT s_s1, CAN_RX_SETTER_SHORT s_s2
//  ){
//  frame_desc->chan = chan;
//  frame_desc->addr = addr;
//  frame_desc->frame_layout = CAN_LAYOUT_ISS;
//  frame_desc->ptr1 = (CAN_VV_PTR)s_i1;
//  frame_desc->ptr2 = (CAN_VV_PTR)s_s1;
//  frame_desc->ptr3 = (CAN_VV_PTR)s_s2;
//} 	 

//// ************************************************************************************
//// ERRORS
//// ************************************************************************************
////// CAN error global variables:
////volatile unsigned long can_error_1 = 0;
////volatile unsigned long can_error_2 = 0;
////volatile unsigned long can_error_3 = 0;
////volatile unsigned long can_error_4 = 0;
////unsigned long can_elapsed_ms;   // CAN module time variable
//// CAN error global variables:
//volatile unsigned long can_error_1 = 0;
//volatile unsigned long can_error_2 = 0;
//volatile unsigned long can_error_3 = 0;
//volatile unsigned long can_error_4 = 0;
//
//void can_error_isr(void) __irq
//{
// volatile int can_status1; //save state of capture register
// volatile int can_status2; //save state of capture register
// volatile int can_status3; //save state of capture register
// volatile int can_status4; //save state of capture register
// 
// volatile unsigned long can_acc_filter_error;   //save location of error in AF lookup table
//  
// can_status1 = C1ICR; //save state of capture register
// can_status2 = C2ICR; //save state of capture register
// can_status3 = C3ICR; //save state of capture register
// can_status4 = C4ICR; //save state of capture register
//
//  #ifdef DEBUG
//    if (FIO0PIN & 1<<14)
//    {
//      VICIntEnClr = 0xFFFFFFFF; //Disable all interrupts before debugging
//      VICVectAddr = 0;
//      return;
//    }
//  #endif
//
//  if (LUTerr & (1<<0))
//  {
//    can_acc_filter_error = LUTerrAd;  // Address in look-up table RAM (AF) at which error was encountered
//                                      // Reading this register clears LUTerr error flag
//    error_occurred_irq(ERROR_CAN_ACC_FILTER);  // Acceptance filter error - most likely a table error
//  }
//
//  if (C1GSR & (1<<7))
//  { //Bus-off due to too many transmit errors. Probably because someone flashed a board somewhere...
// //   C1MOD = 1; //reset the can bus
// //   C1MOD = 0; //restart the can bus
//    error_occurred_irq(ERROR_CAN1_TX_BUSOFF);  // Transmit errors leading to bus off error state
//    can_error_1 = ERROR_CAN1_TX_BUSOFF;
//    //can_tx_send_next_frame(CHAN_CAN1); //start the transmissions again
//    //VICSoftInt = 1<<20;   //Force CAN1 TX interrupt     // **** TEST CODE ****
//  }
//  if (C2GSR & (1<<7))
//  { //Bus-off due to too many transmit errors. Probably because someone flashed a board somewhere...
// //   C2MOD = 1; //reset the can bus
// //   C2MOD = 0; //restart the can bus
//    error_occurred_irq(ERROR_CAN2_TX_BUSOFF);  // Transmit errors leading to bus off error state
//    can_error_2 = ERROR_CAN2_TX_BUSOFF;
//   // can_tx_send_next_frame(CHAN_CAN2); //start the transmissions again
//    //VICSoftInt = 1<<21;   //Force CAN2 TX interrupt     // **** TEST CODE ****
//  }
//  if (C3GSR & (1<<7))
//  { //Bus-off due to too many transmit errors. Probably because someone flashed a board somewhere...
////    C3MOD = 1; //reset the can bus
////    C3MOD = 0; //restart the can bus
//    error_occurred_irq(ERROR_CAN3_TX_BUSOFF);  // Transmit errors leading to bus off error state
//    can_error_3 = ERROR_CAN3_TX_BUSOFF;
//    //can_tx_send_next_frame(CHAN_CAN3); //start the transmissions again
//    //VICSoftInt = 1<<22;   //Force CAN3 TX interrupt     // **** TEST CODE ****
//  }
//  if (C4GSR & (1<<7))
//  { //Bus-off due to too many transmit errors. Probably because someone flashed a board somewhere...
////    C4MOD = 1; //reset the can bus
////    C4MOD = 0; //restart the can bus
//    error_occurred_irq(ERROR_CAN4_TX_BUSOFF);  // Transmit errors leading to bus off error state
//    can_error_4 = ERROR_CAN4_TX_BUSOFF;
//   // can_tx_send_next_frame(CHAN_CAN4); //start the transmissions again
//   // VICSoftInt = 1<<23;   //Force CAN4 TX interrupt     // **** TEST CODE ****
//  }
//  
//  if (can_status1 & ((1<<2)|(1<<3)|(1<<5)|(1<<7)))
//  {
//    if (can_status1 & (1<<2))
//    {
//      can_error_1 = ERROR_CAN1_ERR_WARN;
//      error_occurred_irq(ERROR_CAN1_ERR_WARN);  // Shows change in error or bus status bits in either direction
//    }
//    if (can_status1 & (1<<3))
//    {
//      can_error_1 = ERROR_CAN1_DATA_OVRN;
//      error_occurred_irq(ERROR_CAN1_DATA_OVRN);  // Receive data overrun on CAN1 - receive buffer not read before
//                                                    // arrival of next received frame, data lost
//    }
//    if (can_status1 & (1<<5))
//    {
//      can_error_1 = ERROR_CAN1_ERR_PASS;
//      error_occurred_irq(ERROR_CAN1_ERR_PASS);  // Shows change in active or passive error status in either direction
//    }
//    if (can_status1 & (1<<7))
//    {
//      if (can_status1 & (1<<21))
//      {
//        can_error_1 = ERROR_CAN1_BUS_RX;
//        error_occurred_irq(ERROR_CAN1_BUS_RX);  // CAN1 has detected an rx bus error. Details of the error require reading
//                                                  // the other bits in can_status1. For now that has to be done in the debugger,
//                                                  // including all the possiblities in the ERROR_IDs would be cumbersome.
//                                                  // But we could make a non-error CAN_FRAME dedicated to the CAN error status register,
//      }                                           // if we wanted.
//      else
//      {
//         can_error_1 = ERROR_CAN1_BUS_TX;
//         error_occurred_irq(ERROR_CAN1_BUS_TX);
//      }
//    }
//  } 
//  if (can_status2 & (((1<<2)|(1<<3)|(1<<5)|(1<<7))&0x7FF))
//  {
//    if (can_status2 & (1<<2))
//    {
//      can_error_2 = ERROR_CAN2_ERR_WARN;
//      error_occurred_irq(ERROR_CAN2_ERR_WARN);  // Shows change in error or bus status bits in either direction
//    }
//    if (can_status2 & (1<<3))
//    {
//      can_error_2 = ERROR_CAN2_DATA_OVRN;
//      error_occurred_irq(ERROR_CAN2_DATA_OVRN);  // Receive data overrun on CAN2 - receive buffer not read before
//                                                    // arrival of next received frame, data lost
//    }
//    if (can_status2 & (1<<5))
//    {
//      can_error_2 = ERROR_CAN2_ERR_PASS;
//      error_occurred_irq(ERROR_CAN2_ERR_PASS);  // Shows change in active or passive error status in either direction
//    }
//    if (can_status2 & (1<<7))
//    {
//      if (can_status2 & (1<<21))
//      {
//        can_error_2 = ERROR_CAN2_BUS_RX;
//        error_occurred_irq(ERROR_CAN2_BUS_RX);  // CAN2 has detected an rx bus error. Details of the error require reading
//                                                  // the other bits in can_status1. For now that has to be done in the debugger,
//                                                  // including all the possiblities in the ERROR_IDs would be cumbersome.
//                                                  // But we could make a non-error CAN_FRAME dedicated to the CAN error status register,
//                                                  // if we wanted.
//      }
//      else
//      {
//        can_error_2 = ERROR_CAN2_BUS_TX;
//         error_occurred_irq(ERROR_CAN2_BUS_TX);
//      }
//    }
//  }
//  if (can_status3 & (((1<<2)|(1<<3)|(1<<5)|(1<<7))&0x7FF))
//  {
//    if (can_status3 & (1<<2))
//    {
//      can_error_3 = ERROR_CAN3_ERR_WARN;
//      error_occurred_irq(ERROR_CAN3_ERR_WARN);  // Shows change in error or bus status bits in either direction
//    }
//    if (can_status3 & (1<<3))
//    {
//      can_error_3 = ERROR_CAN3_DATA_OVRN;
//      error_occurred_irq(ERROR_CAN3_DATA_OVRN);  // Receive data overrun on CAN3 - receive buffer not read before
//                                                    // arrival of next received frame, data lost
//    }
//    if (can_status3 & (1<<5))
//    {
//      can_error_3 = ERROR_CAN3_ERR_PASS;
//      error_occurred_irq(ERROR_CAN3_ERR_PASS);  // Shows change in active or passive error status in either direction
//    }
//    if (can_status3 & (1<<7))
//    {
//      if (can_status3 & (1<<21))
//      {
//        can_error_3 = ERROR_CAN3_BUS_RX;
//        error_occurred_irq(ERROR_CAN3_BUS_RX);  // CAN3 has detected an rx bus error. Details of the error require reading
//                                                  // the other bits in can_status1. For now that has to be done in the debugger,
//                                                  // including all the possiblities in the ERROR_IDs would be cumbersome.
//                                                  // But we could make a non-error CAN_FRAME dedicated to the CAN error status register,
//                                                  // if we wanted.
//      }
//      else
//      {
//         can_error_3 = ERROR_CAN3_BUS_TX;
//         error_occurred_irq(ERROR_CAN3_BUS_TX);
//      }
//    }
//  }
//  if (can_status4 & (((1<<2)|(1<<3)|(1<<5)|(1<<7))&0x7FF))
//  {
//    if (can_status4 & (1<<2))
//    {
//      can_error_4 = ERROR_CAN4_ERR_WARN;
//      error_occurred_irq(ERROR_CAN4_ERR_WARN);  // Shows change in error or bus status bits in either direction
//    }
//    if (can_status4 & (1<<3))
//    {
//      can_error_4 = ERROR_CAN4_DATA_OVRN;
//      error_occurred_irq(ERROR_CAN4_DATA_OVRN);  // Receive data overrun on CAN4 - receive buffer not read before
//                                                    // arrival of next received frame, data lost
//    }
//    if (can_status4 & (1<<5))
//    {
//      can_error_4 = ERROR_CAN4_ERR_PASS;
//      error_occurred_irq(ERROR_CAN4_ERR_PASS);  // Shows change in active or passive error status in either direction
//    }
//    if (can_status4 & (1<<7))
//    {
//      if (can_status4 & (1<<21))
//      {
//        can_error_4 = ERROR_CAN4_BUS_RX;
//        error_occurred_irq(ERROR_CAN4_BUS_RX);  // CAN4 has detected an rx bus error. Details of the error require reading
//                                                  // the other bits in can_status1. For now that has to be done in the debugger,
//                                                  // including all the possiblities in the ERROR_IDs would be cumbersome.
//                                                  // But we could make a non-error CAN_FRAME dedicated to the CAN error status register,
//                                                  // if we wanted.
//      }
//      else
//      {
//        can_error_4 = ERROR_CAN4_BUS_TX;
//        error_occurred_irq(ERROR_CAN4_BUS_TX);
//      }
//    }
//  }
//
//  //To do? Put software interrupt force bits for all tx channels here? What about 2-channel MCUs vs. 4-channel?
//
//  VICVectAddr = 0;
//}

