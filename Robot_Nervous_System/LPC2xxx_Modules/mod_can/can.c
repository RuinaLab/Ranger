#include <includes.h>

unsigned long can_elapsed_ms = 0;

// ************************************************************************************
// TRANSMIT VARIABLES
// ************************************************************************************
CAN_TX_CHAN_CFG  can_tx_chan_cfgs[5];   //Five elements, index 1 to 4

// CAN transmit frame count global variables:
volatile unsigned short can_tx_frame_count_1 = 0;
volatile unsigned short can_tx_frame_count_2 = 0;
volatile unsigned short can_tx_frame_count_3 = 0;
volatile unsigned short can_tx_frame_count_4 = 0;

//Set up CAN transmit buffers
#ifndef CAN_TX_BUFFER_SIZE
  #define CAN_TX_BUFFER_SIZE  8
#endif
volatile CAN_FRAME can1_tx_buffer[CAN_TX_BUFFER_SIZE];
volatile short int can1_tx_index1 = 0;
volatile short int can1_tx_index2 = 0;

volatile CAN_FRAME can2_tx_buffer[CAN_TX_BUFFER_SIZE];
volatile short int can2_tx_index1 = 0;
volatile short int can2_tx_index2 = 0;

volatile CAN_FRAME can3_tx_buffer[CAN_TX_BUFFER_SIZE];
volatile short int can3_tx_index1 = 0;
volatile short int can3_tx_index2 = 0;

volatile CAN_FRAME can4_tx_buffer[CAN_TX_BUFFER_SIZE];
volatile short int can4_tx_index1 = 0;
volatile short int can4_tx_index2 = 0;



// ************************************************************************************
// RECEIVE VARIABLES
// ************************************************************************************
CAN_FRAME_DESC   ** can_rx_descriptors;
CAN_FRAME_DESC   ** can_rx_rtr_descriptors;
CAN_RX_CHAN_CFG     can_rx_chan_cfgs[5];

// CAN receive frame count global variables:
volatile unsigned short can_rx_frame_count_1 = 0;
volatile unsigned short can_rx_frame_count_2 = 0;
volatile unsigned short can_rx_frame_count_3 = 0;
volatile unsigned short can_rx_frame_count_4 = 0;

//Set up CAN receive buffer
#ifndef CAN_RX_BUFFER_SIZE
  #define CAN_RX_BUFFER_SIZE  64      //CAN buffer size must be a power of 2 >= than 4. E.g., 4, 8, 16, 32, 64
#endif
volatile CAN_RAW_FRAME can_rx_buffer[CAN_RX_BUFFER_SIZE];

//Define can_rx_data struct. Contains information for assembly code, and ring buffer indices.
volatile CAN_RX_DATA can_rx_data;

// ************************************************************************************
// ERROR VARIABLES
// ************************************************************************************
volatile unsigned long can_error_1 = 0;
volatile unsigned long can_error_2 = 0;
volatile unsigned long can_error_3 = 0;
volatile unsigned long can_error_4 = 0;

// ************************************************************************************
// INITIALIZATION FUNCTIONS
// ************************************************************************************

///////////////////////////////////////////////////////////////////////////////////////
//Initialize CAN buffers and structs as needed
void can_init(void)
{
  //Initialize can_rx_data struct
  can_rx_data.write_index = 1;                            //Initial buffer write index
  can_rx_data.read_index = 0;                             //Initial buffer read index
  can_rx_data.buffer_mask = CAN_RX_BUFFER_SIZE - 1;       //CAN buffer increment mask = 2^n - 1. E.g., 63
  can_rx_data.buffer_addr = (unsigned long)can_rx_buffer; //Pointer to base of can_rx_buffer
  can_rx_data.rfs_base_addr = 0xE0040020;                 //CANRFS base address 
                                                          //(add chan*0x4000 to get individual CANRFS base addresses, where chan = 1 to 4)
}
///////////////////////////////////////////////////////////////////////////////////////

// ************************************************************************************
// TRANSMIT FUNCTIONS
// ************************************************************************************

///////////////////////////////////////////////////////////////////////////////////////
//Push one CAN frame onto CAN1 transmit ring buffer
unsigned short int can_tx1_push_frame(CAN_FRAME * frameptr)
{
  short int ret = 1, temp_index = can1_tx_index1;
  
  if (++temp_index == CAN_TX_BUFFER_SIZE) {temp_index = 0;}

  if (temp_index != can1_tx_index2) 
  { 
    //Push new frame onto buffer
    can1_tx_buffer[can1_tx_index1] = *frameptr;
    can1_tx_index1 = temp_index;   //update index1
    ret = 0; //Frame successfully pushed onto buffer
  } 

  return ret; // 0 = successful push, 1 = buffer full
}
////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////
void can_tx1(void)
{
  while (can1_tx_index2 != can1_tx_index1)
  {
    if (C1SR & (1<<2)) // buffer 1 is available
    {              
      C1TFI1 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
      C1TID1 = can1_tx_buffer[can1_tx_index2].addr;   //get CAN_ID
      C1TDA1 = can1_tx_buffer[can1_tx_index2].payload.w.w1;   //load first data word
      C1TDB1 = can1_tx_buffer[can1_tx_index2].payload.w.w2;   //load second data word	
      C1CMR = (1<<0) | (1<<5);     // Select buffer 1, request transmission

      //Move to next read index location
      if (can1_tx_index2 >= CAN_TX_BUFFER_SIZE - 1)
      {
        can1_tx_index2 = 0;
      }
      else
      {
        ++can1_tx_index2;
      }

      can_tx_frame_count_1++;
    }
    else if (C1SR & (1<<10)) // buffer 2 is available
    {
      C1TFI2 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
      C1TID2 = can1_tx_buffer[can1_tx_index2].addr;   //get CAN_ID
      C1TDA2 = can1_tx_buffer[can1_tx_index2].payload.w.w1;   //load first data word
      C1TDB2 = can1_tx_buffer[can1_tx_index2].payload.w.w2;   //load second data word	
      C1CMR = (1<<0) | (1<<6);     // select buffer 2, request transmission

      //Move to next read index location
      if (can1_tx_index2 >= CAN_TX_BUFFER_SIZE - 1)
      {
        can1_tx_index2 = 0;
      }
      else
      {
        ++can1_tx_index2;
      }
      
      can_tx_frame_count_1++;
    }
    else
    {
      return;   //all available buffers full
    } 
  }
}
////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////
//Push one CAN frame onto CAN2 transmit ring buffer
unsigned short int can_tx2_push_frame(CAN_FRAME * frameptr)
{
  short int ret = 1, temp_index = can2_tx_index1;
  
  if (++temp_index == CAN_TX_BUFFER_SIZE) {temp_index = 0;}

  if (temp_index != can2_tx_index2) 
  {
    //Push new frame onto buffer
    can2_tx_buffer[can2_tx_index1] = *frameptr;
    can2_tx_index1 = temp_index;   //update index1
    ret = 0; //Frame successfully pushed onto buffer
  }
  
  return ret; // 0 = successful push, 1 = buffer full
}

////////////////////////////////////////////////////////////////////////////
void can_tx2(void)
{  
  while (can2_tx_index2 != can2_tx_index1)
  {
    if (C2SR & (1<<2)) // buffer 1 is available
    {   
      C2TFI1 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
      C2TID1 = can2_tx_buffer[can2_tx_index2].addr;   //get CAN_ID
      C2TDA1 = can2_tx_buffer[can2_tx_index2].payload.w.w1;   //load first data word
      C2TDB1 = can2_tx_buffer[can2_tx_index2].payload.w.w2;   //load second data word	
      C2CMR = (1<<0) | (1<<5);     //select buffer 1, request transmission
      can_tx_frame_count_2++;

      //Move to next read index location
      if (can2_tx_index2 >= CAN_TX_BUFFER_SIZE - 1)
      {
        can2_tx_index2 = 0;
      }
      else
      {
        ++can2_tx_index2;
      }
    }
    else if (C2SR & (1<<10)) // buffer 2 is available
    {
      C2TFI2 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
      C2TID2 = can2_tx_buffer[can2_tx_index2].addr;   //get CAN_ID
      C2TDA2 = can2_tx_buffer[can2_tx_index2].payload.w.w1;   //load first data word
      C2TDB2 = can2_tx_buffer[can2_tx_index2].payload.w.w2;   //load second data word	
      C2CMR = (1<<0) | (1<<6);     //select buffer 2, request transmission
      can_tx_frame_count_2++;

      //Move to next read index location
      if (can2_tx_index2 >= CAN_TX_BUFFER_SIZE - 1)
      {
        can2_tx_index2 = 0;
      }
      else
      {
        ++can2_tx_index2;
      }
    }
    else
    {
      return;   //all available buffers full
    } 
  }
}
////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////
//Push one CAN frame onto CAN3 transmit ring buffer
unsigned short int can_tx3_push_frame(CAN_FRAME * frameptr)
{
  short int ret = 1, temp_index = can3_tx_index1;
  
  if (++temp_index == CAN_TX_BUFFER_SIZE) {temp_index = 0;}

  if (temp_index != can3_tx_index2) 
  {
    //Push new frame onto buffer
    can3_tx_buffer[can3_tx_index1] = *frameptr;
    can3_tx_index1 = temp_index;   //update index1
    ret = 0; //Frame successfully pushed onto buffer
  }

  return ret; // 0 = successful push, 1 = buffer full
}

////////////////////////////////////////////////////////////////////////////
void can_tx3(void)
{
  while (can3_tx_index2 != can3_tx_index1)
  {
    if (C3SR & (1<<2)) // buffer 1 is available
    {   
      C3TFI1 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
      C3TID1 = can3_tx_buffer[can3_tx_index2].addr;   //get CAN_ID
      C3TDA1 = can3_tx_buffer[can3_tx_index2].payload.w.w1;   //load first data word
      C3TDB1 = can3_tx_buffer[can3_tx_index2].payload.w.w2;   //load second data word	
      C3CMR = (1<<0) | (1<<5);     // select buffer 1, request transmission
      can_tx_frame_count_3++;
 
      //Move to next read index location
      if (can3_tx_index2 >= CAN_TX_BUFFER_SIZE - 1)
      {
        can3_tx_index2 = 0;
      }
      else
      {
        ++can3_tx_index2;
      }
    }
    else if (C3SR & (1<<10)) // buffer 2 is available
    {
      C3TFI2 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
      C3TID2 = can3_tx_buffer[can3_tx_index2].addr;   //get CAN_ID
      C3TDA2 = can3_tx_buffer[can3_tx_index2].payload.w.w1;   //load first data word
      C3TDB2 = can3_tx_buffer[can3_tx_index2].payload.w.w2;   //load second data word	
      C3CMR = (1<<0) | (1<<6);     // select buffer 2, request transmission
      can_tx_frame_count_3++;

      //Move to next read index location
      if (can3_tx_index2 >= CAN_TX_BUFFER_SIZE - 1)
      {
        can3_tx_index2 = 0;
      }
      else
      {
        ++can3_tx_index2;
      }
    }
    else
    {
      return;   //all available buffers full
    } 
  }
}
////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////
//Push one CAN frame onto CAN4 transmit ring buffer
unsigned short int can_tx4_push_frame(CAN_FRAME * frameptr)
{
  short int ret = 1, temp_index = can4_tx_index1;
  
  if (++temp_index == CAN_TX_BUFFER_SIZE) {temp_index = 0;}

  if (temp_index != can4_tx_index2) 
  {
    //Push new frame onto buffer
    can4_tx_buffer[can4_tx_index1] = *frameptr;
    can4_tx_index1 = temp_index;   //update index1
    ret = 0; //Frame successfully pushed onto buffer
  }

  return ret; // 0 = successful push, 1 = buffer full
}


////////////////////////////////////////////////////////////////////////////
void can_tx4(void)
{
  while (can4_tx_index2 != can4_tx_index1)
  {
    if (C4SR & (1<<2)) // buffer 1 is available
    {   
      C4TFI1 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
      C4TID1 = can4_tx_buffer[can4_tx_index2].addr;   //get CAN_ID
      C4TDA1 = can4_tx_buffer[can4_tx_index2].payload.w.w1;   //load first data word
      C4TDB1 = can4_tx_buffer[can4_tx_index2].payload.w.w2;   //load second data word	
      C4CMR = (1<<0) | (1<<5);     // select buffer 1, request transmission
      can_tx_frame_count_4++;
 
      //Move to next read index location
      if (can4_tx_index2 >= CAN_TX_BUFFER_SIZE - 1)
      {
        can4_tx_index2 = 0;
      }
      else
      {
        ++can4_tx_index2;
      }
    }
    else if (C4SR & (1<<10)) // buffer 2 is available
    {
      C4TFI2 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
	    C4TID2 = can4_tx_buffer[can4_tx_index2].addr;   //get CAN_ID
      C4TDA2 = can4_tx_buffer[can4_tx_index2].payload.w.w1;   //load first data word
      C4TDB2 = can4_tx_buffer[can4_tx_index2].payload.w.w2;   //load second data word	
      C4CMR = (1<<0) | (1<<6);     // select buffer 2, request transmission
      can_tx_frame_count_4++;

      //Move to next read index location
      if (can4_tx_index2 >= CAN_TX_BUFFER_SIZE - 1)
      {
        can4_tx_index2 = 0;
      }
      else
      {
        ++can4_tx_index2;
      }
    }
    else
    {
      return;   //all available buffers full
    } 
  }
}
////////////////////////////////////////////////////////////////////////////


unsigned short can_tx_get_frame_count_1(CAN_FRAME * frame)
{
  frame->payload.w.w1 = can_tx_frame_count_1;
  can_tx_frame_count_1 = 0;     //Reset frame counter - could lose a count here, but not critical
  frame->payload.w.w2 = can_elapsed_ms;
  return 0;
}

unsigned short can_tx_get_frame_count_2(CAN_FRAME * frame)
{
  frame->payload.w.w1 = can_tx_frame_count_2;
  can_tx_frame_count_2 = 0;     //Reset frame counter - could lose a count here, but not critical
  frame->payload.w.w2 = can_elapsed_ms;
  return 0;
}

unsigned short can_tx_get_frame_count_3(CAN_FRAME * frame)
{
  frame->payload.w.w1 = can_tx_frame_count_3;
  can_tx_frame_count_3 = 0;     //Reset frame counter - could lose a count here, but not critical
  frame->payload.w.w2 = can_elapsed_ms;
  return 0;
}

unsigned short can_tx_get_frame_count_4(CAN_FRAME * frame)
{
  frame->payload.w.w1 = can_tx_frame_count_4;
  can_tx_frame_count_4 = 0;     //Reset frame counter - could lose a count here, but not critical
  frame->payload.w.w2 = can_elapsed_ms;
  return 0;
}

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
  // This may or may not be useful. ***************************************************************
  // So the chan argument does nothing at this point.
  frame.addr = frame_desc->addr;
  frame.chan = frame_desc->chan;
  frame.dlc  = dlc;
  frame.rtr  = rtr;
  
  //The frame is now fully populated with Address, Data Length, and Payload
  //and is therefore ready to be sent out.
  return can_transmit_frame(&frame);
}

// Push CAN_FRAME onto related CAN ring buffer
int can_transmit_frame(CAN_FRAME * frameptr)
{
  CAN_CHANNEL   chan      = frameptr->chan;
  int           ret = 1;

  if (chan == CHAN_CAN1)
  {
    ret = can_tx1_push_frame(frameptr);
    can_tx1();
  }
  else if (chan == CHAN_CAN2)
  {
    ret = can_tx2_push_frame(frameptr);
    can_tx2();
  }
  else if (chan == CHAN_CAN3)
  {
    ret = can_tx3_push_frame(frameptr);
    can_tx3();
  }
  else if (chan == CHAN_CAN4)
  {
    ret = can_tx4_push_frame(frameptr);
    can_tx4();
  }

  return ret; // ret = 1: buffer full; ret = 0: frame was successfully pushed onto buffer
}

// ************************************************************************************
// RECEIVE
// ************************************************************************************

__asm void can_rx_push(CAN_CHANNEL chan)
{
    import can_rx_data

    STMFD    SP!,{R8, R9}
    LDR R12, =can_rx_data
    LDMIA R12, {R1, R2, R3, R8, R9}
    
    ;R1 = write index
    ;R2 = read index
    ;R3 = buffer increment mask
    ;R8 = rx buffer address
    ;R9 = CANRFS base address

    CMP R1, R2          ;check for buffer full condition (R1 write index = R2 read index)
    BEQ write_cmr_bit   ;If equal, throw away incoming data frame, buffer is full

    ;calculate write location in rx buffer and update write index
    ADD R8, R8, R1, LSL #4      ;each raw frame has 4 32-bit words = 16 bytes
    ADD R1, #1                  ;Increment buffer write index
    AND R1, R3                  ;Mask to keep index in range
    STR R1, [R12]               ;Write back updated index value to can_rx_data

    ADD R9, R9, R0, LSL #14     ;Should now have the RFS address for the correct CAN channel in R9
    ;R1, R2, R3, and R12 are now free

    ;Read in rx data from selected CAN controller and save to ring buffer
    LDMIA R9, {R1, R2, R3, R12}
    BIC R1, #0x3800             ;Clear reserved bits 11 - 13 of RFS
    ORR R1, R1, R0, LSL #11     ;Put channel number in bits 11 - 13
    STMIA R8, {R1, R2, R3, R12}

write_cmr_bit
    MOV R0, #4
    STR R0, [R9, #-28]      ;write one to the Release Receive Buffer bit in CANxCMR to ack read and clear
   
    LDMFD SP!,{R8, R9}
    MOV PC, LR     
}

void can_rx_dispatch_all(void)
{
  CAN_FRAME frame;
  while (!can_rx_pop_frame(&frame))
  {
    can_rx_dispatch_frame(&frame);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned short can_rx_pop_frame(CAN_FRAME * frameptr)
{
  signed long temp_read_index;

  temp_read_index = (can_rx_data.read_index + 1) & (CAN_RX_BUFFER_SIZE - 1); //increment read index, mask to keep in range
  if (temp_read_index != can_rx_data.write_index)
  { 
    frameptr->chan = (CAN_CHANNEL)((can_rx_buffer[temp_read_index].canrfs >> 11) & 7);  //Retrieve channel number from rfs field
    frameptr->addr = can_rx_buffer[temp_read_index].canrid & 0x7FF;                   //CAN ID, with reserved bits masked off
    frameptr->dlc = (can_rx_buffer[temp_read_index].canrfs >> 16) & 15;               //Obtain data length code (DLC) from CANRFS register value
    frameptr->rtr = (can_rx_buffer[temp_read_index].canrfs >> 30) & 1;                //Obtain RTR code from CANRFS
    frameptr->payload.w.w1 = can_rx_buffer[temp_read_index].canrda;                   //First four data bytes
    frameptr->payload.w.w2 = can_rx_buffer[temp_read_index].canrdb;                   //second four data bytes

    can_rx_data.read_index = temp_read_index;                                         //update read index
    return 0;
  }
  else
  {
    return 1;   //No data available in buffer
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
unsigned short can_rx_pop_frame(CAN_FRAME * frameptr)
{
  volatile short int temp_index1 = can_rx_index1;
  volatile short int temp_index2 = can_rx_index2;

  if (temp_index2 != temp_index1)
  {
    *frameptr = can_rx_buffer[temp_index2];
    if (++temp_index2 >= CAN_RX_BUFFER_SIZE) 
    {
      temp_index2 = 0;
    }
    can_rx_index2 = temp_index2;  // Update read index
    return 0; // Frame popped successfully
  }
  else
  {
    return 1; // Buffer empty
  }
}
*/
/*
////////////////////////////////////////////////////////////////////////////
//Read CAN rx controller buffer, create CAN frame and push onto receive ring buffer
void can_rx1_fiq(void)
{
  short int temp_index = can_rx_index1; 

  #ifdef DEBUG
  if (FIO0PIN & 1<<14)
  {
    VICIntEnClr = 0xFFFFFFFF; //Disable all interrupts before debugging
    return;
  }
  #endif

  if (++temp_index == CAN_RX_BUFFER_SIZE) {temp_index = 0;}
  if (temp_index == can_rx_index2)
  {
    //Buffer full, new data will be lost; don't advance write index
    error_occurred_fiq(ERROR_CAN1_RX_BUF_OF);  
    C1CMR = 1<<2; //Release CAN data buffer
    return;
  }
  ++can_rx_frame_count_1; // Count received frame
 
  can_rx_buffer[temp_index].chan            = CHAN_CAN1;
  can_rx_buffer[temp_index].addr            = C1RID;
  can_rx_buffer[temp_index].dlc             = (C1RFS>>16)&0xF;
  can_rx_buffer[temp_index].rtr             = (C1RFS>>30)&0x1;
  can_rx_buffer[temp_index].payload.w.w1    = C1RDA;
  can_rx_buffer[temp_index].payload.w.w2    = C1RDB;
   
  C1CMR = 1<<2; //Release CAN data buffer 

  can_rx_index1 = temp_index;   //update index1

}
////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////
//Read CAN rx controller buffer, create CAN frame and push onto receive ring buffer
void can_rx2_fiq(void)
{
  short int temp_index = can_rx_index1;

  #ifdef DEBUG
  if (FIO0PIN & 1<<14)
  {
    VICIntEnClr = 0xFFFFFFFF; //Disable all interrupts before debugging
    return;
  }
  #endif

  if (++temp_index == CAN_RX_BUFFER_SIZE) {temp_index = 0;}
  if (temp_index == can_rx_index2)
  {
    //Buffer full, new data will be lost; don't advance write index
    error_occurred_fiq(ERROR_CAN2_RX_BUF_OF);  
    C2CMR = 1<<2; //Release CAN data buffer
    return;
  }
  ++can_rx_frame_count_2; // Count received frame

  can_rx_buffer[temp_index].chan            = CHAN_CAN2;
  can_rx_buffer[temp_index].addr            = C2RID;
  can_rx_buffer[temp_index].dlc             = (C2RFS>>16)&0xF;
  can_rx_buffer[temp_index].rtr             = (C2RFS>>30)&0x1;
  can_rx_buffer[temp_index].payload.w.w1    = C2RDA;
  can_rx_buffer[temp_index].payload.w.w2    = C2RDB; 

  C2CMR = 1<<2; //Release CAN data buffer 
   
  can_rx_index1 = temp_index;   //update index1
}
////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////
//Read CAN rx controller buffer, create CAN frame and push onto receive ring buffer
void can_rx3_fiq(void)
{
  short int temp_index = can_rx_index1;

  #ifdef DEBUG
  if (FIO0PIN & 1<<14)
  {
    VICIntEnClr = 0xFFFFFFFF; //Disable all interrupts before debugging
    return;
  }
  #endif

  if (++temp_index == CAN_RX_BUFFER_SIZE) {temp_index = 0;}
  if (temp_index == can_rx_index2)
  {
    //Buffer full, new data will be lost; don't advance write index
    error_occurred_fiq(ERROR_CAN3_RX_BUF_OF);  
    C3CMR = 1<<2; //Release CAN data buffer
    return;
  }

  ++can_rx_frame_count_3; // Count received frame

  can_rx_buffer[temp_index].chan            = CHAN_CAN3;
  can_rx_buffer[temp_index].addr            = C3RID;
  can_rx_buffer[temp_index].dlc             = (C3RFS>>16)&0xF;
  can_rx_buffer[temp_index].rtr             = (C3RFS>>30)&0x1;
  can_rx_buffer[temp_index].payload.w.w1    = C3RDA;
  can_rx_buffer[temp_index].payload.w.w2    = C1RDB;
  
  C3CMR = 1<<2; //Release CAN data buffer 
   
  can_rx_index1 = temp_index;   //update index1
}
////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////
//Read CAN rx controller buffer, create CAN frame and push onto receive ring buffer
void can_rx4_fiq(void)
{
  short int temp_index = can_rx_index1;

  #ifdef DEBUG
  if (FIO0PIN & 1<<14)
  {
    VICIntEnClr = 0xFFFFFFFF; //Disable all interrupts before debugging
    return;
  }
  #endif

  if (++temp_index == CAN_RX_BUFFER_SIZE) {temp_index = 0;}
  if (temp_index == can_rx_index2)
  {
    //Buffer full, new data will be lost; don't advance write index
    error_occurred_fiq(ERROR_CAN4_RX_BUF_OF);  
    C4CMR = 1<<2; //Release CAN data buffer
    return;
  }

  ++can_rx_frame_count_4; // Count received frame

  can_rx_buffer[temp_index].chan            = CHAN_CAN4;
  can_rx_buffer[temp_index].addr            = C4RID;
  can_rx_buffer[temp_index].dlc             = (C4RFS>>16)&0xF;
  can_rx_buffer[temp_index].rtr             = (C4RFS>>30)&0x1;
  can_rx_buffer[temp_index].payload.w.w1    = C4RDA;
  can_rx_buffer[temp_index].payload.w.w2    = C4RDB;
  
  C4CMR = 1<<2; //Release CAN data buffer 
   
  can_rx_index1 = temp_index;   //update index1
}
////////////////////////////////////////////////////////////////////////////
*/

////////////////////////////////////////////////////////////////////////////
//Read CAN rx controller buffer, create CAN frame and push onto receive ring buffer
void can_rx1_isr(void) __irq
{
  #ifdef DEBUG
  if (FIO0PIN & 1<<14)
  {
    VICIntEnClr = 0xFFFFFFFF; //Disable all interrupts before debugging
    VICVectAddr = 0;
    return;
  }
  #endif
  
  can_rx_push((CAN_CHANNEL)1);          //Call assembly routine to save frame
  ++can_rx_frame_count_1;  //Count received frame

  VICVectAddr = 0;         //Clear interrupt in VIC.
}
////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////
//Read CAN rx controller buffer, create CAN frame and push onto receive ring buffer
void can_rx2_isr(void) __irq
{
  #ifdef DEBUG
  if (FIO0PIN & 1<<14)
  {
    VICIntEnClr = 0xFFFFFFFF; //Disable all interrupts before debugging
    VICVectAddr = 0;
    return;
  }
  #endif
  
  can_rx_push((CAN_CHANNEL)2);          //Call assembly routine to save frame
  ++can_rx_frame_count_2;  //Count received frame

  VICVectAddr = 0;         //Clear interrupt in VIC.
}
////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////
//Read CAN rx controller buffer, create CAN frame and push onto receive ring buffer
void can_rx3_isr(void) __irq
{
  #ifdef DEBUG
  if (FIO0PIN & 1<<14)
  {
    VICIntEnClr = 0xFFFFFFFF; //Disable all interrupts before debugging
    VICVectAddr = 0;
    return;
  }
  #endif
  
  can_rx_push((CAN_CHANNEL)3);          //Call assembly routine to save frame
  ++can_rx_frame_count_3;  //Count received frame

  VICVectAddr = 0;         //Clear interrupt in VIC.
}
////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////
//Read CAN rx controller buffer, create CAN frame and push onto receive ring buffer
void can_rx4_isr(void) __irq
{
  #ifdef DEBUG
  if (FIO0PIN & 1<<14)
  {
    VICIntEnClr = 0xFFFFFFFF; //Disable all interrupts before debugging
    VICVectAddr = 0;
    return;
  }
  #endif
  
  can_rx_push((CAN_CHANNEL)4);          //Call assembly routine to save frame
  ++can_rx_frame_count_4;  //Count received frame

  VICVectAddr = 0;         //Clear interrupt in VIC.
}
////////////////////////////////////////////////////////////////////////////

unsigned short can_rx_get_frame_count_1(CAN_FRAME * frame)
{
  frame->payload.w.w1 = can_rx_frame_count_1;
  can_rx_frame_count_1 = 0;     //Reset frame counter - could lose a count here, but not critical
  frame->payload.w.w2 = can_elapsed_ms;
  return 0;
}

unsigned short can_rx_get_frame_count_2(CAN_FRAME * frame)
{
  frame->payload.w.w1 = can_rx_frame_count_2;
  can_rx_frame_count_2 = 0;     //Reset frame counter - could lose a count here, but not critical
  frame->payload.w.w2 = can_elapsed_ms;
  return 0;
}

unsigned short can_rx_get_frame_count_3(CAN_FRAME * frame)
{
  frame->payload.w.w1 = can_rx_frame_count_3;
  can_rx_frame_count_3 = 0;     //Reset frame counter - could lose a count here, but not critical
  frame->payload.w.w2 = can_elapsed_ms;
  return 0;
}

unsigned short can_rx_get_frame_count_4(CAN_FRAME * frame)
{
  frame->payload.w.w1 = can_rx_frame_count_4;
  can_rx_frame_count_4 = 0;     //Reset frame counter - could lose a count here, but not critical
  frame->payload.w.w2 = can_elapsed_ms;
  return 0;
}

void can_rx_set_descriptors(CAN_FRAME_DESC ** rx_descriptors,CAN_FRAME_DESC ** rtr_descriptors){
  can_rx_descriptors     = rx_descriptors;
  can_rx_rtr_descriptors = rtr_descriptors;
//  can_rx_acceptance_filter_init();  Put acceptance filter config here? And RTR-subscribe? Or keep it separate in software_setup.c?
}

void can_rx_set_chan_cfg(CAN_CHANNEL chan,volatile unsigned long * base_addr, CAN_RING * rx_ring, CAN_DISPATCH_MODE mode){
  CAN_RX_CHAN_CFG * cfg = &can_rx_chan_cfgs[chan];
  
  cfg->base_addr     = base_addr;
  cfg->dispatch_mode = mode;
  cfg->ring          = rx_ring; 
}

void can_rx_dispatch_frame(CAN_FRAME * frame){
  CAN_FRAME_DESC  * frame_desc = 0;
  CAN_LAYOUT        layout;
  int               expected_dlc;
  int               i;
  CAN_FRAME_DESC ** search_list;

 // static unsigned long old_time = 0;
  
  search_list = (frame->rtr) ? can_rx_rtr_descriptors : can_rx_descriptors;
  
  if(search_list == NULL) {
    return; //Error, list not valid.
  }
  
  i = 0;
  while(1){
    frame_desc = search_list[i];
    if((frame_desc == 0) ||                               //if we have reached the end of the rx descriptor list, or
      ((frame_desc->addr&0x7FF) == (frame->addr&0x7FF))) {//if we have found an address match
      break;
    }
    i++;
  }

  if(frame_desc == 0) {
    //Throw error.  This shouldn't have gotten through the acceptance filter
    return;
  }
  //frame_desc points now to the descriptor for the received frame

  //now, verify that the correct data length field value has been received  
  layout = frame_desc->frame_layout;

  if(frame->rtr) {
    expected_dlc = 0;
  } else {
    switch(layout){
      case CAN_LAYOUT_D:
      case CAN_LAYOUT_FF:
      case CAN_LAYOUT_II:
      case CAN_LAYOUT_FI:
  	  case CAN_LAYOUT_ISS:
        expected_dlc =  8;
        break;
      default:
        //this packet uses an unrecognized layout
        expected_dlc = -1;
        break;
    };
  }
  if(expected_dlc == -1){
    //handle unrecognized frame layout exception
    return;
  }
  if((expected_dlc == 8) &&(frame->dlc < 8) ||
     (expected_dlc != frame->dlc)) {
    //handle dlc mismatch exception
    return;
  }

  /*  Test code - check for missing timestamps
    if (frame->addr == 0)
    {
      mcu_led_green_blink(5);
      if (frame->payload.w.w2 != old_time + 1)
      {
        mcu_led_red_blink(50);
      }
      old_time = frame->payload.w.w2;
    }
    */

  //dispatch payload contents per frame layout based on frame descriptor
  if(frame->rtr == 0){
    switch(layout){
      case CAN_LAYOUT_D:
        ((CAN_RX_SETTER_DOUBLE)frame_desc->ptr1)(frame->payload.d.d1);
        break;
      case CAN_LAYOUT_FF:
        ((CAN_RX_SETTER_FLOAT)frame_desc->ptr1)(frame->payload.f.f1);
        ((CAN_RX_SETTER_FLOAT)frame_desc->ptr2)(frame->payload.f.f2);
        break;
      case CAN_LAYOUT_II:
        ((CAN_RX_SETTER_INT)frame_desc->ptr1)(frame->payload.i.i1);
        ((CAN_RX_SETTER_INT)frame_desc->ptr2)(frame->payload.i.i2);
        break;
      case CAN_LAYOUT_FI:
        ((CAN_RX_SETTER_FLOAT)frame_desc->ptr1)(frame->payload.f.f1);
        ((CAN_RX_SETTER_INT)  frame_desc->ptr2)(frame->payload.i.i2);
        break;
  	  case CAN_LAYOUT_ISS:
        ((CAN_RX_SETTER_INT  )frame_desc->ptr1)(frame->payload.i.i1);
        ((CAN_RX_SETTER_SHORT)frame_desc->ptr2)(frame->payload.s.s3);
        ((CAN_RX_SETTER_SHORT)frame_desc->ptr3)(frame->payload.s.s4);	  
        break;	  
      default:
        //this packet uses an unrecognized layout, throw an error
        break;
    };
  } else {
    can_transmit(frame_desc);
  }
  //YAY!!!
  //Now we're done, this frame has been fully dispatched!
}

// ************************************************************************************
// ERROR HANDLING
// ************************************************************************************

unsigned short can_get_error_1(CAN_FRAME * frame)
{
  frame->payload.w.w1 = can_error_1;
  can_error_1 = 0;     //Reset error
  frame->payload.w.w2 = can_elapsed_ms;
  return 0;
}

unsigned short can_get_error_2(CAN_FRAME * frame)
{
  frame->payload.w.w1 = can_error_2;
  can_error_2 = 0;     //Reset error
  frame->payload.w.w2 = can_elapsed_ms;
  return 0;
}

unsigned short can_get_error_3(CAN_FRAME * frame)
{
  frame->payload.w.w1 = can_error_3;
  can_error_3 = 0;     //Reset error
  frame->payload.w.w2 = can_elapsed_ms;
  return 0;
}

unsigned short can_get_error_4(CAN_FRAME * frame)
{
  frame->payload.w.w1 = can_error_4;
  can_error_4 = 0;     //Reset error
  frame->payload.w.w2 = can_elapsed_ms;
  return 0;
}

void can_error_isr(void) __irq
{
 volatile int can_status1; //save state of capture register
 volatile int can_status2; //save state of capture register
 volatile int can_status3; //save state of capture register
 volatile int can_status4; //save state of capture register
 
 volatile unsigned long can_acc_filter_error;   //save location of error in AF lookup table
  
  #ifdef DEBUG
    if (FIO0PIN & 1<<14)
    {
      VICIntEnClr = 0xFFFFFFFF; //Disable all interrupts before debugging
      VICVectAddr = 0;
      return;
    }
  #endif

  if (LUTerr & (1<<0))
  {
    can_acc_filter_error = LUTerrAd;  // Address in look-up table RAM (AF) at which error was encountered
                                      // Reading this register clears LUTerr error flag
    error_occurred_irq(ERROR_CAN_ACC_FILTER);  // Acceptance filter error - most likely a table error
  }

#ifdef USE_CAN1
  if (C1GSR & (1<<7))
  { //Bus-off due to too many transmit errors. Probably because someone flashed a board somewhere...
    C1MOD = 0; //restart the can bus
    error_occurred_irq(ERROR_CAN1_TX_BUSOFF);  // Transmit errors leading to bus off error state
    can_error_1 = ERROR_CAN1_TX_BUSOFF;
    //can_tx_send_next_frame(CHAN_CAN1); //start the transmissions again
    //VICSoftInt = 1<<20;   //Force CAN1 TX interrupt     // **** TEST CODE ****
  }
  can_status1 = C1ICR; //save state of capture register
    if (can_status1 & ((1<<2)|(1<<3)|(1<<5)|(1<<7)))
  {
    if (can_status1 & (1<<2))
    {
      can_error_1 = ERROR_CAN1_ERR_WARN;
      error_occurred_irq(ERROR_CAN1_ERR_WARN);  // Shows change in error or bus status bits in either direction
    }
    if (can_status1 & (1<<3))
    {
      can_error_1 = ERROR_CAN1_DATA_OVRN;
      error_occurred_irq(ERROR_CAN1_DATA_OVRN);  // Receive data overrun on CAN1 - receive buffer not read before
                                                    // arrival of next received frame, data lost
    }
    if (can_status1 & (1<<5))
    {
      can_error_1 = ERROR_CAN1_ERR_PASS;
      error_occurred_irq(ERROR_CAN1_ERR_PASS);  // Shows change in active or passive error status in either direction
    }
    if (can_status1 & (1<<7))
    {
      if (can_status1 & (1<<21))
      {
        can_error_1 = ERROR_CAN1_BUS_RX;
        error_occurred_irq(ERROR_CAN1_BUS_RX);  // CAN1 has detected an rx bus error. Details of the error require reading
                                                  // the other bits in can_status1. For now that has to be done in the debugger,
                                                  // including all the possiblities in the ERROR_IDs would be cumbersome.
                                                  // But we could make a non-error CAN_FRAME dedicated to the CAN error status register,
      }                                           // if we wanted.
      else
      {
         can_error_1 = ERROR_CAN1_BUS_TX;
         error_occurred_irq(ERROR_CAN1_BUS_TX);
      }
    }
  } 
#endif

#ifdef USE_CAN2
  if (C2GSR & (1<<7))
  { //Bus-off due to too many transmit errors. Probably because someone flashed a board somewhere...
    C2MOD = 0; //restart the can bus
    error_occurred_irq(ERROR_CAN2_TX_BUSOFF);  // Transmit errors leading to bus off error state
    can_error_2 = ERROR_CAN2_TX_BUSOFF;
   // can_tx_send_next_frame(CHAN_CAN2); //start the transmissions again
    //VICSoftInt = 1<<21;   //Force CAN2 TX interrupt     // **** TEST CODE ****
  }
  can_status2 = C2ICR; //save state of capture register
  if (can_status2 & (((1<<2)|(1<<3)|(1<<5)|(1<<7))&0x7FF))
  {
    if (can_status2 & (1<<2))
    {
      can_error_2 = ERROR_CAN2_ERR_WARN;
      error_occurred_irq(ERROR_CAN2_ERR_WARN);  // Shows change in error or bus status bits in either direction
    }
    if (can_status2 & (1<<3))
    {
      can_error_2 = ERROR_CAN2_DATA_OVRN;
      error_occurred_irq(ERROR_CAN2_DATA_OVRN);  // Receive data overrun on CAN2 - receive buffer not read before
                                                    // arrival of next received frame, data lost
    }
    if (can_status2 & (1<<5))
    {
      can_error_2 = ERROR_CAN2_ERR_PASS;
      error_occurred_irq(ERROR_CAN2_ERR_PASS);  // Shows change in active or passive error status in either direction
    }
    if (can_status2 & (1<<7))
    {
      if (can_status2 & (1<<21))
      {
        can_error_2 = ERROR_CAN2_BUS_RX;
        error_occurred_irq(ERROR_CAN2_BUS_RX);  // CAN2 has detected an rx bus error. Details of the error require reading
                                                  // the other bits in can_status1. For now that has to be done in the debugger,
                                                  // including all the possiblities in the ERROR_IDs would be cumbersome.
                                                  // But we could make a non-error CAN_FRAME dedicated to the CAN error status register,
                                                  // if we wanted.
      }
      else
      {
        can_error_2 = ERROR_CAN2_BUS_TX;
         error_occurred_irq(ERROR_CAN2_BUS_TX);
      }
    }
  }
#endif

#ifdef USE_CAN3
  if (C3GSR & (1<<7))
  { //Bus-off due to too many transmit errors. Probably because someone flashed a board somewhere...
    C3MOD = 0; //restart the can bus
    error_occurred_irq(ERROR_CAN3_TX_BUSOFF);  // Transmit errors leading to bus off error state
    can_error_3 = ERROR_CAN3_TX_BUSOFF;
    //can_tx_send_next_frame(CHAN_CAN3); //start the transmissions again
    //VICSoftInt = 1<<22;   //Force CAN3 TX interrupt     // **** TEST CODE ****
  } 
  can_status3 = C3ICR; //save state of capture register
  if (can_status3 & (((1<<2)|(1<<3)|(1<<5)|(1<<7))&0x7FF))
  {
    if (can_status3 & (1<<2))
    {
      can_error_3 = ERROR_CAN3_ERR_WARN;
      error_occurred_irq(ERROR_CAN3_ERR_WARN);  // Shows change in error or bus status bits in either direction
    }
    if (can_status3 & (1<<3))
    {
      can_error_3 = ERROR_CAN3_DATA_OVRN;
      error_occurred_irq(ERROR_CAN3_DATA_OVRN);  // Receive data overrun on CAN3 - receive buffer not read before
                                                    // arrival of next received frame, data lost
    }
    if (can_status3 & (1<<5))
    {
      can_error_3 = ERROR_CAN3_ERR_PASS;
      error_occurred_irq(ERROR_CAN3_ERR_PASS);  // Shows change in active or passive error status in either direction
    }
    if (can_status3 & (1<<7))
    {
      if (can_status3 & (1<<21))
      {
        can_error_3 = ERROR_CAN3_BUS_RX;
        error_occurred_irq(ERROR_CAN3_BUS_RX);  // CAN3 has detected an rx bus error. Details of the error require reading
                                                  // the other bits in can_status1. For now that has to be done in the debugger,
                                                  // including all the possiblities in the ERROR_IDs would be cumbersome.
                                                  // But we could make a non-error CAN_FRAME dedicated to the CAN error status register,
                                                  // if we wanted.
      }
      else
      {
         can_error_3 = ERROR_CAN3_BUS_TX;
         error_occurred_irq(ERROR_CAN3_BUS_TX);
      }
    }
  }
#endif

#ifdef USE_CAN4
  if (C4GSR & (1<<7))
  { //Bus-off due to too many transmit errors. Probably because someone flashed a board somewhere...
    C4MOD = 0; //restart the can bus
    error_occurred_irq(ERROR_CAN4_TX_BUSOFF);  // Transmit errors leading to bus off error state
    can_error_4 = ERROR_CAN4_TX_BUSOFF;
   // can_tx_send_next_frame(CHAN_CAN4); //start the transmissions again
   // VICSoftInt = 1<<23;   //Force CAN4 TX interrupt     // **** TEST CODE ****
  }
  can_status4 = C4ICR; //save state of capture register 
  
  if (can_status4 & (((1<<2)|(1<<3)|(1<<5)|(1<<7))&0x7FF))
  {
    if (can_status4 & (1<<2))
    {
      can_error_4 = ERROR_CAN4_ERR_WARN;
      error_occurred_irq(ERROR_CAN4_ERR_WARN);  // Shows change in error or bus status bits in either direction
    }
    if (can_status4 & (1<<3))
    {
      can_error_4 = ERROR_CAN4_DATA_OVRN;
      error_occurred_irq(ERROR_CAN4_DATA_OVRN);  // Receive data overrun on CAN4 - receive buffer not read before
                                                    // arrival of next received frame, data lost
    }
    if (can_status4 & (1<<5))
    {
      can_error_4 = ERROR_CAN4_ERR_PASS;
      error_occurred_irq(ERROR_CAN4_ERR_PASS);  // Shows change in active or passive error status in either direction
    }
    if (can_status4 & (1<<7))
    {
      if (can_status4 & (1<<21))
      {
        can_error_4 = ERROR_CAN4_BUS_RX;
        error_occurred_irq(ERROR_CAN4_BUS_RX);  // CAN4 has detected an rx bus error. Details of the error require reading
                                                  // the other bits in can_status1. For now that has to be done in the debugger,
                                                  // including all the possiblities in the ERROR_IDs would be cumbersome.
                                                  // But we could make a non-error CAN_FRAME dedicated to the CAN error status register,
                                                  // if we wanted.
      }
      else
      {
        can_error_4 = ERROR_CAN4_BUS_TX;
        error_occurred_irq(ERROR_CAN4_BUS_TX);
      }
    }
  }
 #endif
  //To do? Put software interrupt force bits for all tx channels here? What about 2-channel MCUs vs. 4-channel?

  VICVectAddr = 0;
}
