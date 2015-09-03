#include <includes.h>

// CAN error global variables:
volatile unsigned long can_error_1 = 0;
volatile unsigned long can_error_2 = 0;
volatile unsigned long can_error_3 = 0;
volatile unsigned long can_error_4 = 0;

// CAN receive frame count global variables:
volatile unsigned short can_rx_frame_count_1 = 0;
volatile unsigned short can_rx_frame_count_2 = 0;
volatile unsigned short can_rx_frame_count_3 = 0;
volatile unsigned short can_rx_frame_count_4 = 0;

// CAN transmit frame count global variables:
volatile unsigned short can_tx_frame_count_1 = 0;
volatile unsigned short can_tx_frame_count_2 = 0;
volatile unsigned short can_tx_frame_count_3 = 0;
volatile unsigned short can_tx_frame_count_4 = 0;

//Set up CAN transmit buffers (version 2)
#define CSR_CAN_TX_BUFFER_SIZE  8
volatile CAN_FRAME csr_can1_tx_buffer[CSR_CAN_TX_BUFFER_SIZE];
volatile short int csr_can1_tx_index1 = 0;
volatile short int csr_can1_tx_index2 = 0;

volatile CAN_FRAME csr_can2_tx_buffer[CSR_CAN_TX_BUFFER_SIZE];
volatile short int csr_can2_tx_index1 = 0;
volatile short int csr_can2_tx_index2 = 0;

volatile CAN_FRAME csr_can3_tx_buffer[CSR_CAN_TX_BUFFER_SIZE];
volatile short int csr_can3_tx_index1 = 0;
volatile short int csr_can3_tx_index2 = 0;

volatile CAN_FRAME csr_can4_tx_buffer[CSR_CAN_TX_BUFFER_SIZE];
volatile short int csr_can4_tx_index1 = 0;
volatile short int csr_can4_tx_index2 = 0;

//Set up CAN receive buffers (version 2)
#define CSR_CAN_RX_BUFFER_SIZE  64
volatile CAN_FRAME csr_can_rx_buffer[CSR_CAN_RX_BUFFER_SIZE];
volatile short int csr_can_rx_index1 = 0;
volatile short int csr_can_rx_index2 = 0;

unsigned short csr_can_rx_pop_frame(CAN_FRAME * frameptr)
{
  volatile short int temp_index1 = csr_can_rx_index1;
  volatile short int temp_index2 = csr_can_rx_index2;

  if (temp_index2 != temp_index1)
  {
    *frameptr = csr_can_rx_buffer[temp_index2];
    if (++temp_index2 >= CSR_CAN_RX_BUFFER_SIZE) 
    {
      temp_index2 = 0;
    }
    csr_can_rx_index2 = temp_index2;  // Update read index
    return 0; // Frame popped successfully
  }
  else
  {
    return 1; // Buffer empty
  }
}

////////////////////////////////////////////////////////////////////////////
//Read CAN rx controller buffer, create CAN frame and push onto receive ring buffer
void csr_can_rx1_fiq(void)
{
  short int temp_index = csr_can_rx_index1;

  if (++temp_index == CSR_CAN_RX_BUFFER_SIZE) {temp_index = 0;}
  if (temp_index == csr_can_rx_index2)
  {
    //Buffer full, new data will be lost; don't advance write index
    error_occurred_fiq(ERROR_CAN1_RX_BUF_OF);  
    C1CMR = 1<<2; //Release CAN data buffer
    return;
  }
  ++can_rx_frame_count_1; // Count received frame

  csr_can_rx_buffer[temp_index].chan            = CHAN_CAN1;
  csr_can_rx_buffer[temp_index].addr            = C1RID;
  csr_can_rx_buffer[temp_index].dlc             = (C1RFS>>16)&0xF;
  csr_can_rx_buffer[temp_index].rtr             = (C1RFS>>30)&0x1;
  csr_can_rx_buffer[temp_index].payload.w.w1    = C1RDA;
  csr_can_rx_buffer[temp_index].payload.w.w2    = C1RDB;
   
  C1CMR = 1<<2; //Release CAN data buffer 
    
  csr_can_rx_index1 = temp_index;   //update index1
}
////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////
//Read CAN rx controller buffer, create CAN frame and push onto receive ring buffer
void csr_can_rx2_fiq(void)
{
  short int temp_index = csr_can_rx_index1;

  if (++temp_index == CSR_CAN_RX_BUFFER_SIZE) {temp_index = 0;}
  if (temp_index == csr_can_rx_index2)
  {
    //Buffer full, new data will be lost; don't advance write index
    error_occurred_fiq(ERROR_CAN2_RX_BUF_OF);  
    C2CMR = 1<<2; //Release CAN data buffer
    return;
  }
  ++can_rx_frame_count_2; // Count received frame

  csr_can_rx_buffer[temp_index].chan            = CHAN_CAN2;
  csr_can_rx_buffer[temp_index].addr            = C2RID;
  csr_can_rx_buffer[temp_index].dlc             = (C2RFS>>16)&0xF;
  csr_can_rx_buffer[temp_index].rtr             = (C2RFS>>30)&0x1;
  csr_can_rx_buffer[temp_index].payload.w.w1    = C2RDA;
  csr_can_rx_buffer[temp_index].payload.w.w2    = C2RDB; 

  C2CMR = 1<<2; //Release CAN data buffer 
   
  csr_can_rx_index1 = temp_index;   //update index1
}
////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////
//Read CAN rx controller buffer, create CAN frame and push onto receive ring buffer
void csr_can_rx3_fiq(void)
{
  short int temp_index = csr_can_rx_index1;

  if (++temp_index == CSR_CAN_RX_BUFFER_SIZE) {temp_index = 0;}
  if (temp_index == csr_can_rx_index2)
  {
    //Buffer full, new data will be lost; don't advance write index
    error_occurred_fiq(ERROR_CAN3_RX_BUF_OF);  
    C3CMR = 1<<2; //Release CAN data buffer
    return;
  }

  ++can_rx_frame_count_3; // Count received frame

  csr_can_rx_buffer[temp_index].chan            = CHAN_CAN3;
  csr_can_rx_buffer[temp_index].addr            = C3RID;
  csr_can_rx_buffer[temp_index].dlc             = (C3RFS>>16)&0xF;
  csr_can_rx_buffer[temp_index].rtr             = (C3RFS>>30)&0x1;
  csr_can_rx_buffer[temp_index].payload.w.w1    = C3RDA;
  csr_can_rx_buffer[temp_index].payload.w.w2    = C3RDB;
  
  C3CMR = 1<<2; //Release CAN data buffer 
   
  csr_can_rx_index1 = temp_index;   //update index1
}
////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////
//Read CAN rx controller buffer, create CAN frame and push onto receive ring buffer
void csr_can_rx4_fiq(void)
{
  short int temp_index = csr_can_rx_index1;

  if (++temp_index == CSR_CAN_RX_BUFFER_SIZE) {temp_index = 0;}
  if (temp_index == csr_can_rx_index2)
  {
    //Buffer full, new data will be lost; don't advance write index
    error_occurred_fiq(ERROR_CAN4_RX_BUF_OF);  
    C4CMR = 1<<2; //Release CAN data buffer
    return;
  }

  ++can_rx_frame_count_4; // Count received frame

  csr_can_rx_buffer[temp_index].chan            = CHAN_CAN4;
  csr_can_rx_buffer[temp_index].addr            = C4RID;
  csr_can_rx_buffer[temp_index].dlc             = (C4RFS>>16)&0xF;
  csr_can_rx_buffer[temp_index].rtr             = (C4RFS>>30)&0x1;
  csr_can_rx_buffer[temp_index].payload.w.w1    = C4RDA;
  csr_can_rx_buffer[temp_index].payload.w.w2    = C4RDB;
  
  C4CMR = 1<<2; //Release CAN data buffer 
   
  csr_can_rx_index1 = temp_index;   //update index1
}
////////////////////////////////////////////////////////////////////////////

/*
////////////////////////////////////////////////////////////////////////////
//Read CAN rx controller buffer, create CAN frame and push onto receive ring buffer
void csr_can_rx1_isr(void) __irq
{
  short int temp_index = csr_can_rx_index1;

  #ifdef DEBUG
  if (FIO0PIN & 1<<14)
  {
    VICIntEnClr = 0xFFFFFFFF; //Disable all interrupts before debugging
    VICVectAddr = 0;
    return;
  }
  #endif
  
  if (++temp_index == CSR_CAN_RX_BUFFER_SIZE) {temp_index = 0;}
  if (temp_index == csr_can_rx_index2)
  {
    //Buffer full, new data will be lost; don't advance write index
    error_occurred_irq(ERROR_CAN1_RX_BUF_OF);  
    C1CMR = 1<<2; //Release CAN data buffer
    return;
  }
  ++can_rx_frame_count_1; // Count received frame

  csr_can_rx_buffer[temp_index].chan            = CHAN_CAN1;
  csr_can_rx_buffer[temp_index].addr            = C1RID;
  csr_can_rx_buffer[temp_index].dlc             = (C1RFS>>16)&0xF;
  csr_can_rx_buffer[temp_index].rtr             = (C1RFS>>30)&0x1;
  csr_can_rx_buffer[temp_index].payload.w.w1    = C1RDA;
  csr_can_rx_buffer[temp_index].payload.w.w2    = C1RDB;
   
  C1CMR = 1<<2; //Release CAN data buffer 
   
  csr_can_rx_index1 = temp_index;   //update index1
  VICVectAddr = 0;    // Clear interrupt in VIC.
}
////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////
//Read CAN rx controller buffer, create CAN frame and push onto receive ring buffer
void csr_can_rx2_isr(void) __irq
{
  short int temp_index = csr_can_rx_index1;

  #ifdef DEBUG
  if (FIO0PIN & 1<<14)
  {
    VICIntEnClr = 0xFFFFFFFF; //Disable all interrupts before debugging
    VICVectAddr = 0;
    return;
  }
  #endif

  if (++temp_index == CSR_CAN_RX_BUFFER_SIZE) {temp_index = 0;}
  if (temp_index == csr_can_rx_index2)
  {
    //Buffer full, new data will be lost; don't advance write index
    error_occurred_irq(ERROR_CAN2_RX_BUF_OF);  
    C2CMR = 1<<2; //Release CAN data buffer
    return;
  }
  ++can_rx_frame_count_2; // Count received frame

  csr_can_rx_buffer[temp_index].chan            = CHAN_CAN2;
  csr_can_rx_buffer[temp_index].addr            = C2RID;
  csr_can_rx_buffer[temp_index].dlc             = (C2RFS>>16)&0xF;
  csr_can_rx_buffer[temp_index].rtr             = (C2RFS>>30)&0x1;
  csr_can_rx_buffer[temp_index].payload.w.w1    = C2RDA;
  csr_can_rx_buffer[temp_index].payload.w.w2    = C2RDB; 

  C2CMR = 1<<2; //Release CAN data buffer 
   
  csr_can_rx_index1 = temp_index;   //update index1
  VICVectAddr = 0;    // Clear interrupt in VIC.
}
////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////
//Read CAN rx controller buffer, create CAN frame and push onto receive ring buffer
void csr_can_rx3_isr(void) __irq
{
  short int temp_index = csr_can_rx_index1;

  #ifdef DEBUG
  if (FIO0PIN & 1<<14)
  {
    VICIntEnClr = 0xFFFFFFFF; //Disable all interrupts before debugging
    VICVectAddr = 0;
    return;
  }
  #endif

  if (++temp_index == CSR_CAN_RX_BUFFER_SIZE) {temp_index = 0;}
  if (temp_index == csr_can_rx_index2)
  {
    //Buffer full, new data will be lost; don't advance write index
    error_occurred_irq(ERROR_CAN3_RX_BUF_OF);  
    C3CMR = 1<<2; //Release CAN data buffer
    return;
  }

  ++can_rx_frame_count_3; // Count received frame

  csr_can_rx_buffer[temp_index].chan            = CHAN_CAN3;
  csr_can_rx_buffer[temp_index].addr            = C3RID;
  csr_can_rx_buffer[temp_index].dlc             = (C3RFS>>16)&0xF;
  csr_can_rx_buffer[temp_index].rtr             = (C3RFS>>30)&0x1;
  csr_can_rx_buffer[temp_index].payload.w.w1    = C3RDA;
  csr_can_rx_buffer[temp_index].payload.w.w2    = C1RDB;
  
  C3CMR = 1<<2; //Release CAN data buffer 
   
  csr_can_rx_index1 = temp_index;   //update index1
  VICVectAddr = 0;    // Clear interrupt in VIC.
}
////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////
//Read CAN rx controller buffer, create CAN frame and push onto receive ring buffer
void csr_can_rx4_isr(void) __irq
{
  short int temp_index = csr_can_rx_index1;

  #ifdef DEBUG
  if (FIO0PIN & 1<<14)
  {
    VICIntEnClr = 0xFFFFFFFF; //Disable all interrupts before debugging
    VICVectAddr = 0;
    return;
  }
  #endif
   
  if (++temp_index == CSR_CAN_RX_BUFFER_SIZE) {temp_index = 0;}
  if (temp_index == csr_can_rx_index2)
  {
    //Buffer full, new data will be lost; don't advance write index
    error_occurred_irq(ERROR_CAN4_RX_BUF_OF);  
    C4CMR = 1<<2; //Release CAN data buffer
    return;
  }

  ++can_rx_frame_count_4; // Count received frame

  csr_can_rx_buffer[temp_index].chan            = CHAN_CAN4;
  csr_can_rx_buffer[temp_index].addr            = C4RID;
  csr_can_rx_buffer[temp_index].dlc             = (C4RFS>>16)&0xF;
  csr_can_rx_buffer[temp_index].rtr             = (C4RFS>>30)&0x1;
  csr_can_rx_buffer[temp_index].payload.w.w1    = C4RDA;
  csr_can_rx_buffer[temp_index].payload.w.w2    = C4RDB;
  
  C4CMR = 1<<2; //Release CAN data buffer 
   
  csr_can_rx_index1 = temp_index;   //update index1
  VICVectAddr = 0;    // Clear interrupt in VIC.
}
////////////////////////////////////////////////////////////////////////////
*/

////////////////////////////////////////////////////////////////////////////
//Push one CAN frame onto CAN1 transmit ring buffer
unsigned short int csr_can1_tx_push_frame(CAN_FRAME * frameptr)
{
  short int ret = 1, temp_index = csr_can1_tx_index1;
  
  if (++temp_index == CSR_CAN_TX_BUFFER_SIZE) {temp_index = 0;}

  if (temp_index != csr_can1_tx_index2) 
  {
    //Push new frame onto buffer
    csr_can1_tx_buffer[csr_can1_tx_index1] = *frameptr;
    csr_can1_tx_index1 = temp_index;   //update index1
    ret = 0; //Frame successfully pushed onto buffer
  }
   
// uncomment the following two lines for use with isr
 // VICSoftInt = 1<<20;   //Force CAN1 TX interrupt
 // VICIntEnable = 1<<20; //Enable CAN1 TX interrupts
  
  return ret; // 0 = successful push, 1 = buffer full
}

/*
////////////////////////////////////////////////////////////////////////////
void csr_can1_tx_isr(void) __irq
{
  #ifdef DEBUG
    if (FIO0PIN & 1<<14)
    {
      VICIntEnClr = 0xFFFFFFFF; //Disable all interrupts before debugging
      VICVectAddr = 0;
      return;
    }
  #endif
  
  while (csr_can1_tx_index2 != csr_can1_tx_index1)
  {
    if (C1SR & (1<<2)) // buffer 1 is available
    {   
      C1TFI1 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
      C1TID1 = csr_can1_tx_buffer[csr_can1_tx_index2].addr;   //get CAN_ID
      C1TDA1 = csr_can1_tx_buffer[csr_can1_tx_index2].payload.w.w1;   //load first data word
      C1TDB1 = csr_can1_tx_buffer[csr_can1_tx_index2].payload.w.w2;   //load second data word	
      C1CMR = (1<<0) | (1<<5);     // Select buffer 1, request transmission
      if (++csr_can1_tx_index2 == CSR_CAN_TX_BUFFER_SIZE) {csr_can1_tx_index2 = 0;}

      //Move to next read index location
      if (csr_can1_tx_index2 >= CSR_CAN_TX_BUFFER_SIZE - 1)
      {
        csr_can1_tx_index2 = 0;
      }
      else
      {
        ++csr_can1_tx_index2;
      }

      can_tx_frame_count_1++;
    //  b10a_can_packet_count(1);
    }
    else if (C1SR & (1<<10)) // buffer 2 is available
    {
      C1TFI2 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
      C1TID2 = csr_can1_tx_buffer[csr_can1_tx_index2].addr;   //get CAN_ID
      C1TDA2 = csr_can1_tx_buffer[csr_can1_tx_index2].payload.w.w1;   //load first data word
      C1TDB2 = csr_can1_tx_buffer[csr_can1_tx_index2].payload.w.w2;   //load second data word	
      C1CMR = (1<<0) | (1<<6);     // select buffer 2, request transmission

      //Move to next read index location
      if (csr_can1_tx_index2 >= CSR_CAN_TX_BUFFER_SIZE - 1)
      {
        csr_can1_tx_index2 = 0;
      }
      else
      {
        ++csr_can1_tx_index2;
      }


      can_tx_frame_count_1++;
     // b10a_can_packet_count(1);
    }
    else
    {
      break;   //all available buffers full
    } 
  }
  
  if (csr_can1_tx_index2 == csr_can1_tx_index1)
  {
//   C1IER = (C1IER & (~(1<<1))) & 0x7FF; // clear TIE1 (Transmit Interrupt Enable 1) and mask reserved bits
    VICIntEnClr = 1<<20; //Disable CAN1 TX interrupts
  }
    
  VICSoftIntClr = 1<<20;  //Clear any CAN1 TX software interrupt
  VICVectAddr = 0;    // Clear interrupt in VIC.
}
////////////////////////////////////////////////////////////////////////////
*/
////////////////////////////////////////////////////////////////////////////
void csr_can1_tx(void)
{
  while (csr_can1_tx_index2 != csr_can1_tx_index1)
  {
    if (C1SR & (1<<2)) // buffer 1 is available
    {   
      C1TFI1 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
      C1TID1 = csr_can1_tx_buffer[csr_can1_tx_index2].addr;   //get CAN_ID
      C1TDA1 = csr_can1_tx_buffer[csr_can1_tx_index2].payload.w.w1;   //load first data word
      C1TDB1 = csr_can1_tx_buffer[csr_can1_tx_index2].payload.w.w2;   //load second data word	
      C1CMR = (1<<0) | (1<<5);     // Select buffer 1, request transmission

      //Move to next read index location
      if (csr_can1_tx_index2 >= CSR_CAN_TX_BUFFER_SIZE - 1)
      {
        csr_can1_tx_index2 = 0;
      }
      else
      {
        ++csr_can1_tx_index2;
      }

      can_tx_frame_count_1++;
    //  b10a_can_packet_count(1);
    }
    else if (C1SR & (1<<10)) // buffer 2 is available
    {
      C1TFI2 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
      C1TID2 = csr_can1_tx_buffer[csr_can1_tx_index2].addr;   //get CAN_ID
      C1TDA2 = csr_can1_tx_buffer[csr_can1_tx_index2].payload.w.w1;   //load first data word
      C1TDB2 = csr_can1_tx_buffer[csr_can1_tx_index2].payload.w.w2;   //load second data word	
      C1CMR = (1<<0) | (1<<6);     // select buffer 2, request transmission

      //Move to next read index location
      if (csr_can1_tx_index2 >= CSR_CAN_TX_BUFFER_SIZE - 1)
      {
        csr_can1_tx_index2 = 0;
      }
      else
      {
        ++csr_can1_tx_index2;
      }


      can_tx_frame_count_1++;
     // b10a_can_packet_count(1);
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
unsigned short int csr_can2_tx_push_frame(CAN_FRAME * frameptr)
{
  short int ret = 1, temp_index = csr_can2_tx_index1;
  
  if (++temp_index == CSR_CAN_TX_BUFFER_SIZE) {temp_index = 0;}

  if (temp_index != csr_can2_tx_index2) 
  {
    //Push new frame onto buffer
    csr_can2_tx_buffer[csr_can2_tx_index1] = *frameptr;
    csr_can2_tx_index1 = temp_index;   //update index1
    ret = 0; //Frame successfully pushed onto buffer
  }
 
// uncomment the following two lines for use with isr  
  //VICSoftInt = 1<<21;   //Force CAN2 TX interrupt
  //VICIntEnable = 1<<21; //Enable CAN2 TX interrupts
  
  return ret; // 0 = successful push, 1 = buffer full
}

////////////////////////////////////////////////////////////////////////////
/*
void csr_can2_tx_isr(void) __irq
{
 #ifdef DEBUG
    if (FIO0PIN & 1<<14)
    {
      VICIntEnClr = 0xFFFFFFFF; //Disable all interrupts before debugging
      VICVectAddr = 0;
      return;
    }
  #endif
  
  while (csr_can2_tx_index2 != csr_can2_tx_index1)
  {
    if (C2SR & (1<<2)) // buffer 1 is available
    {   
      C2TFI1 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
      C2TID1 = csr_can2_tx_buffer[csr_can2_tx_index2].addr;   //get CAN_ID
      C2TDA1 = csr_can2_tx_buffer[csr_can2_tx_index2].payload.w.w1;   //load first data word
      C2TDB1 = csr_can2_tx_buffer[csr_can2_tx_index2].payload.w.w2;   //load second data word	
      C2CMR = (1<<0) | (1<<5);     // select buffer 1, request transmission
      can_tx_frame_count_2++;

      //Move to next read index location
      if (csr_can2_tx_index2 >= CSR_CAN_TX_BUFFER_SIZE - 1)
      {
        csr_can2_tx_index2 = 0;
      }
      else
      {
        ++csr_can2_tx_index2;
      }
    }
    else if (C2SR & (1<<10)) // buffer 2 is available
    {
      C2TFI2 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
      C2TID2 = csr_can2_tx_buffer[csr_can2_tx_index2].addr;   //get CAN_ID
      C2TDA2 = csr_can2_tx_buffer[csr_can2_tx_index2].payload.w.w1;   //load first data word
      C2TDB2 = csr_can2_tx_buffer[csr_can2_tx_index2].payload.w.w2;   //load second data word	
      C2CMR = (1<<0) | (1<<6);     // select buffer 2, request transmission
      can_tx_frame_count_2++;

      //Move to next read index location
      if (csr_can2_tx_index2 >= CSR_CAN_TX_BUFFER_SIZE - 1)
      {
        csr_can2_tx_index2 = 0;
      }
      else
      {
        ++csr_can2_tx_index2;
      }
    }
    else
    {
      break;   //all available buffers full
    } 
  }
  if (csr_can2_tx_index2 == csr_can2_tx_index1)
  {
    VICIntEnClr = 1<<21; //Disable CAN2 TX interrupts
  }
  
  VICSoftIntClr = 1<<21;  //Clear any CAN2 TX software interrupt
  VICVectAddr = 0;    // Clear interrupt in VIC.
}
////////////////////////////////////////////////////////////////////////////
*/
////////////////////////////////////////////////////////////////////////////
void csr_can2_tx(void)
{  
  while (csr_can2_tx_index2 != csr_can2_tx_index1)
  {
    if (C2SR & (1<<2)) // buffer 1 is available
    {   
      C2TFI1 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
      C2TID1 = csr_can2_tx_buffer[csr_can2_tx_index2].addr;   //get CAN_ID
      C2TDA1 = csr_can2_tx_buffer[csr_can2_tx_index2].payload.w.w1;   //load first data word
      C2TDB1 = csr_can2_tx_buffer[csr_can2_tx_index2].payload.w.w2;   //load second data word	
      C2CMR = (1<<0) | (1<<5);     // select buffer 1, request transmission
      can_tx_frame_count_2++;

      //Move to next read index location
      if (csr_can2_tx_index2 >= CSR_CAN_TX_BUFFER_SIZE - 1)
      {
        csr_can2_tx_index2 = 0;
      }
      else
      {
        ++csr_can2_tx_index2;
      }
    }
    else if (C2SR & (1<<10)) // buffer 2 is available
    {
      C2TFI2 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
      C2TID2 = csr_can2_tx_buffer[csr_can2_tx_index2].addr;   //get CAN_ID
      C2TDA2 = csr_can2_tx_buffer[csr_can2_tx_index2].payload.w.w1;   //load first data word
      C2TDB2 = csr_can2_tx_buffer[csr_can2_tx_index2].payload.w.w2;   //load second data word	
      C2CMR = (1<<0) | (1<<6);     // select buffer 2, request transmission
      can_tx_frame_count_2++;

      //Move to next read index location
      if (csr_can2_tx_index2 >= CSR_CAN_TX_BUFFER_SIZE - 1)
      {
        csr_can2_tx_index2 = 0;
      }
      else
      {
        ++csr_can2_tx_index2;
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
unsigned short int csr_can3_tx_push_frame(CAN_FRAME * frameptr)
{
  short int ret = 1, temp_index = csr_can3_tx_index1;
  
  if (++temp_index == CSR_CAN_TX_BUFFER_SIZE) {temp_index = 0;}

  if (temp_index != csr_can3_tx_index2) 
  {
    //Push new frame onto buffer
    csr_can3_tx_buffer[csr_can3_tx_index1] = *frameptr;
    csr_can3_tx_index1 = temp_index;   //update index1
    ret = 0; //Frame successfully pushed onto buffer
  }
  
 // uncomment the following two lines for use with isr
 // VICSoftInt = 1<<22;   //Force CAN3 TX interrupt
  //VICIntEnable = 1<<22; //Enable CAN3 TX interrupts
  
  return ret; // 0 = successful push, 1 = buffer full
}

////////////////////////////////////////////////////////////////////////////
/*
void csr_can3_tx_isr(void) __irq
{
 #ifdef DEBUG
    if (FIO0PIN & 1<<14)
    {
      VICIntEnClr = 0xFFFFFFFF; //Disable all interrupts before debugging
      VICVectAddr = 0;
      return;
    }
  #endif

  while (csr_can3_tx_index2 != csr_can3_tx_index1)
  {
    if (C3SR & (1<<2)) // buffer 1 is available
    {   
      C3TFI1 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
      C3TID1 = csr_can3_tx_buffer[csr_can3_tx_index2].addr;   //get CAN_ID
      C3TDA1 = csr_can3_tx_buffer[csr_can3_tx_index2].payload.w.w1;   //load first data word
      C3TDB1 = csr_can3_tx_buffer[csr_can3_tx_index2].payload.w.w2;   //load second data word	
      C3CMR = (1<<0) | (1<<5);     // select buffer 1, request transmission
      can_tx_frame_count_3++;
 
      //Move to next read index location
      if (csr_can3_tx_index2 >= CSR_CAN_TX_BUFFER_SIZE - 1)
      {
        csr_can3_tx_index2 = 0;
      }
      else
      {
        ++csr_can3_tx_index2;
      }
    }
    else if (C3SR & (1<<10)) // buffer 2 is available
    {
      C3TFI2 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
      C3TID2 = csr_can3_tx_buffer[csr_can3_tx_index2].addr;   //get CAN_ID
      C3TDA2 = csr_can3_tx_buffer[csr_can3_tx_index2].payload.w.w1;   //load first data word
      C3TDB2 = csr_can3_tx_buffer[csr_can3_tx_index2].payload.w.w2;   //load second data word	
      C3CMR = (1<<0) | (1<<6);     // select buffer 2, request transmission
      can_tx_frame_count_3++;

      //Move to next read index location
      if (csr_can3_tx_index2 >= CSR_CAN_TX_BUFFER_SIZE - 1)
      {
        csr_can3_tx_index2 = 0;
      }
      else
      {
        ++csr_can3_tx_index2;
      }
    }
    else
    {
      break;   //all available buffers full
    } 
  }
  if (csr_can3_tx_index2 == csr_can3_tx_index1)
  {
    VICIntEnClr = 1<<22; //Disable CAN3 TX interrupts
  }
  
  VICSoftIntClr = 1<<22;  //Clear any CAN3 TX software interrupt
  VICVectAddr = 0;    // Clear interrupt in VIC.
}
////////////////////////////////////////////////////////////////////////////
*/
////////////////////////////////////////////////////////////////////////////
void csr_can3_tx(void)
{
  while (csr_can3_tx_index2 != csr_can3_tx_index1)
  {
    if (C3SR & (1<<2)) // buffer 1 is available
    {   
      C3TFI1 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
      C3TID1 = csr_can3_tx_buffer[csr_can3_tx_index2].addr;   //get CAN_ID
      C3TDA1 = csr_can3_tx_buffer[csr_can3_tx_index2].payload.w.w1;   //load first data word
      C3TDB1 = csr_can3_tx_buffer[csr_can3_tx_index2].payload.w.w2;   //load second data word	
      C3CMR = (1<<0) | (1<<5);     // select buffer 1, request transmission
      can_tx_frame_count_3++;
 
      //Move to next read index location
      if (csr_can3_tx_index2 >= CSR_CAN_TX_BUFFER_SIZE - 1)
      {
        csr_can3_tx_index2 = 0;
      }
      else
      {
        ++csr_can3_tx_index2;
      }
    }
    else if (C3SR & (1<<10)) // buffer 2 is available
    {
      C3TFI2 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
      C3TID2 = csr_can3_tx_buffer[csr_can3_tx_index2].addr;   //get CAN_ID
      C3TDA2 = csr_can3_tx_buffer[csr_can3_tx_index2].payload.w.w1;   //load first data word
      C3TDB2 = csr_can3_tx_buffer[csr_can3_tx_index2].payload.w.w2;   //load second data word	
      C3CMR = (1<<0) | (1<<6);     // select buffer 2, request transmission
      can_tx_frame_count_3++;

      //Move to next read index location
      if (csr_can3_tx_index2 >= CSR_CAN_TX_BUFFER_SIZE - 1)
      {
        csr_can3_tx_index2 = 0;
      }
      else
      {
        ++csr_can3_tx_index2;
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
unsigned short int csr_can4_tx_push_frame(CAN_FRAME * frameptr)
{
  short int ret = 1, temp_index = csr_can4_tx_index1;
  
  if (++temp_index == CSR_CAN_TX_BUFFER_SIZE) {temp_index = 0;}

  if (temp_index != csr_can4_tx_index2) 
  {
    //Push new frame onto buffer
    csr_can4_tx_buffer[csr_can4_tx_index1] = *frameptr;
    csr_can4_tx_index1 = temp_index;   //update index1
    ret = 0; //Frame successfully pushed onto buffer
  }
  
// uncomment the following two lines for use with isr
  //VICSoftInt = 1<<23;   //Force CAN4 TX interrupt
  //VICIntEnable = 1<<23; //Enable CAN4 TX interrupts
  
  return ret; // 0 = successful push, 1 = buffer full
}

////////////////////////////////////////////////////////////////////////////
/*
void csr_can4_tx_isr(void) __irq
{
 #ifdef DEBUG
    if (FIO0PIN & 1<<14)
    {
      VICIntEnClr = 0xFFFFFFFF; //Disable all interrupts before debugging
      VICVectAddr = 0;
      return;
    }
  #endif

  while (csr_can4_tx_index2 != csr_can4_tx_index1)
  {
    if (C4SR & (1<<2)) // buffer 1 is available
    {   
      C4TFI1 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
      C4TID1 = csr_can4_tx_buffer[csr_can4_tx_index2].addr;   //get CAN_ID
      C4TDA1 = csr_can4_tx_buffer[csr_can4_tx_index2].payload.w.w1;   //load first data word
      C4TDB1 = csr_can4_tx_buffer[csr_can4_tx_index2].payload.w.w2;   //load second data word	
      C4CMR = (1<<0) | (1<<5);     // select buffer 1, request transmission
      can_tx_frame_count_4++;
 
      //Move to next read index location
      if (csr_can4_tx_index2 >= CSR_CAN_TX_BUFFER_SIZE - 1)
      {
        csr_can4_tx_index2 = 0;
      }
      else
      {
        ++csr_can4_tx_index2;
      }
    }
    else if (C4SR & (1<<10)) // buffer 2 is available
    {
      C4TFI2 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
	    C4TID2 = csr_can4_tx_buffer[csr_can4_tx_index2].addr;   //get CAN_ID
      C4TDA2 = csr_can4_tx_buffer[csr_can4_tx_index2].payload.w.w1;   //load first data word
      C4TDB2 = csr_can4_tx_buffer[csr_can4_tx_index2].payload.w.w2;   //load second data word	
      C4CMR = (1<<0) | (1<<6);     // select buffer 2, request transmission
      can_tx_frame_count_4++;

      //Move to next read index location
      if (csr_can4_tx_index2 >= CSR_CAN_TX_BUFFER_SIZE - 1)
      {
        csr_can4_tx_index2 = 0;
      }
      else
      {
        ++csr_can4_tx_index2;
      }
    }
    else
    {
      break;   //all available buffers full
    } 
  }
  if (csr_can4_tx_index2 == csr_can4_tx_index1)
  {
    VICIntEnClr = 1<<23; //Disable CAN4 TX interrupts
  }
  
  VICSoftIntClr = 1<<23;  //Clear any CAN4 TX software interrupt
  VICVectAddr = 0;    // Clear interrupt in VIC.
}
////////////////////////////////////////////////////////////////////////////
*/
////////////////////////////////////////////////////////////////////////////
void csr_can4_tx(void)
{
  while (csr_can4_tx_index2 != csr_can4_tx_index1)
  {
    if (C4SR & (1<<2)) // buffer 1 is available
    {   
      C4TFI1 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
      C4TID1 = csr_can4_tx_buffer[csr_can4_tx_index2].addr;   //get CAN_ID
      C4TDA1 = csr_can4_tx_buffer[csr_can4_tx_index2].payload.w.w1;   //load first data word
      C4TDB1 = csr_can4_tx_buffer[csr_can4_tx_index2].payload.w.w2;   //load second data word	
      C4CMR = (1<<0) | (1<<5);     // select buffer 1, request transmission
      can_tx_frame_count_4++;
 
      //Move to next read index location
      if (csr_can4_tx_index2 >= CSR_CAN_TX_BUFFER_SIZE - 1)
      {
        csr_can4_tx_index2 = 0;
      }
      else
      {
        ++csr_can4_tx_index2;
      }
    }
    else if (C4SR & (1<<10)) // buffer 2 is available
    {
      C4TFI2 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
	    C4TID2 = csr_can4_tx_buffer[csr_can4_tx_index2].addr;   //get CAN_ID
      C4TDA2 = csr_can4_tx_buffer[csr_can4_tx_index2].payload.w.w1;   //load first data word
      C4TDB2 = csr_can4_tx_buffer[csr_can4_tx_index2].payload.w.w2;   //load second data word	
      C4CMR = (1<<0) | (1<<6);     // select buffer 2, request transmission
      can_tx_frame_count_4++;

      //Move to next read index location
      if (csr_can4_tx_index2 >= CSR_CAN_TX_BUFFER_SIZE - 1)
      {
        csr_can4_tx_index2 = 0;
      }
      else
      {
        ++csr_can4_tx_index2;
      }
    }
    else
    {
      return;   //all available buffers full
    } 
  }
}
/*
////////////////////////////////////////////////////////////////////////////

// **** TEST CODE **** send dummy data, no buffers involved
////////////////////////////////////////////////////////////////////////////
void csr_can1_tx_test(void)
{
    if (C1SR & (1<<2)) // buffer 1 is available
    {   
      C1TFI1 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
      C1TID1 = 600;   //get CAN_ID
      C1TDA1 = 0xAAAAAAAA;   //load first data word
      C1TDB1 = 0xBBBBBBBB;   //load second data word	
      C1CMR = (1<<0) | (1<<5);     // select buffer 1, request transmission
      can_tx_frame_count_1++;
    }
    if (C1SR & (1<<10)) // buffer 2 is available
    {
     C1TFI1 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
      C1TID1 = 601;   //get CAN_ID
      C1TDA1 = 0xAAAAAAAA;   //load first data word
      C1TDB1 = 0xBBBBBBBB;   //load second data word	
      C1CMR = (1<<0) | (1<<5);     // select buffer 1, request transmission
      can_tx_frame_count_1++;
    }
}
////////////////////////////////////////////////////////////////////////////

// **** TEST CODE **** send dummy data, no buffers involved
////////////////////////////////////////////////////////////////////////////
void csr_can2_tx_test(void)
{
    if (C2SR & (1<<2)) // buffer 1 is available
    {   
      C2TFI1 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
      C2TID1 = 602;   //get CAN_ID
      C2TDA1 = 0xAAAAAAAA;   //load first data word
      C2TDB1 = 0xBBBBBBBB;   //load second data word	
      C2CMR = (1<<0) | (1<<5);     // select buffer 1, request transmission
      can_tx_frame_count_2++;
    }
    if (C2SR & (1<<10)) // buffer 2 is available
    {
     C2TFI1 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
      C2TID1 = 603;   //get CAN_ID
      C2TDA1 = 0xAAAAAAAA;   //load first data word
      C2TDB1 = 0xBBBBBBBB;   //load second data word	
      C2CMR = (1<<0) | (1<<5);     // select buffer 1, request transmission
      can_tx_frame_count_2++;
    }
}
////////////////////////////////////////////////////////////////////////////

// **** TEST CODE **** send dummy data, no buffers involved
////////////////////////////////////////////////////////////////////////////
void csr_can3_tx_test(void)
{
    if (C3SR & (1<<2)) // buffer 1 is available
    {   
      C3TFI1 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
      C3TID1 = 604;   //get CAN_ID
      C3TDA1 = 0xAAAAAAAA;   //load first data word
      C3TDB1 = 0xBBBBBBBB;   //load second data word	
      C3CMR = (1<<0) | (1<<5);     // select buffer 1, request transmission
      can_tx_frame_count_3++;
    }
    if (C3SR & (1<<10)) // buffer 2 is available
    {
     C3TFI1 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
      C3TID1 = 605;   //get CAN_ID
      C3TDA1 = 0xAAAAAAAA;   //load first data word
      C3TDB1 = 0xBBBBBBBB;   //load second data word	
      C3CMR = (1<<0) | (1<<5);     // select buffer 1, request transmission
      can_tx_frame_count_3++;
    }
}
////////////////////////////////////////////////////////////////////////////

// **** TEST CODE **** send dummy data, no buffers involved
////////////////////////////////////////////////////////////////////////////
void csr_can4_tx_test(void)
{
    if (C4SR & (1<<2)) // buffer 1 is available
    {   
      C4TFI1 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
      C4TID1 = 605;   //get CAN_ID
      C4TDA1 = 0xAAAAAAAA;   //load first data word
      C4TDB1 = 0xBBBBBBBB;   //load second data word	
      C4CMR = (1<<0) | (1<<5);     // select buffer 1, request transmission
      can_tx_frame_count_4++;
    }
    if (C4SR & (1<<10)) // buffer 2 is available
    {
     C4TFI1 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
      C4TID1 = 606;   //get CAN_ID
      C4TDA1 = 0xAAAAAAAA;   //load first data word
      C4TDB1 = 0xBBBBBBBB;   //load second data word	
      C4CMR = (1<<0) | (1<<5);     // select buffer 1, request transmission
      can_tx_frame_count_4++;
    }
}
////////////////////////////////////////////////////////////////////////////
*/

unsigned short can_get_error_1(CAN_FRAME * frame)
{
  frame->payload.w.w1 = can_error_1;
  can_error_1 = 0;     //Reset error
  frame->payload.w.w2 = 0;
  return 0;
}

unsigned short can_get_error_2(CAN_FRAME * frame)
{
  frame->payload.w.w1 = can_error_2;
  can_error_2 = 0;     //Reset error
  frame->payload.w.w2 = 0;
  return 0;
}

unsigned short can_get_error_3(CAN_FRAME * frame)
{
  frame->payload.w.w1 = can_error_3;
  can_error_3 = 0;     //Reset error
  frame->payload.w.w2 = 0;
  return 0;
}

unsigned short can_get_error_4(CAN_FRAME * frame)
{
  frame->payload.w.w1 = can_error_4;
  can_error_4 = 0;     //Reset error
  frame->payload.w.w2 = 0;
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
