//Code for SSP - CAN router. CAN frames are received from up to 4 CAN buses and the SSP bus,
//and directed out again as per the bits set in the routing table. Each possible standard CAN address
//from 0 to 2031 (CAN does not permit addresses 2032 through 2047) gets its own routing entry (one byte)
//in the CSR_Routing_Table lookup table. Bit 0 (LSB) indicates that SSP (the main brain) is to receive a
//frame; bit 1 through 4 are for CAN buses 1 through 4, respectively.

#include <includes.h>

//Global variables

//CAN packet routing table. This can be filled in manually, or (future) by having each
//processor "subscribe" to data types it wants to receive by sending out a request to
//receive (RTR) packet for each data type. Routing table size would be 11 bits (2048),
//except that CAN does not allow the first 7 bits to be high (recessive), thus limiting
//the top CAN address in standard mode to 2031. (CAN20B.pdf page 13)

unsigned char CSR_Routing_Table[2032];

//Set up ring name and buffer for CAN receive
#define RX_FRAME_BUF_LEN 64
CAN_RING rx_ring;
CAN_FRAME rx_frame_buf[RX_FRAME_BUF_LEN];

//Set up 4 ring names and buffers for CAN transmit
#define TX_FRAME_BUF_LEN 16
CAN_RING tx1_ring;
CAN_FRAME tx1_frame_buf[TX_FRAME_BUF_LEN];
CAN_RING tx2_ring;
CAN_FRAME tx2_frame_buf[TX_FRAME_BUF_LEN];
CAN_RING tx3_ring;
CAN_FRAME tx3_frame_buf[TX_FRAME_BUF_LEN];
CAN_RING tx4_ring;
CAN_FRAME tx4_frame_buf[TX_FRAME_BUF_LEN];

//Global variables for SSP software receive buffer
#define CSR_SEG_SIZE 31
#define CSR_RX_SEG_NUM 8

volatile unsigned short int csr_rx_buffer[CSR_RX_SEG_NUM][CSR_SEG_SIZE];//FIFO read buffer for data from SSP
volatile short int csr_rx_index1 = 0;			//Points to location of segment being written by SSP isr
volatile short int csr_rx_index2 = 0;			//Points to location of segment being read by parser
												                  //If Index2 = Index1, buffer contains no unread data.
                                          //If Index1 precedes Index2 by one in ring, buffer is full.

//Global variables for SSP software transmit buffer
#define CSR_TX_BUFFER_SIZE 500
volatile unsigned short int csr_tx_buffer[CSR_TX_BUFFER_SIZE];
volatile short int csr_tx_index1 = 0;
volatile short int csr_tx_index2 = 0;

////////////////////////////////////////////////////////////////////////////
void csr_routing_table_init(void)
{
  CSR_Routing_Table[0] = 1<<CHAN_CAN1;
  CSR_Routing_Table[1] = 1<<CHAN_CAN2;
  CSR_Routing_Table[2] = 1<<CHAN_CAN3;
  CSR_Routing_Table[3] = 1<<CHAN_CAN4;
  CSR_Routing_Table[4] = 1<<CHAN_CAN1;
  CSR_Routing_Table[5] = 1<<CHAN_CAN2;
  CSR_Routing_Table[6] = 1<<CHAN_CAN3;
  CSR_Routing_Table[7] = 1<<CHAN_CAN4;
  CSR_Routing_Table[8] = 1<<CHAN_CAN1;
  CSR_Routing_Table[9] = 1<<CHAN_CAN2;
  CSR_Routing_Table[10] = 1<<CHAN_CAN3;
  CSR_Routing_Table[11] = 1<<CHAN_CAN4;
  CSR_Routing_Table[12] = 1<<CHAN_CAN1;
  CSR_Routing_Table[13] = 1<<CHAN_CAN2;
  CSR_Routing_Table[14] = 1<<CHAN_CAN3;
  CSR_Routing_Table[15] = 1<<CHAN_CAN4;
  CSR_Routing_Table[16] = 1<<CHAN_CAN1;
  CSR_Routing_Table[17] = 1<<CHAN_CAN2;
  CSR_Routing_Table[18] = 1<<CHAN_CAN3;   
  CSR_Routing_Table[19] = 1<<CHAN_CAN4;
  CSR_Routing_Table[20] = 1<<CHAN_SSP;
  CSR_Routing_Table[21] = 1<<CHAN_SSP;
  CSR_Routing_Table[22] = 1<<CHAN_SSP;
  CSR_Routing_Table[23] = 1<<CHAN_SSP;
  CSR_Routing_Table[24] = 1<<CHAN_SSP;
  CSR_Routing_Table[25] = 1<<CHAN_SSP;
  CSR_Routing_Table[26] = 1<<CHAN_SSP;
  CSR_Routing_Table[27] = 1<<CHAN_SSP;
  CSR_Routing_Table[28] = 1<<CHAN_SSP;   
  CSR_Routing_Table[29] = 1<<CHAN_SSP;
  CSR_Routing_Table[30] = 1<<CHAN_SSP;
  CSR_Routing_Table[31] = 1<<CHAN_SSP;
  CSR_Routing_Table[32] = 1<<CHAN_SSP;
  CSR_Routing_Table[33] = 1<<CHAN_SSP;
  CSR_Routing_Table[34] = 1<<CHAN_SSP;
  CSR_Routing_Table[35] = 1<<CHAN_SSP;
  CSR_Routing_Table[36] = 1<<CHAN_SSP;
  CSR_Routing_Table[37] = 1<<CHAN_SSP;
  CSR_Routing_Table[38] = 1<<CHAN_SSP;   
  CSR_Routing_Table[39] = 1<<CHAN_SSP;
  CSR_Routing_Table[40] = 1<<CHAN_SSP;
  CSR_Routing_Table[41] = 1<<CHAN_SSP;
  CSR_Routing_Table[42] = 1<<CHAN_SSP;
  CSR_Routing_Table[43] = 1<<CHAN_SSP;
  CSR_Routing_Table[44] = 1<<CHAN_SSP;
  CSR_Routing_Table[45] = 1<<CHAN_SSP;
  CSR_Routing_Table[46] = 1<<CHAN_SSP;
  CSR_Routing_Table[47] = 1<<CHAN_SSP;
  CSR_Routing_Table[48] = 1<<CHAN_SSP;   
  CSR_Routing_Table[49] = 1<<CHAN_SSP;
}

#define CSR_CAN_TX_BUFFER_SIZE  50
volatile CAN_FRAME csr_can1_tx_buffer[CSR_CAN_TX_BUFFER_SIZE];
volatile short int csr_can1_tx_index1 = 0;
volatile short int csr_can1_tx_index2 = 0;

////////////////////////////////////////////////////////////////////////////
//Push one CAN frame onto CAN1 transmit ring buffer
unsigned short int csr_can1_tx_push_frame(CAN_FRAME * frameptr)
{
  short int temp_index = csr_can1_tx_index1;
  
  csr_can1_tx_buffer[temp_index] = *frameptr;
  
  if (++temp_index == CSR_CAN_TX_BUFFER_SIZE) {temp_index = 0;}
  if (temp_index == csr_can1_tx_index2) {return 1;} //buffer full
  
  csr_can1_tx_index1 = temp_index;   //update index1
    
  VICSoftInt = 1<<20;   //Force CAN1 TX interrupt
  VICIntEnable = 1<<20; //Enable CAN1 TX interrupts
  
  return 0; //Success - data on buffer
}
////////////////////////////////////////////////////////////////////////////
void csr_can1_tx_isr(void) __irq
{
  while (csr_can1_tx_index2 != csr_can1_tx_index1)
  {
    if (C1SR & (1<<2)) // buffer 1 is available
    {   
      C1TFI1 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
      C1TID1 = csr_can1_tx_buffer[csr_can1_tx_index2].addr;   //get CAN_ID
      C1TDA1 = csr_can1_tx_buffer[csr_can1_tx_index2].payload.w.w1;   //load first data word
      C1TDB1 = csr_can1_tx_buffer[csr_can1_tx_index2].payload.w.w2;   //load second data word	
      C1CMR = 1<<5;     // select buffer 1 for transmit
      C1CMR = 1<<0;     // transmission request
      if (++csr_can1_tx_index2 == CSR_CAN_TX_BUFFER_SIZE) {csr_can1_tx_index2 = 0;}
    }
    else if (C1SR & (1<<10)) // buffer 2 is available
    {
      C1TFI2 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
      C1TID2 = csr_can1_tx_buffer[csr_can1_tx_index2].addr;   //get CAN_ID
      C1TDA2 = csr_can1_tx_buffer[csr_can1_tx_index2].payload.w.w1;   //load first data word
      C1TDB2 = csr_can1_tx_buffer[csr_can1_tx_index2].payload.w.w2;   //load second data word	
      C1CMR = 1<<6;     // select buffer 2 for transmit
      C1CMR = 1<<0;     // transmission request
      if (++csr_can1_tx_index2 == CSR_CAN_TX_BUFFER_SIZE) {csr_can1_tx_index2 = 0;}
    }
    else
    {
      break;   //all available buffers full
    } 
  }
  if (csr_can1_tx_index2 == csr_can1_tx_index1)
  {
    VICIntEnClr = 1<<20; //Disable CAN1 TX interrupts
  }
  VICSoftIntClr = 1<<20;  //Clear any CAN1 TX software interrupt
  VICVectAddr = 0;    // Clear interrupt in VIC.
}
////////////////////////////////////////////////////////////////////////////

volatile CAN_FRAME csr_can2_tx_buffer[CSR_CAN_TX_BUFFER_SIZE];
volatile short int csr_can2_tx_index1 = 0;
volatile short int csr_can2_tx_index2 = 0;

////////////////////////////////////////////////////////////////////////////
//Push one CAN frame onto CAN2 transmit ring buffer
unsigned short int csr_can2_tx_push_frame(CAN_FRAME * frameptr)
{
  short int temp_index = csr_can2_tx_index1;
  
  csr_can2_tx_buffer[temp_index] = *frameptr;
  
  if (++temp_index == CSR_CAN_TX_BUFFER_SIZE) {temp_index = 0;}
  if (temp_index == csr_can2_tx_index2) {return 1;} //buffer full
  
  csr_can2_tx_index1 = temp_index;   //update index1
    
  VICSoftInt = 1<<21;   //Force CAN2 TX interrupt
  VICIntEnable = 1<<21; //Enable CAN2 TX interrupts
  
  return 0; //Success - data on buffer
}
////////////////////////////////////////////////////////////////////////////
void csr_can2_tx_isr(void) __irq
{
  while (csr_can2_tx_index2 != csr_can2_tx_index1)
  {
    if (C2SR & (1<<2)) // buffer 1 is available
    {   
      C2TFI1 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
      C2TID1 = csr_can2_tx_buffer[csr_can2_tx_index2].addr;   //get CAN_ID
      C2TDA1 = csr_can2_tx_buffer[csr_can2_tx_index2].payload.w.w1;   //load first data word
      C2TDB1 = csr_can2_tx_buffer[csr_can2_tx_index2].payload.w.w2;   //load second data word	
      C2CMR = 1<<5;     // select buffer 1 for transmit
      C2CMR = 1<<0;     // transmission request
      if (++csr_can2_tx_index2 == CSR_CAN_TX_BUFFER_SIZE) {csr_can2_tx_index2 = 0;}
    }
    else if (C2SR & (1<<10)) // buffer 2 is available
    {
      C2TFI2 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
      C2TID2 = csr_can2_tx_buffer[csr_can2_tx_index2].addr;   //get CAN_ID
      C2TDA2 = csr_can2_tx_buffer[csr_can2_tx_index2].payload.w.w1;   //load first data word
      C2TDB2 = csr_can2_tx_buffer[csr_can2_tx_index2].payload.w.w2;   //load second data word	
      C2CMR = 1<<6;     // select buffer 2 for transmit
      C2CMR = 1<<0;     // transmission request
      if (++csr_can2_tx_index2 == CSR_CAN_TX_BUFFER_SIZE) {csr_can2_tx_index2 = 0;}
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

volatile CAN_FRAME csr_can3_tx_buffer[CSR_CAN_TX_BUFFER_SIZE];
volatile short int csr_can3_tx_index1 = 0;
volatile short int csr_can3_tx_index2 = 0;

////////////////////////////////////////////////////////////////////////////
//Push one CAN frame onto CAN3 transmit ring buffer
unsigned short int csr_can3_tx_push_frame(CAN_FRAME * frameptr)
{
  short int temp_index = csr_can3_tx_index1;
  
  csr_can3_tx_buffer[temp_index] = *frameptr;
  
  if (++temp_index == CSR_CAN_TX_BUFFER_SIZE) {temp_index = 0;}
  if (temp_index == csr_can3_tx_index2) {return 1;} //buffer full
  
  csr_can3_tx_index1 = temp_index;   //update index1
    
  VICSoftInt = 1<<22;   //Force CAN3 TX interrupt
  VICIntEnable = 1<<22; //Enable CAN3 TX interrupts
  
  return 0; //Success - data on buffer
}
////////////////////////////////////////////////////////////////////////////
void csr_can3_tx_isr(void) __irq
{
  while (csr_can3_tx_index2 != csr_can3_tx_index1)
  {
    if (C3SR & (1<<2)) // buffer 1 is available
    {   
      C3TFI1 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
      C3TID1 = csr_can3_tx_buffer[csr_can3_tx_index2].addr;   //get CAN_ID
      C3TDA1 = csr_can3_tx_buffer[csr_can3_tx_index2].payload.w.w1;   //load first data word
      C3TDB1 = csr_can3_tx_buffer[csr_can3_tx_index2].payload.w.w2;   //load second data word	
      C3CMR = 1<<5;     // select buffer 1 for transmit
      C3CMR = 1<<0;     // transmission request
      if (++csr_can3_tx_index2 == CSR_CAN_TX_BUFFER_SIZE) {csr_can3_tx_index2 = 0;}
    }
    else if (C3SR & (1<<10)) // buffer 2 is available
    {
      C3TFI2 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
      C3TID2 = csr_can3_tx_buffer[csr_can3_tx_index2].addr;   //get CAN_ID
      C3TDA2 = csr_can3_tx_buffer[csr_can3_tx_index2].payload.w.w1;   //load first data word
      C3TDB2 = csr_can3_tx_buffer[csr_can3_tx_index2].payload.w.w2;   //load second data word	
      C3CMR = 1<<6;     // select buffer 2 for transmit
      C3CMR = 1<<0;     // transmission request
      if (++csr_can3_tx_index2 == CSR_CAN_TX_BUFFER_SIZE) {csr_can3_tx_index2 = 0;}
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

volatile CAN_FRAME csr_can4_tx_buffer[CSR_CAN_TX_BUFFER_SIZE];
volatile short int csr_can4_tx_index1 = 0;
volatile short int csr_can4_tx_index2 = 0;

////////////////////////////////////////////////////////////////////////////
//Push one CAN frame onto CAN4 transmit ring buffer
unsigned short int csr_can4_tx_push_frame(CAN_FRAME * frameptr)
{
  short int temp_index = csr_can4_tx_index1;
  
  csr_can4_tx_buffer[temp_index] = *frameptr;
  
  if (++temp_index == CSR_CAN_TX_BUFFER_SIZE) {temp_index = 0;}
  if (temp_index == csr_can4_tx_index2) {return 1;} //buffer full
  
  csr_can4_tx_index1 = temp_index;   //update index1
    
  VICSoftInt = 1<<23;   //Force CAN4 TX interrupt
  VICIntEnable = 1<<23; //Enable CAN4 TX interrupts
  
  return 0; //Success - data on buffer
}
////////////////////////////////////////////////////////////////////////////
void csr_can4_tx_isr(void) __irq
{
  while (csr_can4_tx_index2 != csr_can4_tx_index1)
  {
    if (C4SR & (1<<2)) // buffer 1 is available
    {   
      C4TFI1 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
      C4TID1 = csr_can4_tx_buffer[csr_can4_tx_index2].addr;   //get CAN_ID
      C4TDA1 = csr_can4_tx_buffer[csr_can4_tx_index2].payload.w.w1;   //load first data word
      C4TDB1 = csr_can4_tx_buffer[csr_can4_tx_index2].payload.w.w2;   //load second data word	
      C4CMR = 1<<5;     // select buffer 1 for transmit
      C4CMR = 1<<0;     // transmission request
      if (++csr_can4_tx_index2 == CSR_CAN_TX_BUFFER_SIZE) {csr_can4_tx_index2 = 0;}
    }
    else if (C4SR & (1<<10)) // buffer 2 is available
    {
      C4TFI2 = 1<<19;  //Data length 8, FF = 0, RTR = 0, PRIO = 0
      C4TID2 = csr_can4_tx_buffer[csr_can4_tx_index2].addr;   //get CAN_ID
      C4TDA2 = csr_can4_tx_buffer[csr_can4_tx_index2].payload.w.w1;   //load first data word
      C4TDB2 = csr_can4_tx_buffer[csr_can4_tx_index2].payload.w.w2;   //load second data word	
      C4CMR = 1<<6;     // select buffer 2 for transmit
      C4CMR = 1<<0;     // transmission request
      if (++csr_can3_tx_index2 == CSR_CAN_TX_BUFFER_SIZE) {csr_can3_tx_index2 = 0;}
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

////////////////////////////////////////////////////////////////////////////
//initialize CAN ring buffers
void csr_init_can_rings(void)
{
  can_ring_init(&rx_ring,rx_frame_buf,RX_FRAME_BUF_LEN);
  can_rx_set_chan_cfg(CHAN_CAN1, (volatile unsigned long *)0xE0044000, &rx_ring, CAN_DISPATCH_MANUAL); //CAN controller 1
  can_rx_set_chan_cfg(CHAN_CAN2, (volatile unsigned long *)0xE0048000, &rx_ring, CAN_DISPATCH_MANUAL); //CAN controller 2
  can_rx_set_chan_cfg(CHAN_CAN3, (volatile unsigned long *)0xE004C000, &rx_ring, CAN_DISPATCH_MANUAL); //CAN controller 3
  can_rx_set_chan_cfg(CHAN_CAN4, (volatile unsigned long *)0xE0050000, &rx_ring, CAN_DISPATCH_MANUAL); //CAN controller 4
 
  can_ring_init(&tx1_ring,tx1_frame_buf,TX_FRAME_BUF_LEN);				//CAN controller 1 transmit
  can_ring_init(&tx2_ring,tx2_frame_buf,TX_FRAME_BUF_LEN);				//CAN controller 2 transmit
  can_ring_init(&tx3_ring,tx3_frame_buf,TX_FRAME_BUF_LEN);				//CAN controller 3 transmit
  can_ring_init(&tx4_ring,tx4_frame_buf,TX_FRAME_BUF_LEN);				//CAN controller 4 transmit

  can_tx_set_chan_cfg(CHAN_CAN1, (volatile unsigned long *)0xE0044000, &tx1_ring); //CAN controller 1
  can_tx_set_chan_cfg(CHAN_CAN2, (volatile unsigned long *)0xE0048000, &tx2_ring); //CAN controller 2
  can_tx_set_chan_cfg(CHAN_CAN3, (volatile unsigned long *)0xE004C000, &tx3_ring); //CAN controller 3
  can_tx_set_chan_cfg(CHAN_CAN4, (volatile unsigned long *)0xE0050000, &tx4_ring); //CAN controller 4
}
  ////////////////////////////////////////////////////////////////////////////





//Take incoming data from SSP receive buffer and CAN receive buffer.
//Call csr_route_frame to distribute it to the correct locations
void csr_route(void)
{
  CAN_FRAME frame;

  //pop one CAN frame pointer from ssp; route if available
  if (!csr_pop_ssp_frame(&frame))
  {
    csr_route_frame(&frame);
  }

  //pop one CAN frame from CAN receive buffer; route if available
  if (!can_ring_pop(&rx_ring, &frame))
  {
    frame.addr += 20;    // **** TEST CODE ****
    csr_route_frame(&frame);
  }
}

//Route one received CAN frame
unsigned short int csr_route_frame(CAN_FRAME * frameptr)
{
  unsigned char route;
  unsigned short int buffer_full = 0;

  if (frameptr->rtr)
  {
    //Subscribe CAN/SSP channel "chan" to this CAN_ID/address
	  CSR_Routing_Table[frameptr->addr] |= (1 << frameptr->chan);	
  }

  else	//route frame
  {
    route = CSR_Routing_Table[frameptr->addr];	//Get routing information from lookup table
  
  //Push frame onto destination bus ring buffers,
  //according to entries in routing table  
	
    if (route & 1<<CHAN_SSP)
    {
   	  frameptr->chan = CHAN_SSP;
      buffer_full = csr_push_ssp_frame(frameptr);;
    }

    if (route & 1<<CHAN_CAN1)
    {
   	  buffer_full = csr_can1_tx_push_frame(frameptr);
    }

    if (route & 1<<CHAN_CAN2)
    {
      buffer_full = csr_can2_tx_push_frame(frameptr);
    }

    if (route & 1<<CHAN_CAN3)
    {
      buffer_full = csr_can3_tx_push_frame(frameptr);
    }

    if (route & 1<<CHAN_CAN4)
    {
   	  buffer_full = csr_can4_tx_push_frame(frameptr);
    }
  }
  return(buffer_full);
}


//SSP interrupt service routine for CAN -  SSP router
 //__irq void csr_ssp_isr(void)
void csr_ssp_isr(void)
{	
  static short int rx_index = 0, tx_index = 0, checksum = 0, send_zeroes = 0;  
  short unsigned int dummy; //Dump excess received data here
    
  //Timer1 isr
	if (VICFIQStatus & 1<<5)
  {
    //Update read buffer indexes
    rx_index = 0;
    if (++csr_rx_index1 == CSR_RX_SEG_NUM) {csr_rx_index1 = 0;}
    
    //Check for buffer overflow
    if (csr_rx_index1 == csr_rx_index2)
    {
      //overflow - go back to previous segment
      if (--csr_rx_index1 < 0) {csr_rx_index1 = CSR_RX_SEG_NUM - 1;}
      FIO1CLR = 1<<24;  // **** TEST CODE **** turn on red LED
      //error - read buffer overflow; will overwrite previously received segment
    } 
    
    //Update transmit variables
    checksum = 1; //Checksum is total plus one, so checksum of zero is non-zero
    tx_index = 0;
    send_zeroes = 0;
    
    //Update interrupts
    SSPIMSC = 0xF;		//enable all SSP interrupts
    T1IR = 0xFF;  // Clear all Timer1 interrupt flags.
  }
  
  while(SSPSR & (1<<2))   //while RX FIFO not empty, read half-words - drain hardware buffer
  {
    if (rx_index < CSR_SEG_SIZE)
    {
      csr_rx_buffer[csr_rx_index1][rx_index] = SSPDR;
      ++rx_index;
    }
    else
    {
      dummy = SSPDR;
    }
	}   

  //SSP transmitter not full and SSP transmit interrupts enabled
	while ((SSPSR & 1<<1) && (SSPIMSC & (1<<3)))
  {
    //Software transmit buffer is empty - pad the rest of the transmit segment with zeroes
    if (csr_tx_index2 == csr_tx_index1) {send_zeroes = 1;}
            
    if (tx_index < CSR_SEG_SIZE - 1) //send segment
    {
      if (!send_zeroes) //send data
      {
        SSPDR = csr_tx_buffer[csr_tx_index2];
        checksum += csr_tx_buffer[csr_tx_index2];
        if (++csr_tx_index2 == CSR_TX_BUFFER_SIZE) {csr_tx_index2 = 0;}
      }
      else          //send zeroes to pad out segment
      {
        SSPDR = 0;
      }
    }
    else if (tx_index == CSR_SEG_SIZE - 1)  //Send checksum
    {
      SSPDR = checksum;
      SSPIMSC = (SSPIMSC & ~(1<<3)) & (0xF);    //disable SSP transmit interrupts
    }
//    else if (tx_index == CSR_SEG_SIZE)  //Send extra zero to purge buffers; end transmit
 //   {
 //     SSPIMSC = (SSPIMSC & ~(1<<3)) & (0xF);    //disable SSP transmit interrupts
 //     SSPDR = 0;
  //  }
    else
    {
      //error
    }
    
    ++tx_index; 
  }
 
  if (SSPMIS & (1<<1))	//receive timeout interrupt)
  {
	  //Reading leftover data from the buffer should have taken place above
    SSPICR = (1<<1);	//Clear receive time-out interrupt 
  }
  
  if (SSPMIS & (1<<0))	//RX FIFO overflow
  {
    FIO1CLR = 1<<24;  // **** TEST CODE **** turn on red LED
    SSPICR = (1<<0);	//Clear receive overflow interrupt 
   //error - SSP hardware receive buffer overflow
  }
  VICVectAddr = 0;  //"reset VIC priority hardware"
}
////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
//SSP receive data parser
unsigned short int csr_pop_ssp_frame(CAN_FRAME * frameptr)
{
	static unsigned short int temp1, temp2, temp3, temp4, j = 0;
	unsigned short int i, cal_checksum, no_data;

  
  no_data = 1;
  //Begin parsing if buffer has data available
  if (csr_rx_index2 != csr_rx_index1)
  {
    //New segment? Calculate checksum.
    if (j == 0)
    {
      while (1)
      {
        //Calculate checksum if first segment element is non-zero
        if (csr_rx_buffer[csr_rx_index2][0] != 0)
        {
//          temp1 = csr_rx_buffer[csr_rx_index2][0];  // **** TEST CODE ****
          temp1 = csr_rx_index2;  // **** TEST CODE ****
          cal_checksum = 1;
          for (i = 0; i < CSR_SEG_SIZE - 1; ++i)
          {
            cal_checksum += csr_rx_buffer[csr_rx_index2][i];
          }
          temp2 = csr_rx_index2;  // **** TEST CODE ****
          temp3 = cal_checksum;   // **** TEST CODE ****
          if (cal_checksum == csr_rx_buffer[csr_rx_index2][CSR_SEG_SIZE - 1])
          {
            FIO1CLR = 1<<23;  // **** TEST CODE **** turn on green LED
            goto PARSE;    //checksum is good, continue after while statement
          }
          else
          {
           //error - bad checksum
            temp4 = csr_rx_index2;  // **** TEST CODE ****
           
            FIO1CLR = 1<<24;  // **** TEST CODE **** turn on red LED
          }
        }
            
        //Advance to next segment and try again
        if (csr_rx_index2 == CSR_RX_SEG_NUM - 1)
        {
          csr_rx_index2 = 0;
        }
        else
        {
          ++csr_rx_index2;
        }
        
        //Check that buffer is not empty
        if (csr_rx_index2 == csr_rx_index1)
        {
          return no_data; //Buffer empty
        }
      }
    }
  
    PARSE:
    
    //Look for valid data address
    if ((csr_rx_buffer[csr_rx_index2][j] & 0xF800) == 0x7800)
    {
      //Should have valid data at this point, if if gets this far
      frameptr->chan = CHAN_SSP;							//identifies source of packet to router
	    frameptr->addr = csr_rx_buffer[csr_rx_index2][j]; //Bottom 11 bits are CAN ID (address); .addr is 11-bit field
	   // frameptr->dlc = ~(csr_rx_buffer[csr_rx_index2][j] >> 11); //Next 4 bits are data length code; .dlc is 4-bit field
	    frameptr->dlc = 8;  //Only 8-byte payloads supported at this time (and rtr?)
      frameptr->rtr = csr_rx_buffer[csr_rx_index2][j++] >> 15; //Top bit is remote transmission request; .rtr is 1-bit field
	    frameptr->payload.s.s2 = csr_rx_buffer[csr_rx_index2][j++];  //copy payload data
	    frameptr->payload.s.s1 = csr_rx_buffer[csr_rx_index2][j++];
	    frameptr->payload.s.s4 = csr_rx_buffer[csr_rx_index2][j++];
	    frameptr->payload.s.s3 = csr_rx_buffer[csr_rx_index2][j++];
      no_data = 0;
    }
    else if ((csr_rx_buffer[csr_rx_index2][j] & 0xF800) == 0xF800)
    {
      //remote transfer request (RTR)
      j += 5;
    }
    else  //not a valid data packet, skip the rest of the segment
    {
      j = CSR_SEG_SIZE - 1;
    }
  
    if (j == CSR_SEG_SIZE - 1)
    {
      //Advance to next segment
      if (csr_rx_index2 == CSR_RX_SEG_NUM - 1)
      {
        csr_rx_index2 = 0;
      }
      else
      {
        ++csr_rx_index2;
      }
      
      //reset j to 0
      j = 0;
    }
    return no_data;   //Valid return data 
  }
  else
  {
    return no_data;   //Receive buffer empty
  }
}

unsigned short int csr_push_ssp_frame(CAN_FRAME * frame)
{
  short int temp_index1 = csr_tx_index1;
  
  //load data from incoming frame
  csr_tx_buffer[temp_index1] = frame->addr  //CAN ID (address)
  | (0xF << 11)		   					//extra bits for future
	| (frame->rtr << 15)		//remote transmission request bit
	;
  
  //Advance temp_index to next available location
  if (++temp_index1 == CSR_TX_BUFFER_SIZE)  {temp_index1 = 0;}
  //Check for buffer overflow
  if (temp_index1 == csr_tx_index2) {return 1;} //buffer full

	csr_tx_buffer[temp_index1] = frame->payload.s.s2;	//Load first short int of CAN payload
  
  //Advance temp_index to next available location
  if (++temp_index1 == CSR_TX_BUFFER_SIZE)  {temp_index1 = 0;}
  //Check for buffer overflow
  if (temp_index1 == csr_tx_index2) {return 1;} //buffer full

	csr_tx_buffer[temp_index1] = frame->payload.s.s1;	//Load second short int of CAN payload
  
  //Advance temp_index to next available location
  if (++temp_index1 == CSR_TX_BUFFER_SIZE)  {temp_index1 = 0;}
  //Check for buffer overflow
  if (temp_index1 == csr_tx_index2) {return 1;} //buffer full

  csr_tx_buffer[temp_index1] = frame->payload.s.s4;	//Load third short int of CAN payload
  
  //Advance temp_index to next available location
  if (++temp_index1 == CSR_TX_BUFFER_SIZE)  {temp_index1 = 0;}
  //Check for buffer overflow
  if (temp_index1 == csr_tx_index2) {return 1;} //buffer full
  
  csr_tx_buffer[temp_index1] = frame->payload.s.s3;	//Load fourth short int of CAN payload
  
  //Advance temp_index to next available location
  if (++temp_index1 == CSR_TX_BUFFER_SIZE)  {temp_index1 = 0;}
  //Check for buffer overflow
  if (temp_index1 == csr_tx_index2) {return 1;} //buffer full
  
  csr_tx_index1 = temp_index1;    //update csr_tx_index1 and make new data available to isr
  
		     
  return 0; //Frame added successfully          
}
