//Code for SSP - CAN router. CAN frames are received from up to 4 CAN buses and the SSP bus,
//and directed out again as per the bits set in the routing table. Each possible standard CAN address
//from 0 to 2031 (CAN does not permit addresses 2032 through 2047) gets its own routing entry (one byte)
//in the CSR_Routing_Table lookup table. Bit 0 (LSB) indicates that SSP (the main brain) is to receive a
//frame; bit 1 through 4 are for CAN buses 1 through 4, respectively.

#include <includes.h>



//CAN packet routing table. This can be filled in manually, or by having each
//processor "subscribe" to data types it wants to receive by sending out a request to
//receive (RTR) packet for each data type. Max routing table size would be 11 bits (2048),
//except that CAN does not allow the first (most significant) 7 bits to be all high (recessive), thus limiting
//the top CAN address in standard mode to 0b11111101111 = 2031. (CAN20B.pdf page 13)

#define CSR_ROUTING_TABLE_SIZE 2032
CSR_ROUTE csr_routing_table[CSR_ROUTING_TABLE_SIZE];

/*
//////////////////////////////////////////////////////////////////////////////////////////////////////
void can_test_transmitter(void)
{
 CAN_FRAME frame;
 unsigned short buffer_full = 1;

 frame.rtr = 0;
 frame.dlc = 8;
 frame.payload.w.w1 = 0xAAAA;
 frame.payload.w.w2 = 0x5555;

 //CAN controller 1
 frame.addr = 1;
 frame.chan = CHAN_CAN1;
 buffer_full = can_transmit_frame(&frame);

 //CAN controller 2
 frame.addr = 2;
 frame.chan = CHAN_CAN2;
 buffer_full = can_transmit_frame(&frame);

 //CAN controller 3
 frame.addr = 3;
 frame.chan = CHAN_CAN3;
 buffer_full = can_transmit_frame(&frame);

 //CAN controller 4
 frame.addr = 4;
 frame.chan = CHAN_CAN4;
 buffer_full = can_transmit_frame(&frame);
}
////////////////////////////////////////////////////////////////////////////////////////////////////
*/

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void csr_routing_table_init(void)
{
unsigned short i;

for (i=0; i<CSR_ROUTING_TABLE_SIZE; ++i)
{
  //csr_routing_table[i].destinations = 0;    // default is 0: no data destinations yet included
  csr_routing_table[i].destinations = 1 << CHAN_SSP;    // Start with all data going to SSP; may use rtr later with SSP too.
                                                        // Now have a check to prevent data from being sent back to its source
  csr_routing_table[i].source = 0x7;        // default is 7: no data source yet determined
}

  #include <csr_routing_table.c> // Created by CAN ID parsing script
                          
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Route one received CAN frame
unsigned short int csr_route_frame(CAN_FRAME * frameptr)
{
  unsigned char dest, src;
  unsigned short buffer_full = 0;

  dest = csr_routing_table[frameptr->addr].destinations;	//Get routing information from lookup table
  src = csr_routing_table[frameptr->addr].source;

  if (frameptr->rtr)  // Initiate RTR processing; 
  {
    //Subscribe CAN/SSP channel "chan" to this CAN_ID/address
	  csr_routing_table[frameptr->addr].destinations |= (1 << frameptr->chan);
    
    //Activate below code if needed: Additional RTR handling - send RTR frame to data source
    //(if not also the source of the RTR frame)
    //If data source is not yet defined, send to all except RTR source
    /*
    //Remote Transfer Request (RTR) frame handling
    if (src != 0x7)
    {
      if (src != frameptr->chan)
      {
        frameptr->chan = src;
        buffer_full = csr_push_ssp_frame(frameptr);
      }  
    }
    else  //send to all but RTR source
    {
      if (frameptr->chan != CHAN_SSP)
      {
        frameptr->chan = CHAN_SSP;
        buffer_full = csr_push_ssp_frame(frameptr);
      }
    
      if (frameptr->chan != CHAN_CAN1)
      {    
   	    frameptr->chan = CHAN_CAN1;
        buffer_full = can_transmit_frame(frameptr);
      }

      if (frameptr->chan != CHAN_CAN2)
      {      
   	    frameptr->chan = CHAN_CAN2;
        buffer_full = can_transmit_frame(frameptr);
      }

      if (frameptr->chan != CHAN_CAN3)
      {    
   	    frameptr->chan = CHAN_CAN3;      
        buffer_full = can_transmit_frame(frameptr);
      }

      if (frameptr->chan != CHAN_CAN4)
      {    
   	    frameptr->chan = CHAN_CAN4;   	  
        buffer_full = can_transmit_frame(frameptr);
      }
    }
    */ 	
  }

  else	//route frame
  {
  
    //Check data source validity.
    //Only one data source is permitted per data_id
  
    if (src != frameptr->chan) // New source channel for this DATA_ID
    {
      if (src == 0x7)   // 0x7 is default initialized value, updated when data received
      {
        csr_routing_table[frameptr->addr].source = frameptr->chan;    // Update routing table to include new source
        src = frameptr->chan;
      }
      else  // ERROR - source has changed, or there are multiple sources for frames with this DATA_ID
      {
        // ERROR call
        csr_routing_table[frameptr->addr].source = frameptr->chan;    // Update routing table to use new source instead
        src = frameptr->chan;
      }
    }
  
    //Push frame onto destination bus ring buffers,
    //according to entries in routing table  
	
      if ((dest & 1<<CHAN_SSP) && (src != CHAN_SSP))
      {
        frameptr->chan = CHAN_SSP;
        buffer_full = csr_push_ssp_frame(frameptr);
      }
    
      if ((dest & 1<<CHAN_CAN1) && (src != CHAN_CAN1))
      {    
   	    frameptr->chan = CHAN_CAN1;
        buffer_full = csr_can1_tx_push_frame(frameptr);
        //buffer_full = can_transmit_frame(frameptr);
      }

      if ((dest & 1<<CHAN_CAN2) && (src != CHAN_CAN2))
      {      
   	    frameptr->chan = CHAN_CAN2;
        buffer_full = csr_can2_tx_push_frame(frameptr);
        //buffer_full = can_transmit_frame(frameptr);
      }

      if ((dest & 1<<CHAN_CAN3) && (src != CHAN_CAN3))
      {    
   	    frameptr->chan = CHAN_CAN3;
        buffer_full = csr_can3_tx_push_frame(frameptr);              
       // buffer_full = can_transmit_frame(frameptr);
      }

      if ((dest & 1<<CHAN_CAN4) && (src != CHAN_CAN4))
      {    
   	    frameptr->chan = CHAN_CAN4; 
        buffer_full = csr_can4_tx_push_frame(frameptr);  	  
        //buffer_full = can_transmit_frame(frameptr);
      }
    }
  return(buffer_full);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
