#include <mb_includes.h>

//Global variables

#define MB_CSR_DMA_VAR_ARRAY_SIZE 4096
long unsigned int * mb_variable_pointers[MB_CSR_DMA_VAR_ARRAY_SIZE];
long unsigned int mb_temp_variables[MB_CSR_DMA_VAR_ARRAY_SIZE];
long unsigned int mb_timestamps[MB_CSR_DMA_VAR_ARRAY_SIZE];

CAN_FRAME Example_Frame; //**** TEST CODE ****
volatile long unsigned int Total_Packet_Counter = 0; //**** TEST CODE ****
volatile long unsigned int Sent_Packet_Counter = 0;  //**** TEST CODE ****

////////////////////////////////////////////////////////////////
//Global variables for SSP/DMA software receive buffer//////////
////////////////////////////////////////////////////////////////

#define MB_CSR_DMA_RB_SIZE 6000	  			//DMA receive ring buffer size

unsigned volatile short int MB_CSR_DMA_Rec_Buffer[MB_CSR_DMA_RB_SIZE];
unsigned volatile short int MB_CSR_DMA_Rec_Index1 = 0;		//Points to starting address of DMA packet being written by DMA
unsigned volatile short int MB_CSR_DMA_Rec_Index2 = 0;		//Points to starting address of DMA packet being read by software
unsigned volatile short int MB_CSR_DMA_RB_Index = 0;			//Points to oldest unread data in SSP-DMA read buffer.
													//When = to DMA packet size * MB_CSR_DMA_Rec_Index1, no data left in buffer
/////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
//Global variables for SSP/DMA software transmit buffer//////////
////////////////////////////////////////////////////////////////

//The initial design is for SSP packet lengths of 6.
//The length of the transmit buffer must be an integral number of packet lengths
//i.e., MB_CSR_DMA_TB_SIZE = 6 * total number of packets to store

#define MB_CSR_DMA_TB_SIZE 60	  		//= packet size (6) * total packet quantity
											//i.e., must be a multiple of 6
unsigned volatile short int MB_CSR_DMA_TX_Buffer[MB_CSR_DMA_TB_SIZE];
unsigned volatile short int MB_CSR_DMA_TB_Index1 = 0;		//Points to starting address of data being written by software
unsigned volatile short int MB_CSR_DMA_TB_Index2 = 0;		//Points to starting address of DMA segment being transmitted
/////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////
//Copy pointers to variable_pointers array (test code version)
void mb_csr_dma_var_pointer_init(void)
{
  long unsigned int i;
  
  for (i = 0; i < MB_CSR_DMA_VAR_ARRAY_SIZE; ++i)
  {
    mb_variable_pointers[i] = &mb_temp_variables[i];
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//GPDMA isr; for all non-USB and non-ethernet DMA. E.g., SSP, SPI, serial, SD card
void mb_gpdma_isr(void)
{
  //SSP1 receive is on GPDMA channel 0 (highest priority)
  //SSP DMA receive terminal count interrupt - start new DMA transfer
  if (GPDMA_INT_TCSTAT & 1<<0)
  {
	  mb_csr_gpdma_rx0_isr();
  }


  //SSP1 transmit is on GPDMA channel 1 (second highest priority)
  //Update DMA CH1 transmit over SSP1
  if (GPDMA_INT_TCSTAT & 1<<1)	//Terminal count interrupt on DMA channel 1 (SSP1 transmit)
  {
    mb_csr_gpdma_tx1_isr();
  }
  //clear any unused GPDMA terminal count interrupts
  GPDMA_INT_TCCLR = 0xFC;
  
  //clear any unused GPDMA error interrupts
  GPDMA_INT_ERR_CLR = 0xFF;
}
///////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////
//DMA interrupt handler for SSP receive packets
short unsigned int mb_csr_dma_rx_dummy;
short unsigned int mb_csr_dma_rx_count = 0;

#define MB_CSR_DMA_RB_MIN 32     //Minimum DMA transfer size, except to fill buffer to highest index
                                  //If minimum would overfill buffer, a buffer overflow error is (could be)
                                  //generated. Keep the minimum well below the total buffer size.
                                  
#define MB_CSR_DMA_RB_MAX 256    //Maximum DMA transfer size. DMA transfer sizes are limited to this value
                                  //to control data latency. (The data in a transfer will not be available in this
                                  //function until the transfer is finished and the next terminal count interrupt
                                  //is serviced. 

void mb_csr_gpdma_rx0_isr(void)
{
//  P3_OUTP_SET = 1<<26;	//Test code - Turn on GPIO_1 output (red LED)

    //Update receive buffer index to reflect the just-completed DMA transfer,
    //making the new data available for reading
	  mb_csr_dma_rx_count += MB_CSR_DMA_Rec_Index1;	//Add most recent finished DMA transfer count
														//to ring buffer write index (index1)
	  if (mb_csr_dma_rx_count == MB_CSR_DMA_RB_SIZE) 
	  {
	    MB_CSR_DMA_Rec_Index1 = 0;	//index2 wraps around to zero
	  } 
	  else
	  {
	    MB_CSR_DMA_Rec_Index1 = mb_csr_dma_rx_count; //set index1 to new value
	  											  //Note that this happens after the wrap update
												    //is finished on the temporary index mb_csr_dma_rx_count
                            //to avoid race problems (not likely, though, since this is an isr)
	  }
 
    //Calculate next DMA transfer size
    if (MB_CSR_DMA_Rec_Index1 >= MB_CSR_DMA_Rec_Index2)
    {
	    //receive DMA transfer fills to end of read buffer
	    mb_csr_dma_rx_count = MB_CSR_DMA_RB_SIZE - MB_CSR_DMA_Rec_Index1;
    }

    else
    {
	    //receive DMA fills read buffer to one element before oldest unread short int (at location ...Index2)
	    mb_csr_dma_rx_count = MB_CSR_DMA_Rec_Index2 - MB_CSR_DMA_Rec_Index1;
      
      //buffer is too full to allow a complete transfer,
      //set up a dummy transfer to keep the DMA active until the
      //buffer clears. Send error!
      if (mb_csr_dma_rx_count < MB_CSR_DMA_RB_MIN)
      {
        mb_csr_dma_rx_count = 0;  //No data written to actual buffer this transfer
        GPDMA_CH0_SRC = (unsigned long)&SSP1DR;				//source is ssp1
        GPDMA_CH0_DEST = (unsigned long)&mb_csr_dma_rx_dummy;	//destination is dummy global variable
        //Set up GPDMA_CH1_CTRL. Do not OR bits with reserved bits!	Do not write ones to them, either.
 	      GPDMA_CH0_CTRL = MB_CSR_DMA_RB_MIN  //set up dummy receive of minimum transfer size
        | 1<<12				//Source burst size of 4
        | 1<<15				//Destination burst size of 4
        | 1<<18			 	//Source width (SSP) = 16 bits
        | 1<<21				//Destination width (memory) = 16 bits
        //| 1<<24				//source AHB bus master 1
        //| 1<<25				//destination AHB bus master 1
        //| 1<<26				//Increment the source address after each transfer
        //| 1<<27			//Increment the destination address after each transfer
        //28,29,30 reserved
        | ((unsigned long int)1<<31) 			//terminal count interrupt enable bit.
        ;
  
        //Set up GPDMA_CH0_CFG.	Do not OR bits with reserved bits!	Do not write ones to them, either.
        GPDMA_CH0_CFG = 1	//Enable CH0
        | (3<<1)			//Destination peripheral is SSP1 transmit
        | (2<<11)			//memory to peripheral, with DMA flow control
        // 000 gives memory to memory transfer
        //| (1<<14)			//Enable error interrupt
        | (1<<15)			//Enable terminal count interrupt
        //| (1<<16)			//Enable locked transfers
        ;
        GPDMA_INT_TCCLR = 1<<0; //Clear channel 0 terminal count interrupt
        //error
        P3_OUTP_SET = 1<<26;  // **** TEST CODE **** turn on red LED
        return;
      }
    }
	
  //Limit maximum segment size to control data latency
  if (mb_csr_dma_rx_count > MB_CSR_DMA_RB_MAX) {mb_csr_dma_rx_count = MB_CSR_DMA_RB_MAX;}
  
  //Start new DMA transfer
  GPDMA_CH0_SRC = (unsigned long)&SSP1DR;				//source is ssp1
  GPDMA_CH0_DEST = (unsigned long)&MB_CSR_DMA_Rec_Buffer[MB_CSR_DMA_Rec_Index1];	//destination is buffer in memory
  
  //Set up GPDMA_CH1_CTRL. Do not OR bits with reserved bits!	Do not write ones to them, either.
 	GPDMA_CH0_CTRL = mb_csr_dma_rx_count  //short ints to transmit
  | 1<<12				//Source burst size of 4
  | 1<<15				//Destination burst size of 4
  | 1<<18			 	//Source width (SSP) = 16 bits
  | 1<<21				//Destination width (memory) = 16 bits
  //| 1<<24				//source AHB bus master 1
  //| 1<<25				//destination AHB bus master 1
  //| 1<<26				//Increment the source address after each transfer
  | 1<<27			//Increment the destination address after each transfer
  //28,29,30 reserved
  | ((unsigned long int)1<<31) 			//terminal count interrupt enable bit.
  ;
  
  //Set up GPDMA_CH0_CFG.	Do not OR bits with reserved bits!	Do not write ones to them, either.
  GPDMA_CH0_CFG = 1	//Enable CH0
  | (3<<1)			//Destination peripheral is SSP1 transmit
  | (2<<11)			//memory to peripheral, with DMA flow control
  // 000 gives memory to memory transfer
  //| (1<<14)			//Enable error interrupt
  | (1<<15)			//Enable terminal count interrupt
  //| (1<<16)			//Enable locked transfers
  ;
  GPDMA_INT_TCCLR = 1<<0; //Clear channel 0 terminal count interrupt
}


/*
void mb_csr_gpdma_rx0_isr(void)
{
    MB_CSR_DMA_Rec_Index1 = (++MB_CSR_DMA_Rec_Index1) & (MB_CSR_DMA_PACK_QUAN - 1);	//increment dma packet write pointer
    if (MB_CSR_DMA_Rec_Index1 == MB_CSR_DMA_Rec_Index2)		//when write pointer = read pointer, buffer overflow has occurred
    {
  	  MB_CSR_DMA_Rec_Index1 = (--MB_CSR_DMA_Rec_Index1) & (MB_CSR_DMA_PACK_QUAN - 1);	//undo write pointer increment
	  //the next dma packet will overwrite the most recent one
	  //error messages
    }
  
    GPDMA_CH0_SRC = (unsigned long)&SSP1DR;		//source is ssp1
    GPDMA_CH0_DEST = (unsigned long)&MB_CSR_DMA_Rec_Buffer[MB_CSR_DMA_Rec_Index1<<MB_CSR_DMA_PACK_SIZE_EXP];  //destination is array in memory

    //Set up GPDMA_CH0_CTRL. Do not OR bits with reserved bits!	Do not write ones to them, either.
    GPDMA_CH0_CTRL = MB_CSR_DMA_PACK_SIZE	//DMA transfer size, lower 12 bits
    | 1<<12					//Source burst size of 4
    | 1<<15					//Destination burst size of 4
    | 1<<18			 		//Source width (SSP) = 16 bits
    | 1<<21					//Destination width (memory) = 16 bits
    //| 1<<24				//source AHB bus master 1
    //| 1<<25				//destination AHB bus master 1
    //| 1<<26				//Increment the source address after each transfer
    | 1<<27					//Increment the destination address after each transfer
    //28,29,30 reserved
    | ((unsigned long int)1<<31) 				//terminal count interrupt enable bit.
    ;

  	//Set up GPDMA_CH0_CFG.	Do not OR bits with reserved bits!	Do not write ones to them, either.
    GPDMA_CH0_CFG = 1//Enable CH0
    | (3<<1)		 //Source peripheral is SSP1 receive
    | (2<<11)		 //peripheral to memory, with DMA flow control
    // 000 gives memory to memory transfer
    //| (1<<14)		//Enable error interrupt
    | (1<<15)		//Enable terminal count interrupt
    //| (1<<16)		//Enable locked transfers
    ;
}

*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned short int MB_CSR_GPDMA_Count = 0;

const unsigned short int MB_CSR_DMA_Zeroes[6] = {0,0,0,0,0,0};

void mb_csr_gpdma_tx1_isr(void)
{
 // if (GPDMA_INT_TCSTAT & 1<<1)	//Terminal count interrupt on DMA channel 1 (SSP1 transmit)
 // {
//  P3_OUTP_SET = 1<<26;	//Test code - Turn on GPIO_1 output (red LED)
	  MB_CSR_GPDMA_Count += MB_CSR_DMA_TB_Index2;	//Add most recent finished DMA transfer count
														//to ring buffer read index (index2)
	  if (MB_CSR_GPDMA_Count == MB_CSR_DMA_TB_SIZE) 
	  {
	    MB_CSR_DMA_TB_Index2 = 0;	//index2 wraps around to zero
	  } 
	  else
	  {
	    MB_CSR_DMA_TB_Index2 = MB_CSR_GPDMA_Count; //set index2 to new value
	  											  //Note that this happens after the wrap update
												    //is finished on the temporary index MB_CSR_GPDMA_Count
	  }
//  }
	
  //Start new DMA transfer, but only if data is available, and if a DMA transfer is not already in progress
  //if ((MB_CSR_DMA_TB_Index2 != MB_CSR_DMA_TB_Index1) && (!(GPDMA_CH1_CFG & 1<<0)))
  if (MB_CSR_DMA_TB_Index2 != MB_CSR_DMA_TB_Index1)
  {
    GPDMA_CH1_SRC = (unsigned long)&MB_CSR_DMA_TX_Buffer[MB_CSR_DMA_TB_Index2];	//source is transmit buffer in memory
    GPDMA_CH1_DEST = (unsigned long)&SSP1DR;									//destination is ssp1
	
	
    if (MB_CSR_DMA_TB_Index2 > MB_CSR_DMA_TB_Index1)
    {
	    //Transmit DMA segment goes to end of buffer
	    MB_CSR_GPDMA_Count = MB_CSR_DMA_TB_SIZE - MB_CSR_DMA_TB_Index2;
    }

    else
    {
	    //Transmit DMA segment includes all data remaining in buffer
	    MB_CSR_GPDMA_Count = MB_CSR_DMA_TB_Index1 - MB_CSR_DMA_TB_Index2;
    }

	//Unless the segment would be too big for the GPDMA hardware.
	//The maximum DMA segment length is 4095; maximum integral multiple of 6 is 4092
	if (MB_CSR_GPDMA_Count > 4092) {MB_CSR_GPDMA_Count = 4092;}

 //   U5THR = (MB_CSR_GPDMA_Count + 0x40) & 0xFF;  //**** TEST CODE ****
//    P3_OUTP_SET = 1<<26;	//Test code - Turn on GPIO_1 output (red LED)

    //Set up GPDMA_CH1_CTRL. Do not OR bits with reserved bits!	Do not write ones to them, either.
 	  GPDMA_CH1_CTRL = MB_CSR_GPDMA_Count  //short ints to transmit
    | 1<<12				//Source burst size of 4
    | 1<<15				//Destination burst size of 4
    | 1<<18			 	//Source width (SSP) = 16 bits
    | 1<<21				//Destination width (memory) = 16 bits
    | 1<<24				//source AHB bus master 1
    | 1<<25				//destination AHB bus master 1
    | 1<<26				//Increment the source address after each transfer
    //| 1<<27			//Increment the destination address after each transfer
    //28,29,30 reserved
    | ((unsigned long int)1<<31) 			//terminal count interrupt enable bit.
    ;
  
  //Set up GPDMA_CH1_CFG.	Do not OR bits with reserved bits!	Do not write ones to them, either.
  GPDMA_CH1_CFG = 1	//Enable CH1
  | (11<<6)			//Destination peripheral is SSP1 transmit
  | (1<<11)			//memory to peripheral, with DMA flow control
  // 000 gives memory to memory transfer
  //| (1<<14)			//Enable error interrupt
  | (1<<15)			//Enable terminal count interrupt
  //| (1<<16)			//Enable locked transfers
  ;
  }
  
  //If no data, send array of zeroes over DMA/SSP
  else if ((MB_CSR_DMA_TB_Index2 == MB_CSR_DMA_TB_Index1) && (!(GPDMA_CH1_CFG & 1<<0)))
  {
    GPDMA_CH1_SRC = (unsigned long)&MB_CSR_DMA_Zeroes[0];	//source is array of zeroes
    GPDMA_CH1_DEST = (unsigned long)&SSP1DR;							//destination is ssp1
    
    MB_CSR_GPDMA_Count = 0;                               //No actual data is sent in this DMA segment;
                                                          //Don't change the ring buffer index at segment
                                                          //completion (terminal count)
    
  //  P3_OUTP_SET = 1<<26;	//Test code - Turn on GPIO_1 output (red LED)
  
    //Set up GPDMA_CH1_CTRL. Do not OR bits with reserved bits!	Do not write ones to them, either.
 	  GPDMA_CH1_CTRL = 6  //number of short ints to transmit
    | 1<<12				//Source burst size of 4
    | 1<<15				//Destination burst size of 4
    | 1<<18			 	//Source width (SSP) = 16 bits
    | 1<<21				//Destination width (memory) = 16 bits
    | 1<<24				//source AHB bus master 1
    | 1<<25				//destination AHB bus master 1
    | 1<<26				//Increment the source address after each transfer
    //| 1<<27			//Increment the destination address after each transfer
    //28,29,30 reserved
    | ((unsigned long int)1<<31) 			//terminal count interrupt enable bit.
    ;
  
    //Set up GPDMA_CH1_CFG.	Do not OR bits with reserved bits!	Do not write ones to them, either.
    GPDMA_CH1_CFG = 1	//Enable CH1
    | (11<<6)			//Destination peripheral is SSP1 transmit
    | (1<<11)			//memory to peripheral, with DMA flow control
    // 000 gives memory to memory transfer
    //| (1<<14)			//Enable error interrupt
    | (1<<15)			//Enable terminal count interrupt
    //| (1<<16)			//Enable locked transfers
    ;
  }
  GPDMA_INT_TCCLR = 1<<1; //Clear channel 1 terminal count interrupt
}

/*
//Pop a data packet from the SSP receive data DMA ring buffer.
//The receive data DMA buffer array, unlike the transmit buffer,
//is divided into fixed-length DMA segments. ...Index1 and ...Index2 refer
//to the segment numbers; ...Index points to the individual half-word (16-bit) data.
unsigned short int mb_ssp_pop_frame(CAN_FRAME * frame)
{
	#define MB_CSR_PACKET_LENGTH 6		//Number of 16-bit half-words in packet
	#define MB_CSR_ADDRESS_8 0x4000		//Address half-words should have the following leading
										//(MSB) bits: 01000, for an 8-byte CAN packet.
										//The MSB is defined to always be 0 for a data frame;
										//(1 for remote transmission request frame)
										//the next four bits give the data size, up to 8 bytes.
										//At present this code only implements 8-byte data packets.

  unsigned short int i, cal_checksum, rec_checksum, increment_index2, temp_index, rollover;
  unsigned short int no_data_found;
		
  //Some data left in SSP-DMA read buffer?
  //Note that Index1 must be multiplied by MB_CSR_DMA_PACK_SIZE for comparison with Index
  if (MB_CSR_DMA_RB_Index != MB_CSR_DMA_Rec_Index1<<MB_CSR_DMA_PACK_SIZE_EXP)
  {
	//Shift right by 11 bits to compare only the top 5 bits. Future: just see if zero after shifting?
//	  while	((MB_CSR_DMA_Rec_Buffer[MB_CSR_DMA_RB_Index] >> 11)!= (MB_CSR_ADDRESS_8 >> 11))	//invalid rtr/dlc
	  while	(!MB_CSR_DMA_Rec_Buffer[MB_CSR_DMA_RB_Index])	//while zero
	  {
	    MB_CSR_DMA_RB_Index = (MB_CSR_DMA_RB_Index + 1) & (MB_CSR_DMA_RB_SIZE- 1); 	//Move to next read buffer location
																					//Wrap around to zero as needed
		//End of DMA packet?
		if (!(MB_CSR_DMA_RB_Index & (MB_CSR_DMA_PACK_SIZE - 1))) 
//		if (!(MB_CSR_DMA_RB_Index << (16 - MB_CSR_DMA_PACK_SIZE_EXP)))
		  {
		  //At end of read buffer?
		    if (MB_CSR_DMA_RB_Index == MB_CSR_DMA_Rec_Index1<<MB_CSR_DMA_PACK_SIZE_EXP)
		  	{
			  MB_CSR_DMA_RB_Index = (MB_CSR_DMA_RB_Index - 1) & (MB_CSR_DMA_RB_SIZE - 1); 	//return to previous read buffer location
		      return(1); //Receive buffer is empty
		  	}
			else //Go on to next DMA segment if needed; allow writing of previous segment
			{
			  MB_CSR_DMA_Rec_Index2 = (MB_CSR_DMA_Rec_Index2 + 1) & (MB_CSR_DMA_PACK_QUAN - 1);
			}
		}
	  }														//

	//Address half-word found, parse packet
	//for multiple lengths of CAN payloads, there would be an if-else here for each length/address prefix

		//Calculate checksum of potential packet
		cal_checksum = 0; increment_index2 = 0; rollover = 0; no_data_found = 0;
		temp_index = MB_CSR_DMA_RB_Index;
		for (i = 0; i < (MB_CSR_PACKET_LENGTH-1); ++i)
		{
			cal_checksum += MB_CSR_DMA_Rec_Buffer[temp_index];			//calculate checksum
			temp_index = (temp_index + 1) & (MB_CSR_DMA_RB_SIZE - 1); 	//Move to next read buffer location

		   	//End of DMA packet?
			if (!(temp_index & (MB_CSR_DMA_PACK_SIZE - 1))) 
//			if (!(MB_CSR_DMA_RB_Index << (16 - MB_CSR_DMA_PACK_SIZE_EXP)))
			{
		  	  if (temp_index == (MB_CSR_DMA_Rec_Index1 << MB_CSR_DMA_PACK_SIZE_EXP))
		  	  {
		        return(1); //At end of data -- stop parsing. Leave RB index unchanged. No data
		  	  }
			  else
			  {
			    increment_index2 = 1; 	//Tentative - finished a DMA packet, clear slot for new incoming data
			  							//at end of function if checksum works out
  				if (temp_index == 0) //temp_index wraps around at end of buffer
  				{
  				  rollover = 1;		//This is a flag passed to a data output function,
  				  					//along with a pointer to the receive buffer,
  									//to notify the function that it has to work around the rollover
  				}
			  }
			}				
		}
		
		cal_checksum = ~cal_checksum;			//bit-wise negation (checksum of zero should not be zero)
		
		//read in received checksum from end of potential packet
		rec_checksum = MB_CSR_DMA_Rec_Buffer[temp_index];
				
		//Valid packet found? Put received values into CAN frame
		if (cal_checksum == rec_checksum)
		{	
			if (rollover) //receive buffer rolls over to zero, check for it with each increment
			{
			  frame->addr = MB_CSR_DMA_Rec_Buffer[MB_CSR_DMA_RB_Index];	//Bottom 11 bits are CAN ID (address); .addr is 11-bit field
			  frame->dlc =  ~(MB_CSR_DMA_Rec_Buffer[MB_CSR_DMA_RB_Index] >> 11);//Next 4 bits are data length code; .dlc is 4-bit field
			  frame->rtr = MB_CSR_DMA_Rec_Buffer[MB_CSR_DMA_RB_Index] >> 15; //Top bit is remote transmission request; .rtr is 1-bit field
			  frame->payload.s.s1 = MB_CSR_DMA_Rec_Buffer[(MB_CSR_DMA_RB_Index + 1) & (MB_CSR_DMA_RB_SIZE - 1)];
			  frame->payload.s.s2 = MB_CSR_DMA_Rec_Buffer[(MB_CSR_DMA_RB_Index + 2) & (MB_CSR_DMA_RB_SIZE - 1)];
			  frame->payload.s.s3 = MB_CSR_DMA_Rec_Buffer[(MB_CSR_DMA_RB_Index + 3) & (MB_CSR_DMA_RB_SIZE - 1)];
			  frame->payload.s.s4 = MB_CSR_DMA_Rec_Buffer[(MB_CSR_DMA_RB_Index + 4) & (MB_CSR_DMA_RB_SIZE - 1)];
			}
			else
			{
			  frame->addr = MB_CSR_DMA_Rec_Buffer[MB_CSR_DMA_RB_Index];	//Bottom 11 bits are CAN ID (address); .addr is 11-bit field
			  frame->dlc =  ~(MB_CSR_DMA_Rec_Buffer[MB_CSR_DMA_RB_Index] >> 11);//Next 4 bits are data length code; .dlc is 4-bit field
			  frame->rtr = MB_CSR_DMA_Rec_Buffer[MB_CSR_DMA_RB_Index] >> 15;//Top bit is remote transmission request; .rtr is 1-bit field
			  frame->payload.s.s1 = MB_CSR_DMA_Rec_Buffer[MB_CSR_DMA_RB_Index + 1];
			  frame->payload.s.s2 = MB_CSR_DMA_Rec_Buffer[MB_CSR_DMA_RB_Index + 2];
			  frame->payload.s.s3 = MB_CSR_DMA_Rec_Buffer[MB_CSR_DMA_RB_Index + 3];
			  frame->payload.s.s4 = MB_CSR_DMA_Rec_Buffer[MB_CSR_DMA_RB_Index + 4];
			}

			MB_CSR_DMA_RB_Index = temp_index;		//Move buffer index to point to last half-word of packet

		}
		
		else	//not a valid packet - look for next potential command byte
		{
			//Error - SSP checksum received by CAN_SSP_Router did not match
//      P3_OUTP_SET = 1<<26;  // **** TEST CODE **** turn on red LED
		  increment_index2 = 0;
			no_data_found = 1;
		}

		//Move to next read buffer location, wrap around as needed
		MB_CSR_DMA_RB_Index = (MB_CSR_DMA_RB_Index + 1) & (MB_CSR_DMA_RB_SIZE - 1); 
		
		//End of DMA packet?
		if (!(MB_CSR_DMA_RB_Index & (MB_CSR_DMA_PACK_SIZE - 1))) 
//		if (!(MB_CSR_DMA_RB_Index << (16 - MB_CSR_DMA_PACK_SIZE_EXP)))
		{
			   increment_index2 = 1; 	//finished a DMA packet, enable move to next
		}

		//Go on to next DMA segment if needed; allow writing of previous segment
		MB_CSR_DMA_Rec_Index2 = (MB_CSR_DMA_Rec_Index2 + increment_index2) & (MB_CSR_DMA_PACK_QUAN - 1);
		return(no_data_found);
  }

  else 
  {
     return(1);	//receive buffer empty
  }
}

*/

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Push a single frame onto the SSP DMA transmit buffer
//Return 0 if successful, 1 if buffer full.
unsigned short int mb_ssp_push_frame(CAN_FRAME * frame)
{
  unsigned short int checksum, temp_index1;

  //set temporary pointer
  temp_index1 = MB_CSR_DMA_TB_Index1;
//  checksum = 0;

  //load data and calculate checksum
  MB_CSR_DMA_TX_Buffer[temp_index1] = frame->addr //CAN ID (address)
	| ((~(frame->dlc) & 0xF) << 11)		   					//data length (bit-wise invert of)
	| ((frame->rtr) << 15)							 	//remote transmission request
	;
	checksum = MB_CSR_DMA_TX_Buffer[temp_index1];
	++temp_index1;
	MB_CSR_DMA_TX_Buffer[temp_index1] = frame->payload.s.s1;
	checksum += MB_CSR_DMA_TX_Buffer[temp_index1];
	++temp_index1;
	MB_CSR_DMA_TX_Buffer[temp_index1] = frame->payload.s.s2;
	checksum += MB_CSR_DMA_TX_Buffer[temp_index1];
	++temp_index1;
	MB_CSR_DMA_TX_Buffer[temp_index1] = frame->payload.s.s3;
	checksum += MB_CSR_DMA_TX_Buffer[temp_index1];
	++temp_index1;
	MB_CSR_DMA_TX_Buffer[temp_index1] = frame->payload.s.s4;
	checksum += MB_CSR_DMA_TX_Buffer[temp_index1];
	++temp_index1;

  
  //load checksum into buffer
  MB_CSR_DMA_TX_Buffer[temp_index1] = ~checksum;

  //increase temp_index1 to next packet start location
  ++temp_index1;

  //if temp_index equals the buffer size, wrap around to zero
  if (temp_index1 == MB_CSR_DMA_TB_SIZE) {temp_index1 = 0;}

  //transmit buffer is full; leave index as is; abandon most recent packet
  if (temp_index1 == MB_CSR_DMA_TB_Index2)
  {
    return(1);
  }

   
  //set Index1 to temp-pointer value
  MB_CSR_DMA_TB_Index1 = temp_index1;
  return(0);
}


////////////////////////////////////////////////////////////////////////////////
//SSP receive data parser. Pops all frames from the SSP receive buffer.
void mb_ssp_pop_frames(void)
{
	unsigned short int cal_checksum, rec_checksum, temp_index, can_id, rtr;
  unsigned long int data, timestamp;
		
	#define CSR_PACKET_LENGTH 6			//Number of 16-bit short ints in ssp packet
//At present this code only implements 8-byte packets, plus a 16-bit checksum

//  T1TCR = (T1TCR | (1<<0)) & 3;		//TEST CODE - start timer 1

  //Only look for a packet to read if the buffer is not empty
  if (MB_CSR_DMA_Rec_Index2 == MB_CSR_DMA_Rec_Index1)
  {
 //   T1TCR = (T1TCR & (~(1<<0))) & 3;	//TEST CODE - stop timer 1
    return;   //no data in buffer
  }
			
  //skip leading zero short ints; address/start short int must be non-zero
  while (!MB_CSR_DMA_Rec_Buffer[MB_CSR_DMA_Rec_Index2])
	{    
    //Move to next short int to check for address status
    //This construction is needed to keep the index from ever going outside the bounds of the ring buffer,
    //even just briefly. A race condition would result with the isr if it did, for example by using
    //just ++index and checking for wraparound afterward.
    if (MB_CSR_DMA_Rec_Index2 == MB_CSR_DMA_RB_SIZE-1){MB_CSR_DMA_Rec_Index2 = 0;} 
    else {++MB_CSR_DMA_Rec_Index2;}
 
    P3_OUTP_SET = 1<<26;  // **** TEST CODE **** turn on red LED
    if (MB_CSR_DMA_Rec_Index2 == MB_CSR_DMA_Rec_Index1) 
    {
      return;
    }    //end of buffer, no start-of-packet found
	}

  //Address short int found

  //Parse all data in the SSP receive buffer. To set a max packet number, use a for loop
  while (1)
  {
	

    //Calculate checksum and save potential incoming data
    temp_index = MB_CSR_DMA_Rec_Index2;
    
		cal_checksum = MB_CSR_DMA_Rec_Buffer[temp_index] + 1; //Start checksum - add one (checksum of zero should not be zero)
    can_id = MB_CSR_DMA_Rec_Buffer[temp_index] & 0x7FF;	//Bottom 11 bits are CAN ID (address);
		rtr = MB_CSR_DMA_Rec_Buffer[temp_index] >> 15; //Top bit is remote transmission request;
		
    if (++temp_index == MB_CSR_DMA_RB_SIZE) {temp_index = 0;}
    if (temp_index == MB_CSR_DMA_Rec_Index1) {return;}
    cal_checksum += MB_CSR_DMA_Rec_Buffer[temp_index];
    data = MB_CSR_DMA_Rec_Buffer[temp_index];

    if (++temp_index == MB_CSR_DMA_RB_SIZE) {temp_index = 0;}
    if (temp_index == MB_CSR_DMA_Rec_Index1) {return;}
    cal_checksum += MB_CSR_DMA_Rec_Buffer[temp_index];
    data |= (MB_CSR_DMA_Rec_Buffer[temp_index]<<16);
    
    if (++temp_index == MB_CSR_DMA_RB_SIZE) {temp_index = 0;}
    if (temp_index == MB_CSR_DMA_Rec_Index1) {return;}    
    cal_checksum += MB_CSR_DMA_Rec_Buffer[temp_index];
    timestamp = MB_CSR_DMA_Rec_Buffer[temp_index];
    
    if (++temp_index == MB_CSR_DMA_RB_SIZE) {temp_index = 0;}
    if (temp_index == MB_CSR_DMA_Rec_Index1) {return;}    
    cal_checksum += MB_CSR_DMA_Rec_Buffer[temp_index];
    timestamp |= (MB_CSR_DMA_Rec_Buffer[temp_index]<<16);
    
    if (++temp_index == MB_CSR_DMA_RB_SIZE) {temp_index = 0;}
    if (temp_index == MB_CSR_DMA_Rec_Index1) {return;}    
 
		//read in received checksum from end of potential packet
		rec_checksum = MB_CSR_DMA_Rec_Buffer[temp_index];
    

 	
		//Valid packet found? Parse and put received values into appropriate location
		if (cal_checksum == rec_checksum)
		{	
      //write data to array here
      P3_OUTP_SET = 1<<25;  // **** TEST CODE **** turn on green LED
      
      ++Total_Packet_Counter;
      
      * mb_variable_pointers[can_id] = data;            
      mb_timestamps[can_id] = timestamp;
      
      //Move index to start of next (potential) packet
      if (++temp_index == MB_CSR_DMA_RB_SIZE) {temp_index = 0;}
      MB_CSR_DMA_Rec_Index2 = temp_index;         //Update global index (frees space for new packets)
      if (MB_CSR_DMA_Rec_Index2 == MB_CSR_DMA_Rec_Index1) {return;}
		}
		
		else	//not a valid packet - look for next potential command byte
		{
			//Error - SSP checksum received did not match
      
      P3_OUTP_SET = 1<<26;  // **** TEST CODE **** turn on red LED
      
      //Move to next potential address half-word
			//MB_CSR_DMA_Rec_Index2 = (MB_CSR_DMA_Rec_Index2 + 1) & (MB_CSR_DMA_RB_SIZE-1);
      //This construction is needed to keep the index from ever going outside the bounds of the ring buffer,
      //even just briefly. A race condition would result with the isr if it did, for example by using
      //just ++index and checking for wraparound afterward.
      if (MB_CSR_DMA_Rec_Index2 == MB_CSR_DMA_RB_SIZE-1){MB_CSR_DMA_Rec_Index2 = 0;} 
      else {++MB_CSR_DMA_Rec_Index2;}
      if (MB_CSR_DMA_Rec_Index2 == MB_CSR_DMA_Rec_Index1) {return;}
 
 //     T1TCR = (T1TCR & (~(1<<0))) & 3;	//TEST CODE - stop timer 1
		}	
	}
//  T1TCR = (T1TCR & (~(1<<0))) & 3;	//TEST CODE - stop timer 1
}
////////////////////////////////////////////////////////////////////////////////

