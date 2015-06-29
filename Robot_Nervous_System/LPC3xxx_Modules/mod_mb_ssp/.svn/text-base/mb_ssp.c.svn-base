#include <mb_includes.h>

extern volatile long unsigned int mb_ssp_test_rx_counter;    //**** TEST CODE ****
extern volatile long unsigned int mb_ssp_test_tx_counter;    //**** TEST CODE ****
extern CAN_FRAME mb_ssp_test_example_frame; //**** Test Code ****

//Global variables

//Global variables for SSP software receive buffer
#define CSR_RB_SIZE 32   //Buffer sizes must be a power of 2
unsigned short int CSR_RB_Num_Data = 0;
unsigned short int CSR_ReadBuffer[CSR_RB_SIZE];		//FIFO read buffer for data from SSP
unsigned short int CSR_RB_Index1 = 0;		//Points to location of next received word
unsigned short int CSR_RB_Index2 = 0;		//Points to location of oldest unused rec. word
											                  //If Index2 = Index1, buffer contains no unused data.
                                        
//Global variables for SSP software secondary transmit buffer
#define CSR_TB_SIZE 6
unsigned short int CSR_TransBuffer[CSR_TB_SIZE];
unsigned short int CSR_TB_Index = 0;



////////////////////////////////////////////////////////////////////////////////
// SSP1 interrupt service routine for receive and transmit
void mb_ssp1_isr(void)
{
  short unsigned int check_sum;
  CAN_FRAME frame;
  
    #ifdef DEBUG
    if (P3_INP_STATE & 1<<1)
    {
      MIC_ER = 0;
      return;
    }
  #endif
  
  if (SSP1MIS & (1<<2))	//receive data available (RX FIFO not empty (>=4 frames)
  {
    while(SSP1SR & (1<<2))   //while RX FIFO not empty, read half-words - drain hardware buffer
    {
	 
	  CSR_ReadBuffer[CSR_RB_Index1] = SSP1DR;		//Put new data in next buffer location
	  CSR_RB_Index1 = (CSR_RB_Index1 + 1) & (CSR_RB_SIZE-1);		//Increment index; roll over to 0 after reaching (CSR_RB_SIZE-1)	

	    if (CSR_RB_Index1 == CSR_RB_Index2)			//software receive buffer overflow
	    {
	      CSR_RB_Index1 = (CSR_RB_Index1 - 1) & (CSR_RB_SIZE-1);	//next data will overwrite most recent half-word
		    //Error - SSP software read buffer overflow
      }	
	  }   
  }
  
   else if (SSP1MIS & (1<<3))	//transmitter hardware FIFO not full 
  {
	  while (SSP1SR & (1<<1)) 	//TX FIFO not full, load with data
    {
	    //The standard SSP packet size is 6 half-words; unfortunately, the SSP hardware
	    //buffer is only 4 half-words, max. To be sure that packets are not split up
	    //accidentally, with zeroes mixed in, a secondary software buffer holds the packet
	    //being sent. This is loaded only during the interrupt, one whole packet at a time.
	    if (!CSR_TB_Index)	//Secondary transmit software FIFO empty
	    {
      
//		    if (csr_ssp_tx_ring_ptr!=NULL&&!can_ring_pop(csr_ssp_tx_ring_ptr, &frame))  //SSP CAN frame buffer not empty
        if (1) // **** TEST CODE ****
		    {
          ++mb_ssp_test_tx_counter;  // **** TEST CODE ****
        	//load data and calculate checksum
          
    	    CSR_TransBuffer[5] = mb_ssp_test_example_frame.addr  //CAN ID (address)
          | ((~(mb_ssp_test_example_frame.dlc) & 0xF) << 11)		   					//data length (bit-wise invert of)
		      | (mb_ssp_test_example_frame.rtr << 15)		//remote transmission request bit
		      ;
       
		      check_sum = CSR_TransBuffer[5] + 1;	//Start checksum calculation
                                              //add one (checksum of zero should not be zero)
		  
		      CSR_TransBuffer[4] = mb_ssp_test_example_frame.payload.s.s2;	//Send second half-word of CAN payload
		      check_sum += CSR_TransBuffer[4];

		      CSR_TransBuffer[3] = mb_ssp_test_example_frame.payload.s.s1;	//Send first half-word of CAN payload
		      check_sum += CSR_TransBuffer[3];

		      CSR_TransBuffer[2] = mb_ssp_test_example_frame.payload.s.s4;	//Send fourth half-word of CAN payload
		      check_sum += CSR_TransBuffer[2];

		      CSR_TransBuffer[1] = mb_ssp_test_example_frame.payload.s.s3;	//Send third half-word of CAN payload
		      check_sum += CSR_TransBuffer[1];

		      CSR_TransBuffer[0] = check_sum; 	//Send checksum.
          
		      //load data and calculate checksum
  //  	    CSR_TransBuffer[5] = frame.addr  //CAN ID (address)
 //         | ((~(frame.dlc) & 0xF) << 11)		   					//data length (bit-wise invert of)
	//	      | (frame.rtr << 15)		//remote transmission request bit
	//	      ;
  //     
	//	      check_sum = CSR_TransBuffer[5];	//Start checksum calculation
		  
	//	      CSR_TransBuffer[4] = frame.payload.s.s1;	//Send first half-word of CAN payload
	//	      check_sum += CSR_TransBuffer[4];

	//	      CSR_TransBuffer[3] = frame.payload.s.s2;	//Send second half-word of CAN payload
	//	      check_sum += CSR_TransBuffer[3];

		//      CSR_TransBuffer[2] = frame.payload.s.s3;	//Send third half-word of CAN payload
		//      check_sum += CSR_TransBuffer[2];

		 //     CSR_TransBuffer[1] = frame.payload.s.s4;	//Send fourth half-word of CAN payload
		 //     check_sum += CSR_TransBuffer[1];

		 //     CSR_TransBuffer[0] = ~check_sum; 	//Send bit-inverse of checksum.
          
		   
		      CSR_TB_Index = CSR_TB_SIZE;		   
		    }

	    }
	
	    if (CSR_TB_Index)	//Secondary transmit software buffer not empty
	    {
		    --CSR_TB_Index;		 
		    SSP1DR = CSR_TransBuffer[CSR_TB_Index];		//Transmit oldest half-word from software FIFO
	    }
	    else
	    {		   
		   SSP1DR = 0;		//Send dummy frame = 0
	    }
    }
  }

  else if (SSP1MIS & (1<<1))	//receive timeout interrupt)
  {
	  SSP1ICR = (1<<1);	//Clear receive time-out interrupt 

	  while(SSP1SR & (1<<2))   //while RX FIFO not empty, read half-words
	  {
	    CSR_ReadBuffer[CSR_RB_Index1] = SSP1DR;		//Put new data in next buffer location
	    CSR_RB_Index1 = (CSR_RB_Index1 + 1) & (CSR_RB_SIZE-1);		//Increment index; roll over to 0 after reaching (CSR_RB_SIZE-1)	

	    if (CSR_RB_Index1 == CSR_RB_Index2)			//software receive buffer overflow
	    {
	      CSR_RB_Index1 = (CSR_RB_Index1 - 1) & (CSR_RB_SIZE-1);	//next data will overwrite most recent half-word
		  //Error - read buffer overflow
      }
	  }	
  }
	
 // else if (SSP1MIS & (1<<0))	//RX FIFO overflow
   else	//RX FIFO overflow
  {
    SSP1ICR = (1<<0);	//Clear receive overflow interrupt
   //error
  }
}
////////////////////////////////////////////////////////////////////////////////

/*
////////////////////////////////////////////////////////////////////////////////
// SSP1 interrupt service routine for receive and transmit
void mb_ssp1_isr(void)
{
  short unsigned int check_sum;
  CAN_FRAME frame;
  
  if (SSP1MIS & (1<<2))	//receive data available (RX FIFO not empty (>=4 frames)
  {
    while(SSP1SR & (1<<2))   //while RX FIFO not empty, read half-words - drain hardware buffer
    {
	 
	  CSR_ReadBuffer[csr_ssp_ring.idx1] = SSP1DR;		//Put new data in next buffer location
	  ++csr_ssp_ring.idx1;		//Increment index; roll over to 0 after reaching (CSR_RB_SIZE-1)	

	    if (csr_ssp_ring.idx1 == csr_ssp_ring.idx2)			//software receive buffer overflow
	    {
	      --csr_ssp_ring.idx1;	//next data will overwrite most recent half-word
        P3_OUTP_SET = 1<<26;  // **** TEST CODE **** turn on red LED
		    //Error - SSP software read buffer overflow
      }	
	  }   
  }
  
   else if (SSP1MIS & (1<<3))	//transmitter hardware FIFO not full 
  {
	  while (SSP1SR & (1<<1)) 	//TX FIFO not full, load with data
    {
	    //The standard SSP packet size is 6 half-words; unfortunately, the SSP hardware
	    //buffer is only 4 half-words, max. To be sure that packets are not split up
	    //accidentally, with zeroes mixed in, a secondary software buffer holds the packet
	    //being sent. This is loaded only during the interrupt, one whole packet at a time.
	    if (!CSR_TB_Index)	//Secondary transmit software FIFO empty
	    {
      
//		    if (csr_ssp_tx_ring_ptr!=NULL&&!can_ring_pop(csr_ssp_tx_ring_ptr, &frame))  //SSP CAN frame buffer not empty
        if (1) // **** TEST CODE ****
		    {
          ++mb_ssp_test_tx_counter;  // **** TEST CODE ****
        	//load data and calculate checksum
          
    	    CSR_TransBuffer[5] = mb_ssp_test_example_frame.addr  //CAN ID (address)
          | ((~(mb_ssp_test_example_frame.dlc) & 0xF) << 11)		   					//data length (bit-wise invert of)
		      | (mb_ssp_test_example_frame.rtr << 15)		//remote transmission request bit
		      ;
       
		      check_sum = CSR_TransBuffer[5] + 1;	//Start checksum calculation
                                              //add one (checksum of zero should not be zero)
		  
		      CSR_TransBuffer[4] = mb_ssp_test_example_frame.payload.s.s1;	//Send first half-word of CAN payload
		      check_sum += CSR_TransBuffer[4];

		      CSR_TransBuffer[3] = mb_ssp_test_example_frame.payload.s.s2;	//Send second half-word of CAN payload
		      check_sum += CSR_TransBuffer[3];

		      CSR_TransBuffer[2] = mb_ssp_test_example_frame.payload.s.s3;	//Send third half-word of CAN payload
		      check_sum += CSR_TransBuffer[2];

		      CSR_TransBuffer[1] = mb_ssp_test_example_frame.payload.s.s4;	//Send fourth half-word of CAN payload
		      check_sum += CSR_TransBuffer[1];

		      CSR_TransBuffer[0] = check_sum; 	//Send checksum.
		   
		      CSR_TB_Index = CSR_TB_SIZE;		   
		    }

	    }
	
	    if (CSR_TB_Index)	//Secondary transmit software buffer not empty
	    {
		    --CSR_TB_Index;		 
		    SSP1DR = CSR_TransBuffer[CSR_TB_Index];		//Transmit oldest half-word from software FIFO
	    }
	    else
	    {		   
		   SSP1DR = 0;		//Send dummy frame = 0
	    }
    }
  }

  else if (SSP1MIS & (1<<1))	//receive timeout interrupt)
  {
	  SSP1ICR = (1<<1);	//Clear receive time-out interrupt 

	  while(SSP1SR & (1<<2))   //while RX FIFO not empty, read half-words
	  {
	    CSR_ReadBuffer[csr_ssp_ring.idx1] = SSP1DR;		//Put new data in next buffer location
	    ++csr_ssp_ring.idx1;		//Increment index; roll over to 0 after reaching (CSR_RB_SIZE-1)	

	    if (csr_ssp_ring.idx1 == csr_ssp_ring.idx2)			//software receive buffer overflow
	    {
	      --csr_ssp_ring.idx1;	//next data will overwrite most recent half-word
        P3_OUTP_SET = 1<<26;  // **** TEST CODE **** turn on red LED
		  //Error - read buffer overflow
      }
	  }	
  }
	
 // else if (SSP1MIS & (1<<0))	//RX FIFO overflow
   else	//RX FIFO overflow
  {
    SSP1ICR = (1<<0);	//Clear receive overflow interrupt
   //error
  }
}
////////////////////////////////////////////////////////////////////////////////
*/


/*
////////////////////////////////////////////////////////////////////////////////
// SSP1 interrupt service routine for receive and transmit
void mb_ssp1_isr(void)
{
  short unsigned int check_sum;
  CAN_FRAME frame;
  
  if (SSP1MIS & (1<<2))	//receive data available (RX FIFO not empty (>=4 frames)
  {
    while(SSP1SR & (1<<2))   //while RX FIFO not empty, read half-words - drain hardware buffer
    {
	 
	  CSR_ReadBuffer[CSR_RB_Index1] = SSP1DR;		//Put new data in next buffer location
	  CSR_RB_Index1 = (CSR_RB_Index1 + 1) & (CSR_RB_SIZE-1);		//Increment index; roll over to 0 after reaching (CSR_RB_SIZE-1)	

	    if (CSR_RB_Index1 == CSR_RB_Index2)			//software receive buffer overflow
	    {
	      CSR_RB_Index1 = (CSR_RB_Index1 - 1) & (CSR_RB_SIZE-1);	//next data will overwrite most recent half-word
		    //Error - SSP software read buffer overflow
      }	
	  }   
  }
  
   else if (SSP1MIS & (1<<3))	//transmitter hardware FIFO not full 
  {
	  while (SSP1SR & (1<<1)) 	//TX FIFO not full, load with data
    {
	    //The standard SSP packet size is 6 half-words; unfortunately, the SSP hardware
	    //buffer is only 4 half-words, max. To be sure that packets are not split up
	    //accidentally, with zeroes mixed in, a secondary software buffer holds the packet
	    //being sent. This is loaded only during the interrupt, one whole packet at a time.
	    if (!CSR_TB_Index)	//Secondary transmit software FIFO empty
	    {
      
//		    if (csr_ssp_tx_ring_ptr!=NULL&&!can_ring_pop(csr_ssp_tx_ring_ptr, &frame))  //SSP CAN frame buffer not empty
        if (1) // **** TEST CODE ****
		    {
          ++mb_ssp_test_tx_counter;  // **** TEST CODE ****
        	//load data and calculate checksum
          
    	    CSR_TransBuffer[5] = mb_ssp_test_example_frame.addr  //CAN ID (address)
          | ((~(mb_ssp_test_example_frame.dlc) & 0xF) << 11)		   					//data length (bit-wise invert of)
		      | (mb_ssp_test_example_frame.rtr << 15)		//remote transmission request bit
		      ;
       
		      check_sum = CSR_TransBuffer[5] + 1;	//Start checksum calculation
                                              //add one (checksum of zero should not be zero)
		  
		      CSR_TransBuffer[4] = mb_ssp_test_example_frame.payload.s.s1;	//Send first half-word of CAN payload
		      check_sum += CSR_TransBuffer[4];

		      CSR_TransBuffer[3] = mb_ssp_test_example_frame.payload.s.s2;	//Send second half-word of CAN payload
		      check_sum += CSR_TransBuffer[3];

		      CSR_TransBuffer[2] = mb_ssp_test_example_frame.payload.s.s3;	//Send third half-word of CAN payload
		      check_sum += CSR_TransBuffer[2];

		      CSR_TransBuffer[1] = mb_ssp_test_example_frame.payload.s.s4;	//Send fourth half-word of CAN payload
		      check_sum += CSR_TransBuffer[1];

		      CSR_TransBuffer[0] = check_sum; 	//Send checksum.
          
		      //load data and calculate checksum
  //  	    CSR_TransBuffer[5] = frame.addr  //CAN ID (address)
 //         | ((~(frame.dlc) & 0xF) << 11)		   					//data length (bit-wise invert of)
	//	      | (frame.rtr << 15)		//remote transmission request bit
	//	      ;
  //     
	//	      check_sum = CSR_TransBuffer[5];	//Start checksum calculation
		  
	//	      CSR_TransBuffer[4] = frame.payload.s.s1;	//Send first half-word of CAN payload
	//	      check_sum += CSR_TransBuffer[4];

	//	      CSR_TransBuffer[3] = frame.payload.s.s2;	//Send second half-word of CAN payload
	//	      check_sum += CSR_TransBuffer[3];

		//      CSR_TransBuffer[2] = frame.payload.s.s3;	//Send third half-word of CAN payload
		//      check_sum += CSR_TransBuffer[2];

		 //     CSR_TransBuffer[1] = frame.payload.s.s4;	//Send fourth half-word of CAN payload
		 //     check_sum += CSR_TransBuffer[1];

		 //     CSR_TransBuffer[0] = ~check_sum; 	//Send bit-inverse of checksum.
          
		   
		      CSR_TB_Index = CSR_TB_SIZE;		   
		    }

	    }
	
	    if (CSR_TB_Index)	//Secondary transmit software buffer not empty
	    {
		    --CSR_TB_Index;		 
		    SSP1DR = CSR_TransBuffer[CSR_TB_Index];		//Transmit oldest half-word from software FIFO
	    }
	    else
	    {		   
		   SSP1DR = 0;		//Send dummy frame = 0
	    }
    }
  }

  else if (SSP1MIS & (1<<1))	//receive timeout interrupt)
  {
	  SSP1ICR = (1<<1);	//Clear receive time-out interrupt 

	  while(SSP1SR & (1<<2))   //while RX FIFO not empty, read half-words
	  {
	    CSR_ReadBuffer[CSR_RB_Index1] = SSP1DR;		//Put new data in next buffer location
	    CSR_RB_Index1 = (CSR_RB_Index1 + 1) & (CSR_RB_SIZE-1);		//Increment index; roll over to 0 after reaching (CSR_RB_SIZE-1)	

	    if (CSR_RB_Index1 == CSR_RB_Index2)			//software receive buffer overflow
	    {
	      CSR_RB_Index1 = (CSR_RB_Index1 - 1) & (CSR_RB_SIZE-1);	//next data will overwrite most recent half-word
		  //Error - read buffer overflow
      }
	  }	
  }
	
 // else if (SSP1MIS & (1<<0))	//RX FIFO overflow
   else	//RX FIFO overflow
  {
    SSP1ICR = (1<<0);	//Clear receive overflow interrupt
   //error
  }
}
////////////////////////////////////////////////////////////////////////////////
*/

/*
////////////////////////////////////////////////////////////////////////////////
//SSP receive data parser. Pops one CAN frame from the SSP receive buffer.
//Returns 0 if successful, 1 if error or buffer empty
unsigned short int mb_ssp_pop_frame(CAN_FRAME * frameptr)
{
	short int num_data;
	unsigned short int cal_checksum, rec_checksum, i;
		
	#define CSR_PACKET_LENGTH 6			//Number of 16-bit half-words in ssp packet
//At present this code only implements 8-byte CAN frames, plus a 16-bit checksum
			
  //skip leading zero hwords; address/start hword must be non-zero
  while (!CSR_ReadBuffer[CSR_RB_Index2])
	{
	  CSR_RB_Index2 = (CSR_RB_Index2 + 1) & (CSR_RB_SIZE-1);  //Move to next half-word to check for address status
    P3_OUTP_SET = 1<<26;  // **** TEST CODE **** turn on red LED
    if (CSR_RB_Index2 == CSR_RB_Index1) {return 1;}    //end of buffer, no start-of-packet found
	}

  //Address half-word found - parse packet if there is at least a full packet in buffer

	//Calculate number of hwords in software read buffer
  if (CSR_RB_Index2 > CSR_RB_Index1)
  {
    num_data = (CSR_RB_Index1 + CSR_RB_SIZE) - CSR_RB_Index2;
  }
  else
  {
    num_data = CSR_RB_Index1 - CSR_RB_Index2;
  }
  
  //Full packet in read buffer?
	if (num_data >= CSR_PACKET_LENGTH)
	{
		//Calculate checksum
		cal_checksum = 1; //add one (checksum of zero should not be zero)
		for (i = 0; i < (CSR_PACKET_LENGTH-1); i++)
		{
			cal_checksum += CSR_ReadBuffer[(CSR_RB_Index2 + i) & (CSR_RB_SIZE-1)];		
		}
		
		//read in received checksum from end of potential packet
		rec_checksum = CSR_ReadBuffer[(CSR_RB_Index2 + CSR_PACKET_LENGTH - 1) & (CSR_RB_SIZE-1)];
				
		//Valid packet found? Parse and put received values into appropriate location (test code below)
		if (cal_checksum == rec_checksum)
		{	
    		frameptr->addr = CSR_ReadBuffer[CSR_RB_Index2];	//Bottom 11 bits are CAN ID (address); .addr is 11-bit field
			  frameptr->dlc =  ~(CSR_ReadBuffer[CSR_RB_Index2] >> 11);//Next 4 bits are data length code; .dlc is 4-bit field
			  frameptr->rtr = CSR_ReadBuffer[CSR_RB_Index2] >> 15; //Top bit is remote transmission request; .rtr is 1-bit field
			  frameptr->payload.s.s1 = CSR_ReadBuffer[(CSR_RB_Index2 + 1) & (CSR_RB_SIZE-1)];
			  frameptr->payload.s.s2 = CSR_ReadBuffer[(CSR_RB_Index2 + 2) & (CSR_RB_SIZE-1)];
			  frameptr->payload.s.s3 = CSR_ReadBuffer[(CSR_RB_Index2 + 3) & (CSR_RB_SIZE-1)];
			  frameptr->payload.s.s4 = CSR_ReadBuffer[(CSR_RB_Index2 + 4) & (CSR_RB_SIZE-1)];
        
			  //Move index to start of next (potential) packet
      CSR_RB_Index2 = (CSR_RB_Index2 + CSR_PACKET_LENGTH) & (CSR_RB_SIZE-1);
      return 0;   //Successful frame reception
		}
		
		else	//not a valid packet - look for next potential command byte
		{
			//Error - SSP checksum received did not match
      
      P3_OUTP_SET = 1<<26;  // **** TEST CODE **** turn on red LED
      
      //Move to next potential address half-word
			CSR_RB_Index2 = (CSR_RB_Index2 + 1) & (CSR_RB_SIZE-1);
      return 1; //Packet receive error, checksum mismatch
		}	
	}
  return 1; //Packet receive error, no packets to parse.
}
////////////////////////////////////////////////////////////////////////////////
*/

/*
////////////////////////////////////////////////////////////////////////////////
//SSP receive data parser. Pops one CAN frame from the SSP receive buffer.
//Returns 0 if successful, 1 if error or buffer empty
unsigned short int mb_ssp_pop_frame(CAN_FRAME * frameptr)
{
	short int num_data;
	unsigned short int cal_checksum, rec_checksum;
  SSP_RING temp_ring;
		
	#define CSR_PACKET_LENGTH 6			//Number of 16-bit half-words in ssp packet
//At present this code only implements 8-byte CAN frames, plus a 16-bit checksum

  if (csr_ssp_ring.idx2 == csr_ssp_ring.idx1)
  {
    return 1;   //no data in buffer
  }
			
  //skip leading zero hwords; address/start hword must be non-zero
  while (!CSR_ReadBuffer[csr_ssp_ring.idx2])
	{
	  ++csr_ssp_ring.idx2;  //Move to next half-word to check for address status
    P3_OUTP_SET = 1<<26;  // **** TEST CODE **** turn on red LED
    if (csr_ssp_ring.idx2 == csr_ssp_ring.idx1) {return 1;}    //end of buffer, no start-of-packet found
	}

  //Address half-word found - parse packet if there is at least a full packet in buffer

	//Calculate number of hwords in software read buffer
  if (csr_ssp_ring.idx2 > csr_ssp_ring.idx1)
  {
    num_data = (csr_ssp_ring.idx1 + CSR_RB_SIZE) - csr_ssp_ring.idx2;
  }
  else
  {
    num_data = csr_ssp_ring.idx1 - csr_ssp_ring.idx2;
  }
  
  //Full packet in read buffer?
	if (num_data >= CSR_PACKET_LENGTH)
	{
		//Calculate checksum, save potential frame
    temp_ring.idx2 = csr_ssp_ring.idx2;
		cal_checksum = CSR_ReadBuffer[temp_ring.idx2] + 1; //start checksum, add one (checksum of zero should not be zero)      
    frameptr->addr = CSR_ReadBuffer[temp_ring.idx2];	//Bottom 11 bits are CAN ID (address); .addr is 11-bit field
		frameptr->dlc =  ~(CSR_ReadBuffer[temp_ring.idx2] >> 11);//Next 4 bits are data length code; .dlc is 4-bit field
		frameptr->rtr = CSR_ReadBuffer[temp_ring.idx2++] >> 15; //Top bit is remote transmission request; .rtr is 1-bit field

		cal_checksum += CSR_ReadBuffer[(temp_ring.idx2)];
    frameptr->payload.s.s1 = CSR_ReadBuffer[temp_ring.idx2++];
    
    cal_checksum += CSR_ReadBuffer[(temp_ring.idx2)];
		frameptr->payload.s.s2 = CSR_ReadBuffer[temp_ring.idx2++];
        
    cal_checksum += CSR_ReadBuffer[(temp_ring.idx2)];
		frameptr->payload.s.s3 = CSR_ReadBuffer[temp_ring.idx2++];
        
    cal_checksum += CSR_ReadBuffer[(temp_ring.idx2)];
		frameptr->payload.s.s4 = CSR_ReadBuffer[temp_ring.idx2++];
      		
		
		//read in received checksum from end of potential packet
		rec_checksum = CSR_ReadBuffer[temp_ring.idx2];
				
		//Valid packet found? Parse and put received values into appropriate location (test code below)
		if (cal_checksum == rec_checksum)
		{	        
			//Move index to start of next (potential) packet
      csr_ssp_ring.idx2 = ++temp_ring.idx2;
      return 0;   //Successful frame reception
		}
		
		else	//not a valid packet - look for next potential command byte
		{
			//Error - SSP checksum received did not match
      
      P3_OUTP_SET = 1<<26;  // **** TEST CODE **** turn on red LED
      
      //Move to next potential address half-word
			++csr_ssp_ring.idx2;
      return 1; //Packet receive error, checksum mismatch
		}	
	}
  return 1; //Packet receive error, not enough data to parse.
}
////////////////////////////////////////////////////////////////////////////////
*/

////////////////////////////////////////////////////////////////////////////////
//SSP receive data parser. Pops one CAN frame from the SSP receive buffer.
//Returns 0 if successful, 1 if error or buffer empty
unsigned short int mb_ssp_pop_frame(CAN_FRAME * frameptr)
{
	short int num_data;
	unsigned short int cal_checksum, rec_checksum, temp_index;
		
	#define CSR_PACKET_LENGTH 6			//Number of 16-bit half-words in ssp packet
//At present this code only implements 8-byte CAN frames, plus a 16-bit checksum

  //Only look for a packet to read if the buffer is not empty
  if (CSR_RB_Index2 == CSR_RB_Index1)
  {
    return 1;   //no data in buffer
  }
			
  //skip leading zero hwords; address/start hword must be non-zero
  while (!CSR_ReadBuffer[CSR_RB_Index2])
	{
	  CSR_RB_Index2 = (CSR_RB_Index2 + 1) & (CSR_RB_SIZE-1);  //Move to next half-word to check for address status
    if (CSR_RB_Index2 == CSR_RB_Index1) {return 1;}    //end of buffer, no start-of-packet found
	}

  //Address half-word found - parse packet if there is at least a full packet in buffer

	//Calculate number of hwords in software read buffer  
  if (CSR_RB_Index2 > CSR_RB_Index1)
  {
    num_data = (CSR_RB_Index1 + CSR_RB_SIZE) - CSR_RB_Index2;
  }
  else
  {
    num_data = CSR_RB_Index1 - CSR_RB_Index2;
  }
  
  //Full packet in read buffer?
	if (num_data >= CSR_PACKET_LENGTH)
	{  
    //Calculate checksum and save potential incoming frame
    temp_index = CSR_RB_Index2;
    
		cal_checksum = CSR_ReadBuffer[temp_index] + 1; //Start checksum - add one (checksum of zero should not be zero)
    frameptr->addr = CSR_ReadBuffer[temp_index];	//Bottom 11 bits are CAN ID (address); .addr is 11-bit field
		frameptr->dlc =  ~(CSR_ReadBuffer[temp_index] >> 11);//Next 4 bits are inverted data length code; .dlc is 4-bit field
		frameptr->rtr = CSR_ReadBuffer[temp_index] >> 15; //Top bit is remote transmission request; .rtr is 1-bit field
		  
    if (++temp_index == CSR_RB_SIZE) {temp_index = 0;}
 
    cal_checksum += CSR_ReadBuffer[temp_index];
    frameptr->payload.s.s2 = CSR_ReadBuffer[temp_index];

    if (++temp_index == CSR_RB_SIZE) {temp_index = 0;}
 
    cal_checksum += CSR_ReadBuffer[temp_index];
    frameptr->payload.s.s1 = CSR_ReadBuffer[temp_index];
    
    if (++temp_index == CSR_RB_SIZE) {temp_index = 0;}
 
    cal_checksum += CSR_ReadBuffer[temp_index];
    frameptr->payload.s.s4 = CSR_ReadBuffer[temp_index];
    
    if (++temp_index == CSR_RB_SIZE) {temp_index = 0;}
 
    cal_checksum += CSR_ReadBuffer[temp_index];
    frameptr->payload.s.s3 = CSR_ReadBuffer[temp_index];
    
    if (++temp_index == CSR_RB_SIZE) {temp_index = 0;} 
 
		//read in received checksum from end of potential packet
		rec_checksum = CSR_ReadBuffer[temp_index];
				
		//Valid packet found? Parse and put received values into appropriate location (test code below)
		if (cal_checksum == rec_checksum)
		{	
      //Move index to start of next (potential) packet
      
      if (++temp_index == CSR_RB_SIZE) {temp_index = 0;}
      CSR_RB_Index2 = temp_index;         //Update global index (frees space for new packets)
      return 0;   //Successful frame reception
		}
		
		else	//not a valid packet - look for next potential command byte
		{
			//Error - SSP checksum received did not match
      
      P3_OUTP_SET = 1<<26;  // **** TEST CODE **** turn on red LED
      
      //Move to next potential address half-word
			CSR_RB_Index2 = (CSR_RB_Index2 + 1) & (CSR_RB_SIZE-1);
      return 1; //Packet receive error, checksum mismatch
		}	
	}
  return 1; //Packet receive error, no packets to parse.
}
////////////////////////////////////////////////////////////////////////////////

