//Code for SSP - CAN router. CAN frames are received from up to 4 CAN buses and the SSP bus,
//and directed out again as per the bits set in the routing table. Each possible standard CAN address
//from 0 to 2031 (CAN does not permit addresses 2032 through 2047) gets its own routing entry (one byte)
//in the CSR_Routing_Table lookup table. Bit 0 (LSB) indicates that SSP (the main brain) is to receive a
//frame; bit 1 through 4 are for CAN buses 1 through 4, respectively.

#include <includes.h>

//Global variables

//volatile long unsigned int csr_test_variable = 0; //**** TEST CODE ****
volatile long unsigned int sent_packets = 0;  //**** TEST CODE ****

extern CAN_FRAME Example_Frame; //**** Test Code ****

//Ring buffer pointers. Ring objects are declared in software_setup.c
CAN_RING * csr_ssp_tx_ring_ptr;

//CAN packet routing table. This can be filled in manually, or (future) by having each
//processor "subscribe" to data types it wants to receive by sending out a request to
//receive (RTR) packet for each data type. Routing table size would be 11 bits (2048),
//except that CAN does not allow the first 7 bits to be high (recessive), thus limiting
//the top CAN address in standard mode to 2031. (CAN20B.pdf page 13)
//unsigned char CSR_Routing_Table[2032];

//Global variables for SSP software receive buffer
#define CSR_RB_SIZE 32

unsigned short int CSR_ReadBuffer[CSR_RB_SIZE];//FIFO read buffer for data from SSP
unsigned short int CSR_RB_Index1 = 0;			//Points to location of next received word
unsigned short int CSR_RB_Index2 = 0;			//Points to location of oldest unused rec. word
												                  //If Index2 = Index1, buffer contains no unread data.

//Global variables for SSP software secondary transmit buffer
#define CSR_TB_SIZE 6
unsigned short int CSR_TransBuffer[CSR_TB_SIZE];
unsigned short int CSR_TB_Index = 0;


//Initialize SSP transmit ring buffer
void csr_ssp_tx_ring_ptr_set(CAN_RING * ring)
{
  csr_ssp_tx_ring_ptr = ring;
}


/*
//Take incoming data from SSP receive buffer and CAN receive buffer.
//Call csr_route_frame to distribute it to the correct locations
void csr_route(void)
{
  CAN_FRAME frame;

  //pop one CAN frame pointer from ssp; route if available
  if (!csr_pop_ssp_frame(&frame))
  {
    csr_route_frame(&frame);
 //   ++csr_test_variable;      //Test code
  }

  //pop one CAN frame from CAN receive buffer; route if available
//  if (!can_ring_pop(&rx_ring, &frame))
//  {
//    csr_route_frame(&frame);
//  }
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
 //   route = CSR_Routing_Table[frame->addr];	//Get routing information from lookup table

   route = 1<<SSP;	// **** TEST CODE ****
  
  //Push frame onto destination bus ring buffers,
  //according to entries in routing table  
	
//	for(i = 0;i < 5;i++){
//		if(route & 1<<i) can_ring_push(tx_rings[i],frameptr);
//	}
    if (route & 1<<SSP)
    {
   	  buffer_full = can_ring_push(csr_ssp_tx_ring_ptr, frameptr);
    }

    if (route & 1<<CAN_CHAN_1)
    {
 //  	  buffer_full = can_ring_push(&tx1_ring, frameptr);
    }

    if (route & 1<<CAN_CHAN_2)
    {
//      buffer_full = can_ring_push(&tx2_ring, frameptr);
    }

    if (route & 1<<CAN_CHAN_3)
    {
//   	  buffer_full = can_ring_push(&tx3_ring, frameptr);
    }

    if (route & 1<<CAN_CHAN_4)
    {
 //  	  buffer_full = can_ring_push(&tx4_ring, frameptr);
    }
  }
  return(buffer_full);
}

*/

//test code - prime SSP interrupts
void csr_ssp_prime(void)
{
  SSPDR = 0;
}



//SSP interrupt service routine for CAN -  SSP router
 //__irq void csr_ssp_isr(void)
void csr_ssp_isr(void)
{	
  unsigned short int check_sum;
  CAN_FRAME frame;
  
	#ifdef DEBUG
    if (FIO0PIN & 1<<14)
    {
      VICIntEnClr = 0xFFFFFFFF; //Disable all interrupts before debugging
      VICVectAddr = 0;
      return;
    }
  #endif

  
  if (SSPMIS & (1<<2))	//receive data available (RX FIFO not empty (>=4 frames)
  {
    while(SSPSR & (1<<2))   //while RX FIFO not empty, read half-words - drain hardware buffer
    {
	 
	  CSR_ReadBuffer[CSR_RB_Index1] = SSPDR;		//Put new data in next buffer location
	  CSR_RB_Index1 = ((CSR_RB_Index1 + 1) & (CSR_RB_SIZE - 1));		//Increment index; roll over to 0 after reaching (CSR_RB_SIZE-1)	

	  if (CSR_RB_Index1 == CSR_RB_Index2)			//software receive buffer overflow
	  {
	    CSR_RB_Index1 = ((CSR_RB_Index1 - 1) & (CSR_RB_SIZE - 1));	//next data will overwrite most recent half-word
		//Error - SSP software read buffer overflow
      }	
	}   
  }
  		 	 
  else if (SSPMIS & (1<<3))	//transmitter hardware FIFO not full 
  {
	  while (SSPSR & (1<<1)) 	//TX FIFO not full, load with data
    {
	    //The standard SSP packet size is 6 half-words; unfortunately, the SSP hardware
	    //buffer is only 4 half-words, max. To be sure that packets are not split up
	    //accidentally, with zeroes mixed in, a secondary software buffer holds the packet
	    //being sent. This is loaded only during the interrupt, one whole packet at a time.
	    if (!CSR_TB_Index)	//Secondary transmit software FIFO empty
	    {
//		    if (csr_ssp_tx_ring_ptr!=NULL&&!can_ring_pop(csr_ssp_tx_ring_ptr, &frame))  //SSP CAN frame buffer not empty
        if (1) //**** TEST CODE ****
		    {
          ++sent_packets;  //**** TEST CODE ****
        	//load data and calculate checksum
          
    	    CSR_TransBuffer[5] = Example_Frame.addr  //CAN ID (address)
          | ((~(Example_Frame.dlc) & 0xF) << 11)		   					//data length (bit-wise invert of)
		      | (Example_Frame.rtr << 15)		//remote transmission request bit
		      ;
       
		      check_sum = CSR_TransBuffer[5] + 1;	//Start checksum calculation
                                              //add one (checksum of zero should not be zero)
		  
		      CSR_TransBuffer[4] = Example_Frame.payload.s.s2;	//Send second half-word of CAN payload
		      check_sum += CSR_TransBuffer[4];

		      CSR_TransBuffer[3] = Example_Frame.payload.s.s1;	//Send first half-word of CAN payload
		      check_sum += CSR_TransBuffer[3];

		      CSR_TransBuffer[2] = Example_Frame.payload.s.s4;	//Send fourth half-word of CAN payload
		      check_sum += CSR_TransBuffer[2];

		      CSR_TransBuffer[1] = Example_Frame.payload.s.s3;	//Send third half-word of CAN payload
		      check_sum += CSR_TransBuffer[1];

		      CSR_TransBuffer[0] = check_sum; 	//Send checksum.
        /*
		      //load data and calculate checksum
    	    CSR_TransBuffer[5] = frame.addr  //CAN ID (address)
          | ((~(frame.dlc) & 0xF) << 11)		   					//data length (bit-wise inverse of)
		      | (frame.rtr << 15)		//remote transmission request bit
		      ;
       
		      check_sum = CSR_TransBuffer[5] + 1;	//Start checksum calculation
                                               //add one (checksum of zero should not be zero)
		  
		      CSR_TransBuffer[4] = frame.payload.s.s2;	//Send second half-word of CAN payload
		      check_sum += CSR_TransBuffer[4];

		      CSR_TransBuffer[3] = frame.payload.s.s1;	//Send first half-word of CAN payload
		      check_sum += CSR_TransBuffer[3];

		      CSR_TransBuffer[2] = frame.payload.s.s4;	//Send fourth half-word of CAN payload
		      check_sum += CSR_TransBuffer[2];

		      CSR_TransBuffer[1] = frame.payload.s.s3;	//Send third half-word of CAN payload
		      check_sum += CSR_TransBuffer[1];

		      CSR_TransBuffer[0] = check_sum; 	//Send checksum.
          */
		   
		      CSR_TB_Index = CSR_TB_SIZE;		   
		    }
	    }
	  
	    if (CSR_TB_Index)	//Secondary transmit software buffer not empty
	    {
		    --CSR_TB_Index;		 
		    SSPDR = CSR_TransBuffer[CSR_TB_Index];		//Transmit oldest half-word from software FIFO
	    }
	    else
	    {		   
		   SSPDR = 0;		//Send dummy frame = 0
	    }
    }
  }

  
  else if (SSPMIS & (1<<1))	//receive timeout interrupt)
  {
	  SSPICR = (1<<1);	//Clear receive time-out interrupt 

	  while(SSPSR & (1<<2))   //while RX FIFO not empty, read half-words
	  {
	    CSR_ReadBuffer[CSR_RB_Index1] = SSPDR;		//Put new data in next buffer location
	    CSR_RB_Index1 = ((CSR_RB_Index1 + 1) & (CSR_RB_SIZE - 1));		//Increment index; roll over to 0 after reaching (CSR_RB_SIZE-1)	

	    if (CSR_RB_Index1 == CSR_RB_Index2)			//software receive buffer overflow
	    {
	      CSR_RB_Index1 = ((CSR_RB_Index1 - 1) & (CSR_RB_SIZE-1));	//next data will overwrite most recent half-word
		  //Error - read buffer overflow
        }
	  }	
  }
  
  
  else //(SSPMIS & (1<<0))	//RX FIFO overflow
  {
    SSPICR = (1<<0);	//Clear receive overflow interrupt 
   //error
  }
 
  VICVectAddr = 0;		//Reset interrupt priorities
}

//SSP receive data parser.
unsigned short int csr_pop_ssp_frame(CAN_FRAME * frameptr)
{
	unsigned short int tempindex1 = 0, tempindex2 = 0, i;
	unsigned short int cal_checksum = 0, rec_checksum = 0;
	unsigned short int buffer_empty = 1;
  
	#define CSR_PACKET_LENGTH 6			//Number of 16-bit half-words in packet
//	#define CSR_ADDRESS 0x4000			//Address half-words should have the following leading
										//(MSB) bits: 11000, for an 8-byte CAN packet.
										//The MSB is defined to always be 1 for an address frame;
										//the next four bits give the data size, up to 8 bytes.
										//At present this code only implements 8-byte packets.
			
	tempindex2 = CSR_RB_Index2;
	if (tempindex2 > CSR_RB_Index1)
	{
		tempindex1 = CSR_RB_Index1 + CSR_RB_SIZE;
	}
	else
	{
		tempindex1 = CSR_RB_Index1;
	}

//	while (((CSR_ReadBuffer[tempindex2 & (CSR_RB_SIZE - 1)] & 0xF800) != 0xFFF) && ((tempindex1 - tempindex2) >= CSR_PACKET_LENGTH))
	while (!(CSR_ReadBuffer[tempindex2 & (CSR_RB_SIZE - 1)]) && ((tempindex1 - tempindex2) >= CSR_PACKET_LENGTH))
	{
		tempindex2++;	//Find first non-zero half-word in buffer,
									//indicating a potential address frame,
									//while still leaving enough data to parse a full packet.
									//Leading half-words that don't match are discarded
	}
	
	CSR_RB_Index2 = (tempindex2 & (CSR_RB_SIZE - 1));//Adjust Index2 to point to first potential
													 //command half-word or last half-word checked,
													 //thus skipping over any leading
													 //unrecognized half-words.
		
	//Address half-word found? Parse packet
//	if (((CSR_ReadBuffer[CSR_RB_Index2] & 0xF800) == CSR_ADDRESS) && ((tempindex1 - tempindex2) > CSR_PACKET_LENGTH))
	if (CSR_ReadBuffer[CSR_RB_Index2] && ((tempindex1 - tempindex2) >= CSR_PACKET_LENGTH))
	{

		//Calculate checksum
		cal_checksum = 1;   //add one (checksum of zero should not be zero)
		
		for (i = 0; i < (CSR_PACKET_LENGTH-1); i++)
		{
			cal_checksum += CSR_ReadBuffer[(CSR_RB_Index2 + i) & (CSR_RB_SIZE - 1)];		
		}
		
		//read in received checksum from end of potential packet
		rec_checksum = CSR_ReadBuffer[(CSR_RB_Index2 + CSR_PACKET_LENGTH - 1) & (CSR_RB_SIZE - 1)];
				
		//Valid packet found? Parse and put received values into frame
		if (cal_checksum == rec_checksum)
		{	
			frameptr->chan = CHAN_SSP;							//identifies source of packet to router
			frameptr->addr = CSR_ReadBuffer[CSR_RB_Index2]; //Bottom 11 bits are CAN ID (address); .addr is 11-bit field
			frameptr->dlc = ~(CSR_ReadBuffer[CSR_RB_Index2] >> 11); //Next 4 bits are data length code; .dlc is 4-bit field
			frameptr->rtr = CSR_ReadBuffer[CSR_RB_Index2] >> 15; //Top bit is remote transmission request; .rtr is 1-bit field
			frameptr->payload.s.s2 = CSR_ReadBuffer[(CSR_RB_Index2 + 1) & (CSR_RB_SIZE - 1)];  //copy payload data
			frameptr->payload.s.s1 = CSR_ReadBuffer[(CSR_RB_Index2 + 2) & (CSR_RB_SIZE - 1)];
			frameptr->payload.s.s4 = CSR_ReadBuffer[(CSR_RB_Index2 + 3) & (CSR_RB_SIZE - 1)];
			frameptr->payload.s.s3 = CSR_ReadBuffer[(CSR_RB_Index2 + 4) & (CSR_RB_SIZE - 1)];

			FIO1CLR = 1<<23;  //Test code - turn on green LED
			buffer_empty = 0;
			
			//Advance pointer to next unread buffer location
			CSR_RB_Index2 = ((CSR_RB_Index2 + CSR_PACKET_LENGTH) & (CSR_RB_SIZE - 1));
		}
		
		else	//not a valid packet - look for next potential command byte
		{
			//Error - SSP checksum received by CAN_SSP_Router did not match
      FIO1CLR = 1<<24; 	 	// turn on red mcu led **** TEST CODE ****
			CSR_RB_Index2 = ((CSR_RB_Index2 +1) & (CSR_RB_SIZE - 1));	//Move one step past old potential address half-word in buffer
		}
		
	}
	return(buffer_empty);
}

