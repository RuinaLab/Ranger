//ARM7-side code for SSP (SPI) communication between ARM7 satellites and ARM9 LPC3250 main brain.
//This protocol uses a synchronized "segment" approach, timed by Timer0 on the LPC2194/01. At each Timer0
//interrupt a new data segment is started, and it runs to completion. Therefore, the SSP bit rate and the
//setting of Timer1 must be such that all of the data in the segment can be transmitted before the next timing pulse.
//SSP and Timer1 both use FIQ interrupts, which makes this more likely to work.

//Functions include a combined FIQ interrupt service routine; a CAN frame push function, to put a frame from the CAN
//buses on the SSP transmit buffer; and a CAN frame pop function, which takes a frame off the SSP receive buffer.

#include <includes.h>

//Segment and data packet length settings. For example, a segment could contain six packets of 5 short ints each,
//plus one 16-bit checksum. This would allow a segment time of 100 uS at 6 Mbits/sec, or 600 uS at 1 Mbits/sec.

#define SSP_SEG_SIZE 31
#define SSP_PACKET_SIZE 5

//Global variables for SSP software receive buffer
#define SSP_RX_SEG_NUM 8

volatile unsigned short int csr_ssp_rx_buffer[SSP_RX_SEG_NUM][SSP_SEG_SIZE];//FIFO read buffer for data from SSP
volatile short int csr_ssp_rx_index1 = 0;			//Points to location of segment being written by SSP isr
volatile short int csr_ssp_rx_index2 = 0;			//Points to location of segment being read by parser
												                  //If Index2 = Index1, buffer contains no unread data.
                                          //If Index1 precedes Index2 by one in ring, buffer is full.

//Global variables for SSP software transmit buffer
#define SSP_TX_BUFFER_SIZE 500
volatile unsigned short int csr_ssp_tx_buffer[SSP_TX_BUFFER_SIZE];
volatile short int csr_ssp_tx_index1 = 0;
volatile short int csr_ssp_tx_index2 = 0;


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//SSP and Timer0 Match0 combined interrupt service routine
void csr_ssp_timer0_isr(void) __irq
{	
  static short int rx_index = 0, tx_index = 0, checksum = 0, send_zeroes = 0;  
  volatile short unsigned int dummy; //Dump excess received data here
    
  //Timer0 isr
	if (VICFIQStatus & 1<<4)
  {
    //Update read buffer indexes
    rx_index = 0;
    if (++csr_ssp_rx_index1 == SSP_RX_SEG_NUM) {csr_ssp_rx_index1 = 0;}
    
    //Check for buffer overflow
    if (csr_ssp_rx_index1 == csr_ssp_rx_index2)
    {
      //overflow - go back to previous segment
      if (--csr_ssp_rx_index1 < 0) {csr_ssp_rx_index1 = SSP_RX_SEG_NUM - 1;}
      //error - read buffer overflow; will overwrite previously received segment
    } 
    
    //Update transmit variables
    checksum = 1; //Checksum is total plus one, so checksum of zero is non-zero
    tx_index = 0;
    send_zeroes = 0;
    
    //Update interrupts
    SSPIMSC = 0xF;		//enable all SSP interrupts
    T0IR = 0xFF;  // Clear all Timer0 interrupt flags.
  }
  
  while(SSPSR & (1<<2))   //while RX FIFO not empty, read half-words - drain hardware buffer
  {
    if (rx_index < SSP_SEG_SIZE)
    {
      csr_ssp_rx_buffer[csr_ssp_rx_index1][rx_index] = SSPDR;
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
    if (csr_ssp_tx_index2 == csr_ssp_tx_index1) {send_zeroes = 1;}
            
    if (tx_index < SSP_SEG_SIZE - 1) //send segment
    {
      if (!send_zeroes) //send data
      {
        SSPDR = csr_ssp_tx_buffer[csr_ssp_tx_index2];
        checksum += csr_ssp_tx_buffer[csr_ssp_tx_index2];
        if (++csr_ssp_tx_index2 == SSP_TX_BUFFER_SIZE) {csr_ssp_tx_index2 = 0;}
      }
      else          //send zeroes to pad out segment
      {
        SSPDR = 0;
      }
    }
    else if (tx_index == SSP_SEG_SIZE - 1)  //Send checksum
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
    SSPICR = (1<<0);	//Clear receive overflow interrupt 
   //error - SSP hardware receive buffer overflow
  }
  VICVectAddr = 0;  //"reset VIC priority hardware"
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//SSP receive data parser
unsigned short int csr_ssp_pop_frame(CAN_FRAME * frameptr)
{
	static unsigned short int j = 0;
	unsigned short int i, cal_checksum, no_data;

  
  no_data = 1;
  //Begin parsing if buffer has data available
  if (csr_ssp_rx_index2 != csr_ssp_rx_index1)
  {
    //New segment? Calculate checksum.
    if (j == 0)
    {
      while (1)
      {
        //Calculate checksum if first segment element is non-zero
        if (csr_ssp_rx_buffer[csr_ssp_rx_index2][0] != 0)
        {
          cal_checksum = 1;
          for (i = 0; i < SSP_SEG_SIZE - 1; ++i)
          {
            cal_checksum += csr_ssp_rx_buffer[csr_ssp_rx_index2][i];
          }
          
          if (cal_checksum == csr_ssp_rx_buffer[csr_ssp_rx_index2][SSP_SEG_SIZE - 1])
          {
                           
           //TODO: Fix this to not use goto.  goto is error prone, not optimizer friendly, and rarely is there not a better, cleaner way.
 //           FIO1CLR = 1<<23;  // **** TEST CODE **** turn on green LED
            goto PARSE;    //checksum is good, continue after while statement
          }
          else
          {
           //error - bad checksum
//            FIO1CLR = 1<<24;  // **** TEST CODE **** turn on red LED
          }
        }
            
        //Advance to next segment and try again
        if (csr_ssp_rx_index2 == SSP_RX_SEG_NUM - 1)
        {
          csr_ssp_rx_index2 = 0;
        }
        else
        {
          ++csr_ssp_rx_index2;
        }
        
        //Check that buffer is not empty
        if (csr_ssp_rx_index2 == csr_ssp_rx_index1)
        {
          return no_data; //Buffer empty
        }
      }
    }
  
    PARSE:
    
    //Look for valid data address
    if ((csr_ssp_rx_buffer[csr_ssp_rx_index2][j] & 0xF800) == 0x7800)
    {
      //Should have valid data at this point, if if gets this far
      frameptr->chan = CHAN_SSP;							//identifies source of packet to router
	    frameptr->addr = csr_ssp_rx_buffer[csr_ssp_rx_index2][j]; //Bottom 11 bits are CAN ID (address); .addr is 11-bit field
	   // frameptr->dlc = ~(csr_rx_buffer[csr_rx_index2][j] >> 11); //Next 4 bits are data length code; .dlc is 4-bit field
	    frameptr->dlc = 8;  //Only 8-byte payloads supported at this time (and rtr?)
      frameptr->rtr = csr_ssp_rx_buffer[csr_ssp_rx_index2][j++] >> 15; //Top bit is remote transmission request; .rtr is 1-bit field
	    frameptr->payload.s.s2 = csr_ssp_rx_buffer[csr_ssp_rx_index2][j++];  //copy payload data
	    frameptr->payload.s.s1 = csr_ssp_rx_buffer[csr_ssp_rx_index2][j++];
	    frameptr->payload.s.s4 = csr_ssp_rx_buffer[csr_ssp_rx_index2][j++];
	    frameptr->payload.s.s3 = csr_ssp_rx_buffer[csr_ssp_rx_index2][j++];
      no_data = 0;
    }
    else if ((csr_ssp_rx_buffer[csr_ssp_rx_index2][j] & 0xF800) == 0xF800)
    {
      //remote transfer request (RTR)
      j += SSP_PACKET_SIZE;
    }
    else  //not a valid data packet, skip the rest of the segment
    {
      j = SSP_SEG_SIZE - 1;
    }
  
    if (j == SSP_SEG_SIZE - 1)
    {
      //Advance to next segment
      if (csr_ssp_rx_index2 == SSP_RX_SEG_NUM - 1)
      {
        csr_ssp_rx_index2 = 0;
      }
      else
      {
        ++csr_ssp_rx_index2;
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
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned short int csr_ssp_push_frame(CAN_FRAME * frame)
{
  short int temp_index1 = csr_ssp_tx_index1;
  
  //load data from incoming frame
  csr_ssp_tx_buffer[temp_index1] = frame->addr  //CAN ID (address)
  | (0xF << 11)		   					//extra bits for future
	| (frame->rtr << 15)		//remote transmission request bit
	;
  
  //Advance temp_index to next available location
  if (++temp_index1 == SSP_TX_BUFFER_SIZE)  {temp_index1 = 0;}
  //Check for buffer overflow
  if (temp_index1 == csr_ssp_tx_index2) {return 1;} //buffer full

	csr_ssp_tx_buffer[temp_index1] = frame->payload.s.s2;	//Load first short int of CAN payload
  
  //Advance temp_index to next available location
  if (++temp_index1 == SSP_TX_BUFFER_SIZE)  {temp_index1 = 0;}
  //Check for buffer overflow
  if (temp_index1 == csr_ssp_tx_index2) {return 1;} //buffer full

	csr_ssp_tx_buffer[temp_index1] = frame->payload.s.s1;	//Load second short int of CAN payload
  
  //Advance temp_index to next available location
  if (++temp_index1 == SSP_TX_BUFFER_SIZE)  {temp_index1 = 0;}
  //Check for buffer overflow
  if (temp_index1 == csr_ssp_tx_index2) {return 1;} //buffer full

  csr_ssp_tx_buffer[temp_index1] = frame->payload.s.s4;	//Load third short int of CAN payload
  
  //Advance temp_index to next available location
  if (++temp_index1 == SSP_TX_BUFFER_SIZE)  {temp_index1 = 0;}
  //Check for buffer overflow
  if (temp_index1 == csr_ssp_tx_index2) {return 1;} //buffer full
  
  csr_ssp_tx_buffer[temp_index1] = frame->payload.s.s3;	//Load fourth short int of CAN payload
  
  //Advance temp_index to next available location
  if (++temp_index1 == SSP_TX_BUFFER_SIZE)  {temp_index1 = 0;}
  //Check for buffer overflow
  if (temp_index1 == csr_ssp_tx_index2) {return 1;} //buffer full
  
  csr_ssp_tx_index1 = temp_index1;    //update ssp_tx_index1 and make new data available to isr
  
		     
  return 0; //Frame added successfully          
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
