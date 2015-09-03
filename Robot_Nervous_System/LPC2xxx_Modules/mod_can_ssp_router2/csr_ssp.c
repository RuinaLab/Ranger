#include <includes.h>

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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Clear all global buffer variables. While debugging, etc., the indices do not necessarily get reset as expected.
void csr_global_variable_init(void)
{
  short unsigned int i, j;
    
  //Clear SSP receive buffer variables
  csr_rx_index1 = 0;
  csr_rx_index2 = 0;
  
  for (i = 0; i < CSR_RX_SEG_NUM; ++i)
  {
    for (j = 0; j < CSR_SEG_SIZE; ++j)
    {
      csr_rx_buffer[i][j] = 0;
    }
  }
  
  //Clear SSP transmit buffer variables
  csr_tx_index1 = 0;
  csr_tx_index2 = 0;
  
  for (i = 0; i < CSR_TX_BUFFER_SIZE; ++i)
  {
      csr_tx_buffer[i] = 0;
  }  
}

/////////////////////////////////////////////////////////////////////////////////////////
//SSP interrupt service routine for CAN -  SSP router
 //__irq void csr_ssp_isr(void)
void csr_ssp_isr(void)
{	
  static short int rx_index = 0, tx_index = 0, checksum = 0, send_zeroes = 0;  
  short unsigned int dummy; //Dump excess received data here
    
   #ifdef DEBUG
    if (FIO0PIN & 1<<14)
    {
      VICIntEnClr = 0xFFFFFFFF; //Disable all interrupts before debugging
      VICVectAddr = 0;
      return;
    }
  #endif
  
  //Timer1 isr
	if (VICFIQStatus & 1<<5)
  {  
    
    //Update millisecond counter and send CAN sync frames as required
    csr_clock_tick();
        
    //Update read buffer indexes
    if  (rx_index != CSR_SEG_SIZE)
    {
      // Error - either some kind of pointer corruption caused the rx_index to go out of control,
      // or (more likely) the SSP bit rate is too slow to send all the data within the time period
      // set by the timer interrupt. Set the timer interval to a bit more than the expected packet
      // transmission time (e.g., 10% longer)
      error_occurred_fiq(ERROR_SSP_LOW_RATE);
      b10a_mcu_red_led_blink(50);
    }
    rx_index = 0;
    if (++csr_rx_index1 == CSR_RX_SEG_NUM) {csr_rx_index1 = 0;}
    
    //Check for buffer overflow
    if (csr_rx_index1 == csr_rx_index2)
    {
      //overflow - go back to previous segment
      if (--csr_rx_index1 < 0) {csr_rx_index1 = CSR_RX_SEG_NUM - 1;}
      error_occurred_fiq(ERROR_SSP_RX_BUF_OF);
      b10a_mcu_red_led_blink(50);
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
 
  //SSP isr
  if (VICFIQStatus & 1<<11)
  {
  
  while(SSPSR & (1<<2))   // while RX FIFO not empty, read half-words - drain hardware buffer
  {
    if (rx_index < CSR_SEG_SIZE)
    {
      csr_rx_buffer[csr_rx_index1][rx_index] = SSPDR;
      ++rx_index;
    }
    else    // Throw out half-words beyond the end of the packet
    {
      dummy = SSPDR;
    }
	}   
  
	while ((SSPSR & 1<<1) && (SSPIMSC & (1<<3)))
  {   
    //Software transmit buffer is empty? Pad the rest of the transmit segment with zeroes
    if (csr_tx_index2 == csr_tx_index1) {send_zeroes = 1;}
            
    if (tx_index < CSR_SEG_SIZE - 1) //send segment body
    {
      // To do: revise this function and the tx push buffer function to give more reliable
      // synchronization between the 5-halfword data points and the packets (e.g., 31 half-words).
      // As is, the system will resync whenever this function sends zeroes. However, behavior if the
      // buffer is constantly full is worrisome. No problems were seen in testing so far, though.
      // One solution, a little slower, is to structure the tx buffer as a 2D array, with 5-half-word elements
      // in a ring buffer. Each segment would always start with a fresh 5-word element, solving the sync problem
      // This change would also allow the use of high and low-priority buffers, to give faster throughput
      // and more reliable transmisson to critical data. But a little slower overall.
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
    /*
    else if (tx_index <= CSR_SEG_SIZE + 0)  //Send dummy words to clear DMA buffers
    {
      SSPDR = 0;
    }
    else if (tx_index > CSR_SEG_SIZE + 0)  //
    {
      SSPDR = 0;
      SSPIMSC = (SSPIMSC & ~(1<<3)) & (0xF);    //disable SSP transmit interrupts
    }
    */

    else
    {
      error_occurred_fiq(ERROR_SSP_TX_BUF_IDX);
      b10a_mcu_red_led_blink(50);    //Should never get here
 //     SSPIMSC = (SSPIMSC & ~(1<<3)) & (0xF);    //disable SSP transmit interrupts
    }
    ++tx_index; 
  }
 
  if (SSPMIS & (1<<1))	//receive timeout interrupt)
  {
    SSPICR = (1<<1);	//Clear receive time-out interrupt 
  }
  
  if (SSPMIS & (1<<0))	//RX FIFO overflow
  {
    b10a_mcu_red_led_blink(50);
    error_occurred_fiq(ERROR_SSP_RX_FIFO);
    SSPICR = (1<<0);	//Clear receive overflow interrupt 
   //error - SSP hardware receive buffer overflow
  }
  } 
}
////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
//SSP receive data parser
unsigned short int csr_pop_ssp_frame(CAN_FRAME * frameptr)
{
	static unsigned short int j = 0;
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
          cal_checksum = 1;
          for (i = 0; i < CSR_SEG_SIZE - 1; ++i)
          {
            cal_checksum += csr_rx_buffer[csr_rx_index2][i];
          }
          
          if (cal_checksum == csr_rx_buffer[csr_rx_index2][CSR_SEG_SIZE - 1])
          {
            b10a_mcu_green_led_blink(50);
            goto PARSE;    //checksum is good, continue after while statement
//              break;
          }
          else
          {
            error_occurred(ERROR_SSP_BAD_CHKSM);
            b10a_mcu_red_led_blink(50);
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
    
    //To do: change to:  if ((csr_rx_buffer[csr_rx_index2][j] & 0xF800) != 0) to give RTR and DLC support
    if ((csr_rx_buffer[csr_rx_index2][j] & 0xF800) == 0x7800)
    {
      //Should have valid data at this point, if if gets this far
      frameptr->chan = CHAN_SSP;							//identifies source of packet to router
	    frameptr->addr = csr_rx_buffer[csr_rx_index2][j]; //Bottom 11 bits are CAN ID (address); .addr is 11-bit field
	    // To do: enable this line, get rid of the next, to get DLC working here
     // frameptr->dlc = ~(csr_rx_buffer[csr_rx_index2][j] >> 11); //Next 4 bits are data length code; .dlc is 4-bit field
	    frameptr->dlc = 8;  //Only 8-byte payloads supported at this time (and rtr?)
      frameptr->rtr = csr_rx_buffer[csr_rx_index2][j++] >> 15; //Top bit is remote transmission request; .rtr is 1-bit field
	    frameptr->payload.s.s2 = csr_rx_buffer[csr_rx_index2][j++];  //copy payload data
	    frameptr->payload.s.s1 = csr_rx_buffer[csr_rx_index2][j++];
	    frameptr->payload.s.s4 = csr_rx_buffer[csr_rx_index2][j++];
	    frameptr->payload.s.s3 = csr_rx_buffer[csr_rx_index2][j++];
      no_data = 0;
    }
    else if ((csr_rx_buffer[csr_rx_index2][j] & 0xF800) == 0xF800)  // To do: get rid of this, not needed, see above
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
    else if (j > CSR_SEG_SIZE - 1)
    {
      error_occurred(ERROR_SSP_RX_BUF_IDX);
      b10a_mcu_red_led_blink(50);    //Should never get here
    }
    return no_data;   //Valid return data 
  }
  else
  {
    return no_data;   //Receive buffer empty
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned short int csr_push_ssp_frame(CAN_FRAME * frame)
{
  short int temp_index1;
  
  temp_index1 = csr_tx_index1;
  
  //load data from incoming frame
  csr_tx_buffer[temp_index1] = frame->addr  //CAN ID (address)
  | (0xF << 11)		   					//extra bits for future
	| (frame->rtr << 15)		//remote transmission request bit
	;
  //Advance temp_index to next available location
  if (++temp_index1 == CSR_TX_BUFFER_SIZE)  {temp_index1 = 0;}
  //Check for buffer overflow
  if (temp_index1 == csr_tx_index2)
  {
    error_occurred(ERROR_SSP_TX_BUF_OF);
    return 1;
  }  //buffer full

  //Load first short int of CAN payload
	csr_tx_buffer[temp_index1] = frame->payload.s.s2;	
  //Advance temp_index to next available location
  if (++temp_index1 == CSR_TX_BUFFER_SIZE)  {temp_index1 = 0;}
  //Check for buffer overflow
  if (temp_index1 == csr_tx_index2)
  {
    error_occurred(ERROR_SSP_TX_BUF_OF);
    return 1;
  }  //buffer full
  
  //Load second short int of CAN payload
	csr_tx_buffer[temp_index1] = frame->payload.s.s1;	
  //Advance temp_index to next available location
  if (++temp_index1 == CSR_TX_BUFFER_SIZE)  {temp_index1 = 0;}  
  //Check for buffer overflow
  if (temp_index1 == csr_tx_index2) 
  {
    error_occurred(ERROR_SSP_TX_BUF_OF);
    return 1;
  } //buffer full
  
  //Load third short int of CAN payload
  csr_tx_buffer[temp_index1] = frame->payload.s.s4;	  
  //Advance temp_index to next available location
  if (++temp_index1 == CSR_TX_BUFFER_SIZE)  {temp_index1 = 0;}  
  //Check for buffer overflow
  if (temp_index1 == csr_tx_index2) 
  {
    error_occurred(ERROR_SSP_TX_BUF_OF);
    return 1;
  } //buffer full
    
  //Load fourth short int of CAN payload  
  csr_tx_buffer[temp_index1] = frame->payload.s.s3;	  
  //Advance temp_index to next available location
  if (++temp_index1 == CSR_TX_BUFFER_SIZE)  {temp_index1 = 0;}  
  //Check for buffer overflow
  if (temp_index1 == csr_tx_index2) 
  {
    error_occurred(ERROR_SSP_TX_BUF_OF);    
    return 1;
  } //buffer full
  
  csr_tx_index1 = temp_index1;    //update csr_tx_index1 and make new data available to isr
 	     
  return 0; //Frame added successfully          
}
