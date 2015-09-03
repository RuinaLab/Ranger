/**
	@file oemstar_gps.c
	
	Control code for the NovaTel OEMStar GPS board
  
  Example hardware setup for communicating with OEMStar over UART0:
  @code
  // *******************************************************************************
	// Initialize B2A2 board for communication over UART0
	// *******************************************************************************
  U0LCR=0x83;  //Step 1: DLAB=1, enable baud rate adjustments
  //Set to 8 data bits, 1 stop bit, no parity
	//Step 2: Set baud rate divisor and fractional baud rate divisors
  U0DLL=25;		//Baud rate divisor for 115200 kbaud, with (1+0.3) fractional divider 
  U0DLM=0;		//and peripheral clock = 60MHz.
  U0FDR = 3;		// DIVADDVAL = 3
  U0FDR |= 10<<4;	// MULTVAL = 10
  //U0FCR = 0x81;    	// Enable receive and transmit FIFO; interrupt on receive at 8 bytes.
  U0FCR = 0x1;		// Enable FIFO
  U0FCR = 0x87;		//Clear FIFOs and set to generate interrupt when RX FIFO reaches 8 bytes.
  U0MCR = 0;	   	// Loopback mode, CTS and RTS flow control disabled.
  U0ACR = 0;	   	// Autobaud disabled
  U0TER = 0x80;	   	// Transmit enabled
  U0LCR &= ~(1<<7); // Step 3: DLAB=0, disable baud rate adjustments
  PINSEL0 |= 1<<16;    // UART1 TXD active on MCU pin P0[8]
  PINSEL0 &= ~(1<<17); //
  PINSEL0 |= 1<<18;    // UART1 RXD	active on MCU pin P0[9]
  PINSEL0 &= ~(1<<19); //
  U0IER = (1<<0); // enable RBR interrupts
  U0ACR = (1<<8) | (1<<9); //clear the corresponding interrupt in the U1IIR
  @endcode
  
  @author Nicolas Williamson
  @author Jason Cortell
  @date March 2011
	
*/

#include <includes.h>

//Global variables for UART0 software receive buffer
#define GPS_RX_BUFFER_SIZE 128    //Buffer size must be a power of two!
volatile unsigned char gps_read_buffer[GPS_RX_BUFFER_SIZE];	//FIFO read buffer for data from UART0
volatile unsigned char gps_rb_index1 = 1;			//Points to location of next open spot in buffer
volatile unsigned char gps_rb_index2 = 0;			//Points to location of oldest unused rec. byte
//If Index1 = (Index2 + 1) & (GPS_RX_BUFFER_SIZE - 1), buffer is empty.
//Buffer is full when Index1 = Index2

//Global variables for UART0 software transmit buffer
#define GPS_TX_BUFFER_SIZE 128    //Buffer size must be a power of two!
volatile unsigned char gps_trans_buffer[GPS_TX_BUFFER_SIZE];		//FIFO transmit buffer for data to UART
volatile unsigned char gps_tb_index1 = 0;			//Points to location of next open spot in buffer
volatile unsigned char gps_tb_index2 = 0;			//Points to location of oldest byte not yet transmitted
//If Index1 = Index2 - 1 (with appropriate wraparound), buffer is full.
//Buffer is empty when Index1 = Index2

/**
  The interrupt service routine for UART0.
  Stores incoming data to rx software buffer, sends out data
  from tx software buffer.
*/
void gps_isr(void) __irq
{	
  unsigned long interrupt_id, line_status;
	unsigned short i;

	interrupt_id = U0IIR;
	
	switch (interrupt_id & 0xE) //Look at bits 1 - 3 of U0IIR
	{
	case 4:	//receive data available (>=8 bytes in hardware receive FIFO, as set in U0FCR)
	  for (i=0;i<8;i++)
	  {
	    line_status = U0LSR;
	    if (gps_rb_index1 != gps_rb_index2)			  //software receive buffer not full
	    {
	      gps_read_buffer[gps_rb_index1] = U0RBR;	//Put new data in next buffer location
        if (gps_rb_index1 == GPS_RX_BUFFER_SIZE - 1)
        {
          gps_rb_index1 = 0;  //Roll over gps_rb_index1
        }
        else
        {
          gps_rb_index1++;    //Increment gps_rb_index1
        }
      }
      else //Software receive buffer overflow
      {
         error_occurred_isr(ERROR_GPS_RX_OF);   //Error message
      }	
	  }
	  break;

	  case 12:	//character time-out - at least one left-over byte in hardware receive FIFO
	    line_status = U0LSR;
	    if (gps_rb_index1 != gps_rb_index2)			  //software receive buffer not full
	    {
	      gps_read_buffer[gps_rb_index1] = U0RBR;	//Put new data in next buffer location
        if (gps_rb_index1 == GPS_RX_BUFFER_SIZE - 1)
        {
          gps_rb_index1 = 0;  //Roll over gps_rb_index1
        }
        else
        {
          gps_rb_index1++;    //Increment gps_rb_index1
        }
      }
      else //Software receive buffer overflow
      {
        error_occurred_isr(ERROR_GPS_RX_OF);   //Error message
      }	
	 
    break;
	 
	  case 2:	//transmitter hardware FIFO empty
	    for (i=0;i<16;i++) 	//Move up to 16 bytes (FIFO capacity) to hardware FIFO
	    {
	      if (gps_tb_index1 != gps_tb_index2)	//Transmit software buffer not empty
	   	  {
		      U0THR = gps_trans_buffer[gps_tb_index2];	//Transmit oldest byte from software tx buffer
          if (gps_tb_index2 == GPS_TX_BUFFER_SIZE - 1)
          {
            gps_tb_index2 = 0;  //Roll over to zero
          }
          else
          {
            gps_tb_index2++;    //Increment write index
          }
	   	  }
	      else   //Transmit software buffer is empty, disable transmit interrrupts
	      {
		      U0IER &= (~(1<<1)) & 0x1FF;		//Disable tranmitter holding register empty interrupt, bit 1
		      break;
	     }
	   }	 
	  break;
	 
	  case 6:	//receive line status interrupt
	 	line_status = U1LSR;
		switch(line_status & 30){
			case 1: //Overrun Error(OE)
				//mcu_led_red_blink(500);
				break;
			case 2: //Parity Error(PE)
				//mcu_led_green_blink(500);
				break;
			case 4: //Framing Error(FE)
				//mcu_led_red_blink(2000);
				break;
			case 8: //Break Interrupt(BI)
				//mcu_led_green_blink(2000);
				break;
		}
	 break;	
	 	   
  }		   
  
  VICVectAddr = 0;		//Reset interrupt priorities
}	 


/**
  Push GPS command packet in array bytes onto tx software ring buffer.
  @param bytes An array of bytes to send.
  @param length The length of the array of bytes.
*/
void gps_send_packet(unsigned char * bytes, unsigned long length)
{
  unsigned long i, temp_index;

  for (i=0;i<length;i++)
  {
    temp_index = gps_tb_index1;
    if (temp_index == GPS_TX_BUFFER_SIZE - 1)
    {
      temp_index = 0;   //roll over to zero
    }
    else
    {
      temp_index++;     //increment temp_index
    }
    if (temp_index != gps_tb_index2)
    {
      gps_trans_buffer[gps_tb_index1] = bytes[i];
      gps_tb_index1 = temp_index;
    }
    else
    {
      error_occurred(ERROR_GPS_TX_FULL);
      break;  
    }
  }
  U0IER |= (1<<1) & 0x1FF;    //Re-enable transmit interrupts as needed (mask off reserved bits)
  //This should be ok even if the interrupt happened in the previous line and the buffer is now empty.
  //An interrupt will occur, but then be immediately disabled.	
}

/**
  Returns data from the IMU as a float.
  @param index The index of the data from the packet. @see MSIMU_DATA
  @return The requested data interpreted as a float. @note This function only 
  interprets the bits coming in over the UART as a floating point number, it does
  @b NOT convert from an integer (1) to float (1.0). 
*/
float gps_get_data_float(MSIMU_DATA index){
	void *vpointer;
	float *fpointer;
  float data;
  vpointer = gps_data_buffer + index;
  fpointer = vpointer; 
  data = *fpointer;
  return data;
}

/**
  Returns data from the IMU as an int.
  @param index The index of the data from the packet. @see MSIMU_DATA
  @return The requested data interpreted as an int. @note This function only 
  interprets the bits coming in over the UART as an integer, it does
  @b NOT convert from an float (1.4) to int (1). 
*/
int gps_get_data_int(MSIMU_DATA index){
  return (int)gps_data_buffer[index];
}

/**
  GPS update function, which calls initialization
  and parsing functions. Call this at a regular interval,
  at a sufficient rate to avoid rx buffer overflow.
  (Or increase the rx buffer size.)
*/
void gps_update(void)
{
  gps_parse_buffer();
}

/**
  Parse the rx software buffer to check for a new packet; distribute data appropriately.
*/
void gps_parse_buffer(void)
{
	#define GPS_MAX_LOG_SIZE 256
  static unsigned char data_buffer[GPS_MAX_LOG_SIZE]; //Holds data bytes from the gps, pending CRC check and data distribution to output struct
  static unsigned long state = 0;   //State = 0, looking for sync bytes; state = 1, building packet.
  static unsigned char sync1 = 0, sync2 = 0, sync3 = 0;
  static unsigned long header_length = 0, message_length = 0, packet_size = 0;
  static unsigned long i = 0;
  unsigned long tempindex;
			
	tempindex = gps_rb_index2;
  if (tempindex == GPS_RX_BUFFER_SIZE - 1)
  {
    tempindex = 0;    //roll over to zero
  }
  else
  {
    tempindex++;      //increment tempindex, so it points to next potential unread data
  }

  //Check state: looking for sync bytes?
  if (state == 0) //Check for sync bytes
  {
    while (tempindex != gps_rb_index1) //Software rx buffer is not empty, process a byte
    {
      sync1 = sync2;  //Sync 1 - 3 form a little shift register. Move new data in at sync3, old out at sync1
      sync2 = sync3;
      sync3 = gps_read_buffer[tempindex];
      gps_rb_index2 = tempindex;
      //move tempindex to new value
      if (tempindex == GPS_RX_BUFFER_SIZE - 1)
      {
        tempindex = 0;    //roll over to zero
      }
      else
      {
        tempindex++;      //increment tempindex
      }

      if ((sync1 == 0xAA) && (sync2 == 0x44) && (sync3 == 0x12))
      {
        state = 1;    //OEMStar message header sync bytes match
        data_buffer[0] = sync1;  //Load sync bytes into data_buffer slots
        data_buffer[1] = sync2;
        data_buffer[2] = sync3;
        sync1 = 0;  //Reset sync variables to zero
        sync2 = 0;
        sync3 = 0;
        i = 3;  //point i to next slot in data_buffer
        break;
      }
    }
  }

  //Check state: building packet?
  if (state == 1)  //Build packet
  {
    while (tempindex != gps_rb_index1) //Software rx buffer is not empty, process a byte
    {
      if (i <= 9)
        {
        if (i == 3) //header length byte
        {
          header_length = gps_read_buffer[tempindex];
        }
        if (i == 8) //message length, lower byte
        {
          message_length = gps_read_buffer[tempindex];
        }
        if (i == 9) //message length, upper byte
        {
          message_length |= (gps_read_buffer[tempindex] << 8);
          packet_size = header_length + message_length;
        }
        if (packet_size > GPS_MAX_LOG_SIZE)
        {
          i = 0;               //Reset static variables; go back to looking for sync bytes.
          state = 0;
          header_length = 0;
          message_length = 0;
          packet_size = 0;
          sync1 = 0; sync2 = 0; sync3 = 0;
          break;
        }
        data_buffer[i] = gps_read_buffer[tempindex];
      }
      else if (i < packet_size)
      {

      }
      else if // i = packet_size + 3: all the packet data plus 4 CRC bytes.
      {

      }
    }    
  }  

	while ((gps_read_buffer[tempindex2 & 127] != gps_continuous_command) && ((tempindex1 - tempindex2) > packet_length))
	{
		tempindex2++;				//Find first byte in buffer equal to command
									//while still leaving enough data to parse a full packet
									//Leading characters that don't match are discarded
    if (tempindex2 > gps_rb_index1)
	  {
		  tempindex1 = gps_rb_index1 + 128;
	  }
	  else
	  {
		  tempindex1 = gps_rb_index1;
	  }
	}
  	
	gps_rb_index2 = (tempindex2 & 127);		//Adjust Index2 to point to first potential
												//command byte or last character checked,
												//thus skipping over any leading
												//unrecognized characters.
		
	//COMMAND byte found? Parse first packet
	if (gps_read_buffer[gps_rb_index2] == gps_continuous_command && ((tempindex1 - tempindex2) > packet_length))
	{
		//Calculate checksum
		cal_checksum = 0;
		
		for (i = 0; i < (packet_length-2); i++)
		{
			cal_checksum += gps_read_buffer[(gps_rb_index2 + i) & 127];		
		}
		
		cal_checksum &= 0xFFFF;			//truncate calculated checksum to 16 bits, to match received checksum
		
		//read in received checksum from end of potential packet
		rec_checksum = gps_read_buffer[(gps_rb_index2 + packet_length - 1) & 127];
		rec_checksum |= ((gps_read_buffer[(gps_rb_index2 + packet_length - 2) & 127])<<8);
				
		//Valid packet found? Parse and put received values into global variables
		if (cal_checksum == rec_checksum)
		{		
	      i = 1;
	      while (i < packet_length - 2) {
  	      ulitemp = (unsigned long int)gps_read_buffer[(gps_rb_index2 + i++) & 127];
  	  		ulitemp <<= 8;
  	  		ulitemp |= (unsigned long int)gps_read_buffer[(gps_rb_index2 + i++) & 127];
  	  		ulitemp <<= 8;
  	  		ulitemp |= (unsigned long int)gps_read_buffer[(gps_rb_index2 + i++) & 127];
  	  		ulitemp <<= 8;
  	  		ulitemp |= (unsigned long int)gps_read_buffer[(gps_rb_index2 + i++) & 127];  
          gps_data_buffer[data_count] = ulitemp;
          data_count++; 
	      }
			
			//Advance pointer to next unread buffer location
			gps_rb_index2 = ((gps_rb_index2 + packet_length) & 127);
		}
		
		else	//not a valid packet - look for next potential command byte
		{
			gps_rb_index2 = (++gps_rb_index2 & 127);	//Move one step past old potential command byte in buffer
		}
		
	}
}
