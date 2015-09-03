/**
	@file microstrain_imu.c
	
	Control code for the Microstrain IMU on the UI board.
  
  Example hardware setup for communicating with IMU over UART:
  @code
  // *******************************************************************************
	// Initialize Microstrain IMU over UART1
	// *******************************************************************************
  U1LCR=0x83;  //Step 1: DLAB=1, enable baud rate adjustments
  //Set to 8 data bits, 1 stop bit, no parity
	//Step 2: Set baud rate divisor and fractional baud rate divisors
  U1DLL=25;		//Baud rate divisor for 115200 kbaud, with (1+0.3) fractional divider 
  U1DLM=0;		//and peripheral clock = 60MHz.
  U1FDR = 3;		// DIVADDVAL = 3
  U1FDR |= 10<<4;	// MULTVAL = 10
  //U1FCR = 0x81;    	// Enable receive and transmit FIFO; interrupt on receive at 8 bytes.
  U1FCR = 0x1;		// Enable FIFO
  U1FCR = 0x87;		//Clear FIFOs and set to generate interrupt when RX FIFO reaches 8 bytes.
  U1MCR = 0;	   	// Loopback mode, CTS and RTS flow control disabled.
  U1ACR = 0;	   	// Autobaud disabled
  U1TER = 0x80;	   	// Transmit enabled
  U1LCR &= ~(1<<7); // Step 3: DLAB=0, disable baud rate adjustments
  PINSEL0 |= 1<<16;    // UART1 TXD active on MCU pin P0[8]
  PINSEL0 &= ~(1<<17); //
  PINSEL0 |= 1<<18;    // UART1 RXD	active on MCU pin P0[9]
  PINSEL0 &= ~(1<<19); //
  U1IER = (1<<0); // enable RBR interrupts
  U1ACR = (1<<8) | (1<<9); //clear the corresponding interrupt in the U1IIR
  @endcode
  
  @author Nicolas Williamson
  @author Jason Cortell
  @date December 2009
	
*/

#include <includes.h>

//Global variables for MicroStrain IMU software receive buffer
volatile unsigned char msimu_read_buffer[128];		//FIFO read buffer for data from UART
volatile unsigned char msimu_rb_index1 = 0;			//Points to location of next received byte
volatile unsigned char msimu_rb_index2 = 0;			//Points to location of oldest unused rec. byte
//If Index2 = Index1, buffer contains no unused data.

//Global variables for MicroStrain IMU software transmit buffer
volatile unsigned char msimu_trans_buffer[128];		//FIFO transmit buffer for data to UART
volatile unsigned char msimu_tb_index1 = 0;			//Points to location of next byte to be added to buffer
volatile unsigned char msimu_tb_index2 = 0;			//Points to location of oldest byte not yet transmitted
//If Index2 = Index1, buffer contains no unsent data.

unsigned long msimu_data_buffer[20]; //Holds untyped data from the imu

MSIMU_COMMAND msimu_continuous_command;
                      
//Initializes the imu with a command in continuous mode
/**
  Initializes the microstrin IMU module.
  @param command The data that the IMU will continuously calculate 
  and send to the board over UART.
*/

//IMU continuous data transmission inititialization only - don't run any other time.
void msimu_init(MSIMU_COMMAND command)
{
  float i = 0;

  //// Aug 29, 2015. MPK changed 10000000 to 20000000 with Jason in an attempt to fix IMU start-up bug.////
  while (i < 20000000)   //Confuse the compiler into leaving this delay loop here
  {
    i = i + 1;
    i = i * 1.001;   
  }

  msimu_set_continuous(command);

}

/**
  Returns the length of a return packet associated with a given command.
  @note Not all commands have been added. If you do not see yours listed,
  add it to the switch statement.
  @param command The command to get the return packet length of.
  @return Length of the expected return packet, or 31 if command is unknown.
*/
int msimu_get_length(MSIMU_COMMAND command){
  int length = 0;
  switch (command){
    case MSIMU_RAW_ACCEL_ANG_RATE: length = 31; break;
    case MSIMU_EULER_ANGS_ANG_RATE: length = 31; break;
    default: length = 31;
  }
  return length;
}

/**
  The interrupt service routine for communicating with the IMU over UART.
  Stores incoming data into buffers.
*/
void msimu_isr(void) __irq
{	
	unsigned long int interrupt_id, line_status;
	unsigned short int i;

	interrupt_id = U1IIR;
	//test_counter++;
	
	switch (interrupt_id & 0xE) //Look at bits 1 - 3 of U1IIR
	{
	 case 4:	//receive data available (>=8 bytes in hardware receive FIFO, as set in U1FCR)
	   for (i=0;i<8;i++)
	   {
	     line_status = U1LSR;
	     msimu_read_buffer[msimu_rb_index1] = U1RBR;	//Put new data in next buffer location
	     msimu_rb_index1 = (++msimu_rb_index1 & 127);		//Increment index; roll over to 0 after reaching 127	

	     if (msimu_rb_index1 == msimu_rb_index2)			//software receive software overflow
	     {
	       msimu_rb_index1 = (--msimu_rb_index1 & 127);	//overwrite most recent byte
		     error_occurred(ERROR_MSIMU_RX_OF);
       }	
	   }
	 break;

	 case 12:	//character time-out - at least one left-over byte in hardware receive FIFO
	   line_status = U1LSR;
	   msimu_read_buffer[msimu_rb_index1] = U1RBR;	//Put new data in next buffer location
	   msimu_rb_index1 = (++msimu_rb_index1 & 127);		//Increment index; roll over to 0 after reaching 127	

	   if (msimu_rb_index1 == msimu_rb_index2)		//software receive buffer overflow
	   {
	     msimu_rb_index1 = (--msimu_rb_index1 & 127);	 //overwrite most recent byte
		   error_occurred(ERROR_MSIMU_RX_OF);
	   }	
	 
	 break;
	 
	 case 2:	//transmitter hardware FIFO empty
	   for (i=0;i<16;i++) 	//Move up to 16 bytes (FIFO capacity) to FIFO
	   {
	     if (msimu_tb_index1 != msimu_tb_index2)	//Transmit software FIFO not empty
	   	 {
		   U1THR = msimu_trans_buffer[msimu_tb_index2];	//Transmit oldest byte from software FIFO
		   msimu_tb_index2 = ((++msimu_tb_index2) & 127);		//Increment Index2, roll over at 128
	   	 }
	     else
	     {
		   U1IER &= ~(1<<1);		//Disable tranmitter holding register empty interrupt, bit 1
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
  Starts the IMU in continuous mode with the given command.
  @param command The command for the IMU to continuously compute.
*/
void msimu_set_continuous(MSIMU_COMMAND command){
//  unsigned char bytes1[] = {0xD5,0xBA,0x89,0x02,0xD6,0xC6,0x6B,0x00}; //This code sets IMU to send continuously
                                                                        //at power-on. Hard to stop again!!!
  unsigned char bytes2[] = {0xC4,0xC1,0x29,0x00};

    msimu_continuous_command = command;
//  bytes1[7] = command;
//  msimu_send_all(bytes1, 8);

  bytes2[3] = command;
  msimu_send_all(bytes2, 4);
  
}

/**
  Sends bytes to the IMU over UART.
  @param bytes An array of bytes to send.
  @param length The length of the array of bytes.
*/
void msimu_send_all(unsigned char *bytes, unsigned int length)
{
  unsigned int i;
	if ((length <= 16))		//length must be smaller than 16-byte FIFO and FIFO must be empty
	{										
    if (U1LSR & (1<<5)){
  		for (i=0;i<length;i++){
  			U1THR = *(bytes + i);
  		}	
    } else {
    	error_occurred(ERROR_MSIMU_TX_FULL);
    }
	} else {
    error_occurred(ERROR_MSIMU_NUM_BYT);
  }
}

/**
  Returns data from the IMU as a float.
  @param index The index of the data from the packet. @see MSIMU_DATA
  @return The requested data interpreted as a float. @note This function only 
  interprets the bits coming in over the UART as a floating point number, it does
  @b NOT convert from an integer (1) to float (1.0). 
*/
float msimu_get_data_float(MSIMU_DATA index){
	void *vpointer;
	float *fpointer;
  float data;
  vpointer = msimu_data_buffer + index;
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
int msimu_get_data_int(MSIMU_DATA index){
  return (int)msimu_data_buffer[index];
}

/**
  Updates the IMU module. Parses data from UART and check if there is
  a new packet. Call this from every row of the schedule.
*/
void msimu_update(void){msimu_parse_buffer();}

/**
  Parses the UART buffer to check for a new packet, saving the data if there
  has been a new packet.
*/
void msimu_parse_buffer(void)
{
	unsigned short int tempindex1 = 0, tempindex2 = 0, i;
	unsigned long int cal_checksum = 0, rec_checksum = 0, ulitemp = 0;
  int data_count = 0;
  int packet_length = 0;

	packet_length = msimu_get_length(msimu_continuous_command);//Number of bytes in this specific return packet
			
	tempindex2 = msimu_rb_index2;
	if (tempindex2 > msimu_rb_index1)
	{
		tempindex1 = msimu_rb_index1 + 128;
	}
	else
	{
		tempindex1 = msimu_rb_index1;
	}

	while ((msimu_read_buffer[tempindex2 & 127] != msimu_continuous_command) && ((tempindex1 - tempindex2) > packet_length))
	{
		tempindex2++;				//Find first byte in buffer equal to command
									//while still leaving enough data to parse a full packet
									//Leading characters that don't match are discarded
    if (tempindex2 > msimu_rb_index1)
	  {
		  tempindex1 = msimu_rb_index1 + 128;
	  }
	  else
	  {
		  tempindex1 = msimu_rb_index1;
	  }
	}
  	
	msimu_rb_index2 = (tempindex2 & 127);		//Adjust Index2 to point to first potential
												//command byte or last character checked,
												//thus skipping over any leading
												//unrecognized characters.
		
	//COMMAND byte found? Parse first packet
	if (msimu_read_buffer[msimu_rb_index2] == msimu_continuous_command && ((tempindex1 - tempindex2) > packet_length))
	{
		//Calculate checksum
		cal_checksum = 0;
		
		for (i = 0; i < (packet_length-2); i++)
		{
			cal_checksum += msimu_read_buffer[(msimu_rb_index2 + i) & 127];		
		}
		
		cal_checksum &= 0xFFFF;			//truncate calculated checksum to 16 bits, to match received checksum
		
		//read in received checksum from end of potential packet
		rec_checksum = msimu_read_buffer[(msimu_rb_index2 + packet_length - 1) & 127];
		rec_checksum |= ((msimu_read_buffer[(msimu_rb_index2 + packet_length - 2) & 127])<<8);
				
		//Valid packet found? Parse and put received values into global variables
		if (cal_checksum == rec_checksum)
		{		
	      i = 1;
	      while (i < packet_length - 2) {
  	      ulitemp = (unsigned long int)msimu_read_buffer[(msimu_rb_index2 + i++) & 127];
  	  		ulitemp <<= 8;
  	  		ulitemp |= (unsigned long int)msimu_read_buffer[(msimu_rb_index2 + i++) & 127];
  	  		ulitemp <<= 8;
  	  		ulitemp |= (unsigned long int)msimu_read_buffer[(msimu_rb_index2 + i++) & 127];
  	  		ulitemp <<= 8;
  	  		ulitemp |= (unsigned long int)msimu_read_buffer[(msimu_rb_index2 + i++) & 127];  
          msimu_data_buffer[data_count] = ulitemp;
          data_count++; 
	      }
			
			//Advance pointer to next unread buffer location
			msimu_rb_index2 = ((msimu_rb_index2 + packet_length) & 127);
		}
		
		else	//not a valid packet - look for next potential command byte
		{
			msimu_rb_index2 = (++msimu_rb_index2 & 127);	//Move one step past old potential command byte in buffer
		}
		
	}
}
