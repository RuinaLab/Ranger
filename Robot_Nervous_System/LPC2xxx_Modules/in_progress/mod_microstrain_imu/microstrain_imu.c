/*
	microstrain_imu.c
	
	Control code for the Microstrain IMU for Cornell's Ranger Robot
	
	Nicolas Williamson - Dec. 2009
	Using code by Jason Cortell
*/

#include <includes.h>

//Global variables for MicroStrain IMU software receive buffer
unsigned char msimu_read_buffer[128];		//FIFO read buffer for data from UART
unsigned char msimu_rb_index1 = 0;			//Points to location of next received byte
unsigned char msimu_rb_index2 = 0;			//Points to location of oldest unused rec. byte
//If Index2 = Index1, buffer contains no unused data.

//Global variables for MicroStrain IMU software transmit buffer
unsigned char msimu_trans_buffer[128];		//FIFO transmit buffer for data to UART
unsigned char msimu_tb_index1 = 0;			//Points to location of next byte to be added to buffer
unsigned char msimu_tb_index2 = 0;			//Points to location of oldest byte not yet transmitted
//If Index2 = Index1, buffer contains no unsent data.

unsigned long msimu_data_buffer[20]; //Holds untyped data from the imu

MSIMU_COMMAND msimu_continuous_command = 0x00;
                      
//Initializes the imu with a command in continuous mode
void msimu_init(MSIMU_COMMAND command)
{
  if (continuous){
    msimu_send_continuous(command);
  } else {
    msimu_polled_command = command;
  }
}

//Returns the length of the return packet for a given MSIMU_COMMAND.
//If you want to use the IMU for a command not listed here, you will
//need to add it.
int msimu_get_length(MSIMU_COMMAND command){
  int length = 0;
  switch (command){
    case MSIMU_RAW_ACCEL_ANG_RATE: length = 31; break;
    case MSIMU_EULER_ANGS_ANG_RATE: length = 31; break;
    default: length = 31;
  }
  return length;
}

//Handles an interrupt from the IMU
void msimu_isr(void) __irq
{	
	unsigned long int interrupt_id, line_status;
	unsigned short int i;

	interrupt_id = U1IIR;
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
		     error_occurred(ERROR_MSIMU_RX_BUFFER_OVERFLOW,PRIOTITY_HIGH);
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
		   error_occurred(ERROR_MSIMU_RX_BUFFER_OVERFLOW,PRIOTITY_HIGH);
	   }	
	 
	 break;
	 
	 case 2:	//transmitter hardware FIFO empty
	   for (i=0;i<16;i++) 	//Move up to 16 bytes (FIFO capacity) to FIFO
	   {
	     if (msimu_tb_index1 != msimu_tb_index2)	//Transmit software FIFO not empty
	   	 {
		   U1THR = msinu_trans_buffer[msimu_tb_index2];	//Transmit oldest byte from software FIFO
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
	 break;
	 
  }

  VICVectAddr = 0;		//Reset interrupt priorities
}

//Starts the imu in continuous mode with the given command
void msimu_send_continuous(MSIMU_COMMAND command){
  msimu_continuous_command = command;
  msimu_send_all([0xC4,0xC1,0x29,command], 4);
}

/*//If in polled mode, tells the imu to update the value with command msimu_update_command
void msimu_update(void){
  msimu_send_byte(msimu_polled_command);
}*/

/*//Tells the imu to execute the given command
void msimu_send_polled(MSIMU_COMMAND command){
  msimu_polled_command = command;
  msimu_send_byte(command);
}*/

/*//Sends byte to LPC2xxx hardware FIFO buffer (transmitter holding register)
void msimu_send_byte(unsigned char byte)
{
  if (U1LSR & (1<<5)){
	  U1THR = byte;
  } else {
    error_occurred(ERROR_MSIMU_TRANSMIT_FULL, PRIORITY_MED);
  }
}*/

//Sends all of the bytes
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
    	error_occurred(ERROR_MSIMU_TRANSMIT_FULL, PRIORITY_MED);
    }
	} else {
    error_occurred(ERROR_MSIMU_TOO_MANY_BYTES, PRIORITY_HIGH);
  }
}

//Get the data from the imu data buffer read as float
void msimu_get_data_float(int index){
	void *vpointer;
	float *fpointer;
  float data;
  vpointer = msimu_data_buffer + index;
  fpointer = vpointer; 
  data = *fpointer;
  return data;
}

//Gets the data from the imu data buffer as an int
void msimu_get_data_int(int index){
  return msimu_data[index];
}

//Updates the imu, parsing a packet from the buffer if able
void msimu_update(void){msimu_parse_buffer();}

//Read MicroStrain IMU incoming data from receive buffer FIFO; parse and
//store in float array locations as assigned by input parameters.
void msimu_parse_buffer(void)
{
	unsigned short int tempindex1 = 0, tempindex2 = 0, i;
	unsigned long int cal_checksum = 0, rec_checksum = 0, ulitemp = 0;
	float test1 = 0, test2 = 0;
  int data_count = 0;
		
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
		
		for (i = 0; i < (pakcet_length-2); i++)
		{
			cal_checksum += msimu_read_buffer[(msimu_rb_index2 + i) & 127];		
		}
		
		cal_checksum &= 0xFFFF;			//truncate calculated checksum to 16 bits, to match received checksum
		
		//read in received checksum from end of potential packet
		rec_checksum = msimu_read_buffer[(msimu_rb_index2 + pakcet_length - 1) & 127];
		rec_checksum |= ((msimu_read_buffer[(msimu_rb_index2 + pakcet_length - 2) & 127])<<8);
				
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
