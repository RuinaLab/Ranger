#include <includes.h>


//The following hardware setup code, in some form, needs to run for correct operation of this module
//***to be determined

/*
// Hardware initialization for MicroStrain IMU interface

//
//Resources used in this example implementation for LPC2194/01:
//UART1 serial interface (port pins P0[8]-TXD1 and P0[9]-RXD1)
//Vectored interrupt slot 3

//Assumed:
//60 MHz peripheral clock to UART1
//115.2 Kbaud serial interface speed to IMU
//8 data bits
//1 stop bit
//no parity
//no handshaking

  //UART1 setup
  U1LCR=0x83;  //Step 1: DLAB=1, enable baud rate adjustments
  			   //Set to 8 data bits, 1 stop bit, no parity

			   //Step 2: Set baud rate divisor and fractional baud rate divisors
  U1DLL=25;		//Baud rate divisor for 115200 kbaud, with (1+0.3) fractional divider 
  U1DLM=0;		//and peripheral clock = 60MHz.

  U1FDR = 3;		// DIVADDVAL = 3
  U1FDR |= 10<<4;	// MULTVAL = 10

  U1FCR = 0x81;    	// Enable receive and transmit FIFO; interrupt on receive at 8 bytes.
  U1MCR = 0;	   	// Loopback mode, CTS and RTS flow control disabled.
  U1ACR = 0;	   	// Autobaud disabled
  U1TER = 0x80;	   	// Transmit enabled

  U1LCR &= ~(1<<7); // Step 3: DLAB=0, disable baud rate adjustments
  PINSEL0 |= 1<<16;    // UART1 TXD active on MCU pin P0[8]
  PINSEL0 &= ~(1<<17); //
  PINSEL0 |= 1<<18;    // UART1 RXD	active on MCU pin P0[9]
  PINSEL0 &= ~(1<<19); // 

  VICProtection = 0;		//Allow modification of VIC registers in user mode
  VICVectAddr0 = (unsigned long)ms_imu_isr;	//Set address of interrupt service routine
  VICvectCntl3 = (1<<5) + 7;	//Enable vectored interrupt slot 3 and set it to UART1 (interrupt source 7)
  VICIntSelect &= ~(1<<7)	//Clear bit 7 - use IRQ interrupts for UART1, not FIQ.
  VICIntEnable = 1<<7;		//Enable UART1 interrupt at vectored interrupt controller (VIC)
*/

//The following macros need to be defined in the software setup file for correct operation of this module
//	#define MS_IMU_COMMAND 0xCF			//Command code for Euler angle and angular rate data
//	#define MS_IMU_LENGTH 31			//Number of bytes in packet returned by command code


//Global received data variables
float MS_IMU_x_euler_angle;
float MS_IMU_y_euler_angle;
float MS_IMU_z_euler_angle;

float MS_IMU_x_angular_rate;
float MS_IMU_y_angular_rate;
float MS_IMU_z_angular_rate;

long int MS_IMU_elapsed_time;



//Global variables for MicroStrain IMU software receive buffer
unsigned char MS_IMU_ReadBuffer[128];		//FIFO read buffer for data from UART
unsigned char MS_IMU_RB_Index1 = 0;			//Points to location of next received byte
unsigned char MS_IMU_RB_Index2 = 0;			//Points to location of oldest unused rec. byte
											//If Index2 = Index1, buffer contains no unused data.

//Global variables for MicroStrain IMU software transmit buffer
unsigned char MS_IMU_TransBuffer[128];		//FIFO transmit buffer for data to UART
unsigned char MS_IMU_TB_Index1 = 0;			//Points to location of next byte to be added to buffer
unsigned char MS_IMU_TB_Index2 = 0;			//Points to location of oldest byte not yet transmitted
											//If Index2 = Index1, buffer contains no unsent data.


//Global variables for IMU commands
const unsigned char MS_IMU_EulerAngRateCommand[4] = {0xC4,0xC1,0x29,0xCF};  //stream output, euler angles and ang. rates

//Sends out MS_IMU_EulerAngRateCommand to IMU
void ms_imu_init(void)
{
 	ms_imu_send_char_array(&(MS_IMU_EulerAngRateCommand[0]),4);
}

//Send char array to LPC2xxx hardware FIFO buffer (transmitter holding register)
void ms_imu_send_char_array(const unsigned char *array, unsigned short int length)
{
	unsigned short int i,bufferspace;
	
	if ((length <= 16) && (U1LSR & (1<<5)))		//length must be smaller than 16-byte FIFO
												//and FIFO must be empty
	{										
		for (i=0;i<length;i++)
		{
			U1THR = *(array + i);
		}		
	 }

	 //	else{}	//Buffer full error
	
}

/*	Version below uses software FIFO, in addition to hardware FIFO
void ms_imu_send_char_array(const unsigned char *array, unsigned short int length)
{
	unsigned short int i,bufferspace;
	
	if (MS_IMU_TB_Index1 >= MS_IMU_TB_Index2)
	{
		bufferspace = (MS_IMU_TB_Index2 + 128) - MS_IMU_TB_Index1;	//How much more room in the buffer?
	}
	else
	{
		bufferspace = MS_IMU_TB_Index2 - MS_IMU_TB_Index1;
	}

	if (length < bufferspace)	//length of the new array to send must be less than
								//the remaining space in the buffer. If equal, the indexes
								//will become equal, and the function will think the buffer is empty.
	{
		
		U1THR = *(array + 0);			//Put first char directly into hardware FIFO
										//to trigger THRE interrupt
//		for (i=0;i<length;i++)			//old version, only goes to software buffer	
		for (i=1;i<length;i++)
		{
			MS_IMU_TransBuffer[MS_IMU_TB_Index1] = *(array + i);
			MS_IMU_TB_Index1++;
		}

		U1IER |= (1<<1);		//Enable tranmitter holding register empty interrupt, bit 1;
		
	}
//	else{}	//Buffer full error
	
	
}
*/


void ms_imu_isr(void) __irq
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
	     MS_IMU_ReadBuffer[MS_IMU_RB_Index1] = U1RBR;	//Put new data in next buffer location
	     MS_IMU_RB_Index1 = (++MS_IMU_RB_Index1 & 127);		//Increment index; roll over to 0 after reaching 127	

	     if (MS_IMU_RB_Index1 == MS_IMU_RB_Index2)			//software receive software overflow
	     {
	       MS_IMU_RB_Index1 = (--MS_IMU_RB_Index1 & 127);	//overwrite most recent byte
		   //Error - read buffer overflow
         }	
	   }
	 break;

	 case 12:	//character time-out - at least one left-over byte in hardware receive FIFO
	   line_status = U1LSR;
	   MS_IMU_ReadBuffer[MS_IMU_RB_Index1] = U1RBR;	//Put new data in next buffer location
	   MS_IMU_RB_Index1 = (++MS_IMU_RB_Index1 & 127);		//Increment index; roll over to 0 after reaching 127	

	   if (MS_IMU_RB_Index1 == MS_IMU_RB_Index2)		//software receive buffer overflow
	   {
		
	     MS_IMU_RB_Index1 = (--MS_IMU_RB_Index1 & 127);	 //overwrite most recent byte
		//Error - read buffer overflow
	   }	
	 
	 break;
	 
	 case 2:	//transmitter hardware FIFO empty
	   for (i=0;i<16;i++) 	//Move up to 16 bytes (FIFO capacity) to FIFO
	   {
	     if (MS_IMU_TB_Index1 != MS_IMU_TB_Index2)	//Transmit software FIFO not empty
	   	 {
		   U1THR = MS_IMU_TransBuffer[MS_IMU_TB_Index2];	//Transmit oldest byte from software FIFO
		   MS_IMU_TB_Index2 = ((++MS_IMU_TB_Index2) & 127);		//Increment Index2, roll over at 128
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
		IO1CLR = 1<<24;	// ***test code, turn on red LED


	 
	 break;
	 
  }

  VICVectAddr = 0;		//Reset interrupt priorities
}

//Read MicroStrain IMU incoming data from receive buffer FIFO; parse and
//store in float array locations as assigned by input parameters.
//Presently set to parse only 0xCF IMU packets, with angular rate and Euler
//angle orientation data, and only the Y axis data. However, it is easily
//expanded.
void ms_imu_parse_buffer(void)
{
	unsigned short int tempindex1 = 0, tempindex2 = 0, i;
	unsigned long int cal_checksum = 0, rec_checksum = 0, ulitemp = 0;
	float test1 = 0, test2 = 0;
	void *vpointer;
	float *fpointer;
		
	#define MS_IMU_COMMAND 0xCF			//Command code for Euler angle and angular rate data packet
	#define MS_IMU_LENGTH 31			//Number of bytes in this specific return packet
			
	tempindex2 = MS_IMU_RB_Index2;
	if (tempindex2 > MS_IMU_RB_Index1)
	{
		tempindex1 = MS_IMU_RB_Index1 + 128;
	}
	else
	{
		tempindex1 = MS_IMU_RB_Index1;
	}

	while ((MS_IMU_ReadBuffer[tempindex2 & 127] != MS_IMU_COMMAND) && ((tempindex1 - tempindex2) > MS_IMU_LENGTH))
	{
		tempindex2++;				//Find first byte in buffer equal to MS_IMU_COMMAND
									//while still leaving enough data to parse a full packet
									//Leading characters that don't match are discarded
	}
	
	
	MS_IMU_RB_Index2 = (tempindex2 & 127);		//Adjust Index2 to point to first potential
												//command byte or last character checked,
												//thus skipping over any leading
												//unrecognized characters.
		
	//COMMAND byte found? Parse first packet
	if (MS_IMU_ReadBuffer[MS_IMU_RB_Index2] == MS_IMU_COMMAND && ((tempindex1 - tempindex2) > MS_IMU_LENGTH))
	{
		//Calculate checksum
		cal_checksum = 0;
		
		for (i = 0; i < (MS_IMU_LENGTH-2); i++)
		{
			cal_checksum += MS_IMU_ReadBuffer[(MS_IMU_RB_Index2 + i) & 127];		
		}
		
		cal_checksum &= 0xFFFF;			//truncate calculated checksum to 16 bits, to match received checksum
		
		//read in received checksum from end of potential packet
		rec_checksum = MS_IMU_ReadBuffer[(MS_IMU_RB_Index2 + MS_IMU_LENGTH - 1) & 127];
		rec_checksum |= ((MS_IMU_ReadBuffer[(MS_IMU_RB_Index2 + MS_IMU_LENGTH - 2) & 127])<<8);
				
		//Valid packet found? Parse and put received values into global variables
		if (cal_checksum == rec_checksum)
		{	
			ulitemp = (unsigned long int)MS_IMU_ReadBuffer[(MS_IMU_RB_Index2 + 5) & 127];
			ulitemp <<= 8;
			ulitemp |= (unsigned long int)MS_IMU_ReadBuffer[(MS_IMU_RB_Index2 + 6) & 127];
			ulitemp <<= 8;
			ulitemp |= (unsigned long int)MS_IMU_ReadBuffer[(MS_IMU_RB_Index2 + 7) & 127];
			ulitemp <<= 8;
			ulitemp |= (unsigned long int)MS_IMU_ReadBuffer[(MS_IMU_RB_Index2 + 8) & 127];
			vpointer = &ulitemp;			//Get void pointer to new data
			fpointer = vpointer;			//Create float pointer to new data
			MS_IMU_y_euler_angle = *fpointer;	//put floating point value from packet in assigned data location
			test1 =   MS_IMU_y_euler_angle;
			if (test1>0)
			{
				IO1CLR = 1<<23;	// ***test code
			}
			else
			{
   				IO1SET = 1<<23;	// ***test code
			}

			ulitemp = (unsigned long int)MS_IMU_ReadBuffer[(MS_IMU_RB_Index2 + 17) & 127];
			ulitemp <<= 8;
			ulitemp |= (unsigned long int)MS_IMU_ReadBuffer[(MS_IMU_RB_Index2 + 18) & 127];
			ulitemp <<= 8;
			ulitemp |= (unsigned long int)MS_IMU_ReadBuffer[(MS_IMU_RB_Index2 + 19) & 127];
			ulitemp <<= 8;
			ulitemp |= (unsigned long int)MS_IMU_ReadBuffer[(MS_IMU_RB_Index2 + 20) & 127];
			vpointer = &ulitemp;			//Get void pointer to new data
			fpointer = vpointer;			//Create float pointer to new data
			MS_IMU_y_angular_rate = *fpointer;	//put float value in assigned data location
			test2 =  MS_IMU_y_angular_rate;
			
			//Advance pointer to next unread buffer location
			MS_IMU_RB_Index2 = ((MS_IMU_RB_Index2 + MS_IMU_LENGTH) & 127);
		}
		
		else	//not a valid packet - look for next potential command byte
		{
			MS_IMU_RB_Index2 = (++MS_IMU_RB_Index2 & 127);	//Move one step past old potential command byte in buffer
		}
		
	}
}

