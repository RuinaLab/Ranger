#include <mb_includes.h>

//Global variables

////////////////////////////////////////////////////////////////
//Global variables for BlueTooth serial software receive buffer//////////
////////////////////////////////////////////////////////////////

//#define A9_BT_RX_BUFFER_SIZE 500	  	//receive ring buffer size
const unsigned short A9_BT_RX_BUFFER_SIZE = 500;

unsigned volatile char a9_bt_rx_buffer[A9_BT_RX_BUFFER_SIZE];
unsigned volatile short int a9_bt_rx_index1 = 0;		//Points to address of next received character
unsigned volatile short int a9_bt_rx_index2 = 0;		//Points to oldest unread character in receive buffer
/////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
//Global variables for DMA BlueTooth serial software transmit buffer//////////
////////////////////////////////////////////////////////////////

//The initial design is for BlueTooth transmit packet lengths of 92 bytes.
//(9 10-byte data points plus a two-byte checksum
//The size of the transmit buffer will be an integral number of packet lengths,
//as given by A9_BT_TX_SEG_NUM

#define A9_BT_TX_SEG_SIZE (92)    //bytes in a single DMA transfer segment (10 * number of data points + 2 bytes for checksum)
#define A9_BT_TX_SEG_NUM (20)	  	//number of segments in buffer
                                  //A single 92-byte segment involves sending 900 bits over the serial port;
                                							
unsigned volatile char a9_bt_tx_buffer[A9_BT_TX_SEG_NUM][A9_BT_TX_SEG_SIZE];
unsigned volatile short int a9_bt_tx_index1 = 0;		//Points to starting address of data being written by software
unsigned volatile short int a9_bt_tx_index2 = 0;		//Points to starting address of DMA segment being transmitted

const unsigned short a9_bt_tx_max_send = 10;      //This sets an upper limit on the number of data points the data_send function
                                                  //will attempt to send during one call to it. It should be at equal to the baud rate in
                                                  //effect divided by 80 bits per data point divided by 1000 calls per second (or whatever),
                                                  //plus 20 to 50 percent, or the actual limit desired, whichever is lower. 15 is about right
                                                  //as an absolute maximum, if it's called once per mS. If once every 10 mS, go to 150, etc.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This function searches through the LabView data channels section of the io_data array (near the top of
// the array, only the version number and the total array length entries are higher). The float value read
// from the io_data array is cast to long unsigned int. If bit 16 is set, the data_id is added to the medium-speed
// list; if bit 17 is set, the data_id is added to the high-speed list. Slow speed is the default and no list is
// needed. Blank portions of the list are filled with 0xFFFF; this is not a valid data_id, and serves to indicate the
// end of the list.
// Run this function occasionally (e.g., every 100 mS to 1000 mS)

void mb_bt_create_data_lists(unsigned short first_id,         //First LabView channel data_id to search
                             unsigned short last_id,          //Last LabView channel data_id to search
                             unsigned short medium_list[], //Array to hold list; size = # channels + 1
                             unsigned short fast_list[], //Array to hold list; size = # channels + 1
                             float (* get_float_using_id)(short unsigned int data_id) //Pointer to function to get value of data_id
                             )
{
  unsigned short i = 0, j = 0, k = 0;
  unsigned long temp;
    
  // First, initialize data lists to 0xFFFF
  for (i=0; i < last_id - first_id + 2; ++i)
  {
    fast_list[i]    = 0xFFFF;
    medium_list[i]  = 0xFFFF;
  }
 
  // Second, search LabView channel data_id range for fast- and medium-speed data to add to lists
  for (i=first_id;i<=last_id;++i)
  {
    // Get the value at data_id; cast to long for bit operations
    temp = (unsigned long)(*get_float_using_id)(i);
    
    if (temp & (1<<16)) // medium-speed data transmission for this id
    {
      medium_list[j] = (unsigned short)temp;   //data_ids must be 16 bits
      ++j;                                    //move pointer to next spot in list
    }
    
    if (temp & (1<<17)) // fast data transmission for this id
    {
      fast_list[k] = (unsigned short)temp;   //data_ids must be 16 bits
      ++k;                                    //move pointer to next spot in list
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////
void a9_bt_data_sender
  (
  short unsigned max_data_id,       //Highest data_id to send at slow rate (0 is lowest)
  short unsigned subscriber_number, //Unique process identifier to identify new (unsent) data, range 0 - 31
  short unsigned medium_list[],     //Pointer to array (list) of data_ids to send at medium rate                                //
  short unsigned fast_list[]       //Pointer to array (list) of data_ids to send at fast rate
  )
{
  static unsigned short slow_done = 0, medium_done = 0, medium_index = 0, slow_index = 0, fast_index = 0;
  static signed short j = 0;
  static signed long free_space_integral = 0;
  static const signed long gain = 12;          // Actually the negative log 2 of the integral gain
  static const signed long max_data_send = 10;  // Maximum number of data points to send per function call
                                                  // regardless of bandwidth available
  unsigned short buffer_full; 
  signed short i, buffer_contents_size, temp_index1, temp_index2;
  signed long free_space;
    
  //Send all data points in the fast list, followed by one each from the medium and slow lists.
  //Repeat until transmit buffer is sufficiently full; as the buffer becomes more full the number
  //of writes to the buffer per call of this function is reduced, from a9_bt_tx_max_send down to 0.
  temp_index1 = a9_bt_tx_index1;    // Get copies of transmit buffer indexes, to avoid potential race conditions.
  temp_index2 = a9_bt_tx_index2;    // Note that the originals may change due to interrupt routines
  if (temp_index1 >= temp_index2)
  {
    buffer_contents_size = temp_index1 - temp_index2;
  }
  else 
  {
    buffer_contents_size = temp_index1 - temp_index2 + A9_BT_TX_SEG_NUM;
  }

  free_space =  A9_BT_TX_SEG_NUM - 2 - buffer_contents_size;

  // Integrate error; error = free_space - ~0.5 * buffer size
  // (Try to keep buffer at half full)
  free_space_integral = free_space_integral + free_space - (signed)(A9_BT_TX_SEG_NUM >> 1);

  //Limit integrator windup to maximum useful values
  if (free_space_integral > (max_data_send << gain))
  {
    free_space_integral = max_data_send << gain;
  }
  if (free_space_integral < -(max_data_send << gain))
  {
    free_space_integral = -(max_data_send << gain);
  }

  if (free_space_integral > 0)
  {
    i = free_space_integral >> gain;    // Fine-tune shift amounts for desired response 
  }
  else if (j <= 0)    // For free_space_integral <= 0, send out data only on calls when j == 0
  {
    j = - (free_space_integral >> gain);
    i = 1;
  }
  else
  {
    --j;
    i = 0;    // Send out no data when j != 0
 
  }

  if (i > max_data_send) {i = max_data_send;} // Limit top transmission speed even if bandwidth is available

  buffer_full = 1;    //End if no more data available, or room in buffer
 
  while (i > 0) //Send data frames while i (calculated above) is greater than zero
  {
    //Send data frames from the fast list until its end is reached
    //Only send fresh (newly updated) data frames
    if (fast_list[fast_index] != 0xFFFF)
    { 
      if (!mb_io_data_was_read(fast_list[fast_index], subscriber_number))
      {
        buffer_full = a9_bt_push_io_data_point(mb_io_get_pointer(fast_list[fast_index]));
        if (!buffer_full) 
        {
          mb_io_mark_as_read(fast_list[fast_index], subscriber_number); // Data has been sent to PC, mark accordingly
          ++fast_index;
        }
      }
      else
      {
        ++fast_index;    //old data, skip and move to next list element
      }
 
    }
    //Send one medium speed data point, if any, then set medium_done flag
    //Only send fresh (newly updated) data frames
    //Keep trying until one point is sent successfully, or the end of the list is reached (0xFFFF)
    else if ((medium_list[medium_index] != 0xFFFF) & (!medium_done))
    {
      if (!mb_io_data_was_read(medium_list[medium_index], subscriber_number))
      {
        buffer_full = a9_bt_push_io_data_point(mb_io_get_pointer(medium_list[medium_index]));
        if (!buffer_full)
        {
          medium_done = 1;
          mb_io_mark_as_read(medium_list[medium_index], subscriber_number); // Data has been sent to PC, mark accordingly
          ++medium_index;
        }
      }
      else
      {
        ++medium_index;   //old data, skip and move to next list element
      }
    }
    // Send one slow speed data point then set slow_done flag
    // Send even if no change. Why? 1) too much processor overhead to search through all data points for new ones
    // 2) Want to see all parameters, etc. at least once in a while on PC/laptop display and in log file.
    // The MCU has no way of knowing if its data is being received by anything externally.
    // Since I wanted to use the same callback function for new or old data, this call is with an invalid subscriber
    // number (0xFFFF > 31), which will cause data to be sent regardless of newness.
    else if (!slow_done)
    {
      buffer_full = a9_bt_push_io_data_point(mb_io_get_pointer(slow_index));
      if (!buffer_full)
      {
        slow_done = 1;
        mb_io_mark_as_read(slow_index, subscriber_number); // Data has been sent to PC, mark accordingly
        if (++slow_index > max_data_id) {slow_index = 0;}
      }
    }
    //Start new transmit cycle - reset fast index and flags
    else
    {      
      slow_done = 0;
      medium_done = 0;
      if (medium_list[medium_index] == 0xFFFF) {medium_index = 0;}
      fast_index = 0;      
    }
    i--;  // decrement i
  } 
}
/////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////
//GPDMA isr; for all non-USB and non-ethernet DMA. E.g., SSP, SPI, serial, SD card
void a9_gpdma_isr(void)
{  
  //BlueTooth serial transmit is on GPDMA channel 3
  //HS UART2 transmit terminal count interrupt - start new DMA transfer
  if (GPDMA_INT_TCSTAT & 1<<3)
  {
	  a9_bt_start_ch3_dma_transfer();
  }
  
  //clear GPDMA terminal count interrupts
  GPDMA_INT_TCCLR = 0xFF;
  
  //clear GPDMA error interrupts
  GPDMA_INT_ERR_CLR = 0xFF;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////

void a9_software_int_isr(void)
{
  if ((SW_INT >> 1) == (A9_SW_INT_DMA_CH3))
  {
    a9_bt_start_ch3_dma_transfer();
  }
  
  SW_INT = 0;  //Clear software interrupt
}

#ifdef BT_HSU2_BLUETOOTH
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void a9_bt_dma_receive(void)
{        
  static unsigned char dma_receive_buffer_1[1000], dma_receive_buffer_2[1000];
  static unsigned int buffer_1_active = 0, buffer_2_active = 0;
  short unsigned int temp_index;
  unsigned data_count_1, data_count_2, i = 0;
  
  if (!buffer_1_active)   //stop buffer_2 DMA transfer, start buffer_1 transfer, move buffer_2 data
  {
    if (GPDMA_ENABLED_CHNS & 1<<4)	//if DMA channel 4 active (BlueTooth serial receive)
    {     
      //set DMA transfer halt bits
      GPDMA_CH4_CFG = (GPDMA_CH4_CFG | (1<<18)) & ~((unsigned)0x1FFF<<19);  //set DMA CH4 halt bit
      while (GPDMA_CH4_CFG & (1<<17)) //wait until DMA active bit goes low
      {
        if (++i > 100)
        {
          //possible DMA error
          mb_error_occurred(ERROR_MB_SER_DMA_JAM);
          break;    
        }
      }
      GPDMA_CH4_CFG = (GPDMA_CH4_CFG & ~(1<<0)) & ~((unsigned)0x1FFF<<19); //disable CH4 DMA transfer
      
      data_count_2 = 1000 - GPDMA_CH4_CTRL & 0xFFF;
    }
    else if (((GPDMA_CH4_CTRL & 0xFFF) == 0) && buffer_2_active)  //End of buffer reached during receive (error)
    {
      //error: serial receive DMA buffer overflow
      data_count_2 = 1000;
      mb_error_occurred(ERROR_MB_SER_DMA_FLL);
      a9_dn_red_led_blink(50);
    }
    else    //DMA not active for some other reason
    {
      //Error - serial receive DMA not active (ok for first iteration)
      mb_error_occurred(ERROR_MB_SER_DMA_OFF);
      data_count_2 = 0;
    }
    
    //Restart CH4 DMA transfer to buffer_1
    GPDMA_CH4_SRC = (unsigned long)&HSU2_RX;	//source is high-speed UART2 receive
    GPDMA_CH4_DEST = (unsigned long)&dma_receive_buffer_1[0];	//destination is data array in memory
  
    //Set up GPDMA_CH4_CTRL. Do not OR bits with reserved bits!	Do not write ones to them, either.
 	  GPDMA_CH4_CTRL = 1000//bytes to receive, up to 4095.
    | 0<<12				//Source burst size of 1
    | 1<<15				//Destination burst size of 4
    //| 0<<18			 	//Source width (UART) = 8 bits
    //| 0<<21				//Destination width (memory) = 8 bits
    //| 1<<24				//source AHB bus master 1
    //| 1<<25				//destination AHB bus master 1
    //| 1<<26				//Increment the source address after each transfer
    | 1<<27			//Increment the destination address after each transfer
    //28,29,30 reserved
    //| ((unsigned long int)1<<31) 			//terminal count interrupt enable bit.
    ;
  
    //Set up GPDMA_CH4_CFG.	Do not OR bits with reserved bits!	Do not write ones to them, either.
    GPDMA_CH4_CFG = 1	//Enable CH4
    | (8<<1)			    //source peripheral is high-speed UART2 receive
    | (2<<11)			    //peripheral to memory, with DMA flow control
//    | (6<<11)			    //peripheral to memory, with peripheral flow control
    //| (1<<14)			  //Enable error interrupt
    //| (1<<15)			  //Enable terminal count interrupt
    //| (1<<16)			  //Enable locked transfers
    ;
    
    buffer_2_active = 0;  //DMA serial receive buffer 2 is ready to read  
    buffer_1_active = 1;  //buffer 1 is ready for DMA write
    
    //transfer data from DMA receive array to software buffer
    //for parsing in another function later
    //(kind of inefficient, but ok for low receive data traffic levels).
    temp_index = a9_bt_rx_index1;
    for (i = 0; i < data_count_2; ++i)
    { 
      a9_bt_rx_buffer[temp_index] = dma_receive_buffer_2[i];
  
      if (++temp_index == A9_BT_RX_BUFFER_SIZE) {temp_index = 0;}
      if (temp_index == a9_bt_rx_index2) 
      {
        //error - bluetooth software receive buffer full
        mb_error_occurred(ERROR_MB_SER_RX_FULL);
        a9_dn_red_led_blink(50);
        break;
      }
      else
      {
        a9_bt_rx_index1 = temp_index;   //update index1
      }  
    }
    
    
  }
  else                    //start DMA transfer to buffer_2, transfer buffer_1 data
  {
    if (GPDMA_ENABLED_CHNS & 1<<4)	//if DMA channel 4 active (BlueTooth serial receive)
    {
      //set DMA transfer halt bits
      GPDMA_CH4_CFG = (GPDMA_CH4_CFG | (1<<18)) & ~((unsigned)0x1FFF<<19);  //set DMA CH4 halt bit
      while (GPDMA_CH4_CFG & (1<<17)) //wait until DMA active bit goes low
      {
        if (++i > 100)
        {
          //possible DMA error
          mb_error_occurred(ERROR_MB_SER_DMA_JAM);
          break;    
        }
      }
      GPDMA_CH4_CFG = (GPDMA_CH4_CFG & ~(1<<0)) & ~((unsigned)0x1FFF<<19); //disable CH4 DMA transfer
      
      data_count_1 = 1000 - GPDMA_CH4_CTRL & 0xFFF;
    }
    else if (((GPDMA_CH4_CTRL & 0xFFF) == 0) && buffer_1_active)  //End of buffer reached during receive (error)
    {
      //error: serial receive DMA buffer overflow
      data_count_1 = 1000;
      a9_dn_red_led_blink(50);
      mb_error_occurred(ERROR_MB_SER_DMA_FLL);
    }
    else    //DMA not active for some other reason
    {
      //error: serial receive DMA not active (ok for first iteration)
      mb_error_occurred(ERROR_MB_SER_DMA_OFF);
      data_count_1 = 0;
    }
        
    //Restart CH4 DMA transfer to buffer_2
    GPDMA_CH4_SRC = (unsigned long)&HSU2_RX;	//source is high-speed UART2 receive
    GPDMA_CH4_DEST = (unsigned long)&dma_receive_buffer_2[0];	//destination is data array in memory
  
    //Set up GPDMA_CH4_CTRL. Do not OR bits with reserved bits!	Do not write ones to them, either.
 	  GPDMA_CH4_CTRL = 1000//bytes to receive, up to 4095.
    | 0<<12				//Source burst size of 1
    | 1<<15				//Destination burst size of 4
    //| 0<<18			 	//Source width (UART) = 8 bits
    //| 0<<21				//Destination width (memory) = 8 bits
    //| 1<<24				//source AHB bus master 1
    //| 1<<25				//destination AHB bus master 1
    //| 1<<26				//Increment the source address after each transfer
    | 1<<27			//Increment the destination address after each transfer
    //28,29,30 reserved
    //| ((unsigned long int)1<<31) 			//terminal count interrupt enable bit.
    ;
  
    //Set up GPDMA_CH4_CFG.	Do not OR bits with reserved bits!	Do not write ones to them, either.
    GPDMA_CH4_CFG = 1	//Enable CH4
    | (8<<1)			    //source peripheral is high-speed UART2 receive
    | (2<<11)			    //peripheral to memory, with DMA flow control
//    | (6<<11)			    //peripheral to memory, with peripheral flow control
    //| (1<<14)			  //Enable error interrupt
    //| (1<<15)			  //Enable terminal count interrupt
    //| (1<<16)			  //Enable locked transfers
    ;
    
    buffer_1_active = 0;    //buffer_1 data ready to read
    buffer_2_active = 1;    //buffer_2 ready for DMA write
    
    //transfer data from DMA receive array to software buffer
    //for parsing in another function later
    //(kind of inefficient, but ok for low receive data traffic levels).
    temp_index = a9_bt_rx_index1;
    for (i = 0; i < data_count_1; ++i)
    { 
      a9_bt_rx_buffer[temp_index] = dma_receive_buffer_1[i];
  
      if (++temp_index == A9_BT_RX_BUFFER_SIZE) {temp_index = 0;}
      if (temp_index == a9_bt_rx_index2) 
      {
        //error - bluetooth software receive buffer full
        mb_error_occurred(ERROR_MB_SER_RX_FULL);
        a9_dn_red_led_blink(50);
        break;
      }
      else
      {
        a9_bt_rx_index1 = temp_index;   //update index1
      }  
    }
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
#endif //BT_HSU2_BLUETOOTH


#ifdef BT_HSU1_SERIAL_PORT
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void a9_bt_dma_receive(void)
{        
  static unsigned char dma_receive_buffer_1[1000], dma_receive_buffer_2[1000];
  static unsigned int buffer_1_active = 0, buffer_2_active = 0;
  short unsigned int temp_index;
  unsigned data_count_1, data_count_2, i = 0;
  
  if (!buffer_1_active)   //stop buffer_2 DMA transfer, start buffer_1 transfer, move buffer_2 data
  {
    if (GPDMA_ENABLED_CHNS & 1<<4)	//if DMA channel 4 active (BlueTooth serial receive)
    {     
      //set DMA transfer halt bits
      GPDMA_CH4_CFG = (GPDMA_CH4_CFG | (1<<18)) & ~((unsigned)0x1FFF<<19);  //set DMA CH4 halt bit
      while (GPDMA_CH4_CFG & (1<<17)) //wait until DMA active bit goes low
      {
        if (++i > 100)
        {
          //possible DMA error
          mb_error_occurred(ERROR_MB_SER_DMA_JAM);
          break;    
        }
      }
      GPDMA_CH4_CFG = (GPDMA_CH4_CFG & ~(1<<0)) & ~((unsigned)0x1FFF<<19); //disable CH4 DMA transfer
      
      data_count_2 = 1000 - GPDMA_CH4_CTRL & 0xFFF;
    }
    else if (((GPDMA_CH4_CTRL & 0xFFF) == 0) && buffer_2_active)  //End of buffer reached during receive (error)
    {
      //error: serial receive DMA buffer overflow
      mb_error_occurred(ERROR_MB_SER_DMA_FLL);
      data_count_2 = 1000;
      a9_dn_red_led_blink(50);
    }
    else    //DMA not active for some other reason
    {
      //Error - serial receive DMA not active (ok for first iteration)
      mb_error_occurred(ERROR_MB_SER_DMA_OFF);
      data_count_2 = 0;
    }
    
    //Restart CH4 DMA transfer to buffer_1
    GPDMA_CH4_SRC = (unsigned long)&HSU1_RX;	  //source is high-speed UART1 receive
    GPDMA_CH4_DEST = (unsigned long)&dma_receive_buffer_1[0];	//destination is data array in memory
  
    //Set up GPDMA_CH4_CTRL. Do not OR bits with reserved bits!	Do not write ones to them, either.
 	  GPDMA_CH4_CTRL = 1000//bytes to receive, up to 4095.
    | 0<<12				//Source burst size of 1
    | 1<<15				//Destination burst size of 4
    //| 0<<18			 	//Source width (UART) = 8 bits
    //| 0<<21				//Destination width (memory) = 8 bits
    //| 1<<24				//source AHB bus master 1
    //| 1<<25				//destination AHB bus master 1
    //| 1<<26				//Increment the source address after each transfer
    | 1<<27			//Increment the destination address after each transfer
    //28,29,30 reserved
    //| ((unsigned long int)1<<31) 			//terminal count interrupt enable bit.
    ;
  
    //Set up GPDMA_CH4_CFG.	Do not OR bits with reserved bits!	Do not write ones to them, either.
    GPDMA_CH4_CFG = 1	//Enable CH4
    | (6<<1)			    //source peripheral is high-speed UART1 receive
    | (2<<11)			    //peripheral to memory, with DMA flow control
//    | (6<<11)			    //peripheral to memory, with peripheral flow control
    //| (1<<14)			  //Enable error interrupt
    //| (1<<15)			  //Enable terminal count interrupt
    //| (1<<16)			  //Enable locked transfers
    ;
    
    buffer_2_active = 0;  //DMA serial receive buffer 2 is ready to read  
    buffer_1_active = 1;  //buffer 1 is ready for DMA write
    
    //transfer data from DMA receive array to software buffer
    //for parsing in another function later
    //(kind of inefficient, but ok for low receive data traffic levels).
    temp_index = a9_bt_rx_index1;
    for (i = 0; i < data_count_2; ++i)
    { 
      a9_bt_rx_buffer[temp_index] = dma_receive_buffer_2[i];
  
      if (++temp_index == A9_BT_RX_BUFFER_SIZE) {temp_index = 0;}
      if (temp_index == a9_bt_rx_index2) 
      {
        //error - bluetooth software receive buffer full
        mb_error_occurred(ERROR_MB_SER_RX_FULL);
        a9_dn_red_led_blink(50);
        break;
      }
      else
      {
        a9_bt_rx_index1 = temp_index;   //update index1
      }  
    }
    
    
  }
  else                    //start DMA transfer to buffer_2, transfer buffer_1 data
  {
    if (GPDMA_ENABLED_CHNS & 1<<4)	//if DMA channel 4 active (BlueTooth serial receive)
    {
      //set DMA transfer halt bits
      GPDMA_CH4_CFG = (GPDMA_CH4_CFG | (1<<18)) & ~((unsigned)0x1FFF<<19);  //set DMA CH4 halt bit
      while (GPDMA_CH4_CFG & (1<<17))  //wait until DMA active bit goes low
      {
        if (++i > 100)
        {
          //possible DMA error
          mb_error_occurred(ERROR_MB_SER_DMA_JAM);
          break;    
        }
      }
      GPDMA_CH4_CFG = (GPDMA_CH4_CFG & ~(1<<0)) & ~((unsigned)0x1FFF<<19); //disable CH4 DMA transfer
      
      data_count_1 = 1000 - GPDMA_CH4_CTRL & 0xFFF;
    }
    else if (((GPDMA_CH4_CTRL & 0xFFF) == 0) && buffer_1_active)  //End of buffer reached during receive (error)
    {
      //error: serial receive DMA buffer overflow
      mb_error_occurred(ERROR_MB_SER_DMA_FLL);
      data_count_1 = 1000;
      a9_dn_red_led_blink(50);
    }
    else    //DMA not active for some other reason
    {
      //error: serial receive DMA not active (ok for first iteration)
      mb_error_occurred(ERROR_MB_SER_DMA_OFF);
      data_count_1 = 0;
    }
        
    //Restart CH4 DMA transfer to buffer_2
    GPDMA_CH4_SRC = (unsigned long)&HSU1_RX;	  //source is high-speed UART1 receive
    GPDMA_CH4_DEST = (unsigned long)&dma_receive_buffer_2[0];	//destination is data array in memory
  
    //Set up GPDMA_CH4_CTRL. Do not OR bits with reserved bits!	Do not write ones to them, either.
 	  GPDMA_CH4_CTRL = 1000//bytes to receive, up to 4095.
    | 0<<12				//Source burst size of 1
    | 1<<15				//Destination burst size of 4
    //| 0<<18			 	//Source width (UART) = 8 bits
    //| 0<<21				//Destination width (memory) = 8 bits
    //| 1<<24				//source AHB bus master 1
    //| 1<<25				//destination AHB bus master 1
    //| 1<<26				//Increment the source address after each transfer
    | 1<<27			//Increment the destination address after each transfer
    //28,29,30 reserved
    //| ((unsigned long int)1<<31) 			//terminal count interrupt enable bit.
    ;
  
    //Set up GPDMA_CH4_CFG.	Do not OR bits with reserved bits!	Do not write ones to them, either.
    GPDMA_CH4_CFG = 1	//Enable CH4
    | (6<<1)			    //source peripheral is high-speed UART1 receive
    | (2<<11)			    //peripheral to memory, with DMA flow control
//    | (6<<11)			    //peripheral to memory, with peripheral flow control
    //| (1<<14)			  //Enable error interrupt
    //| (1<<15)			  //Enable terminal count interrupt
    //| (1<<16)			  //Enable locked transfers
    ;
    
    buffer_1_active = 0;    //buffer_1 data ready to read
    buffer_2_active = 1;    //buffer_2 ready for DMA write
    
    //transfer data from DMA receive array to software buffer
    //for parsing in another function later
    //(kind of inefficient, but ok for low receive data traffic levels).
    temp_index = a9_bt_rx_index1;
    for (i = 0; i < data_count_1; ++i)
    { 
      a9_bt_rx_buffer[temp_index] = dma_receive_buffer_1[i];
  
      if (++temp_index == A9_BT_RX_BUFFER_SIZE) {temp_index = 0;}
      if (temp_index == a9_bt_rx_index2) 
      {
        //error - bluetooth software receive buffer full
        mb_error_occurred(ERROR_MB_SER_RX_FULL);
        a9_dn_red_led_blink(50);
        break;
      }
      else
      {
        a9_bt_rx_index1 = temp_index;   //update index1
      }  
    }
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
#endif //BT_HSU1_SERIAL_PORT

#ifdef BT_HSU2_BLUETOOTH
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void a9_bt_start_ch3_dma_transfer(void)
{        
  if (!(GPDMA_ENABLED_CHNS & (1<<3)))	//if DMA channel 3 inactive (BlueTooth serial transmit)
  {
    if (a9_bt_tx_index2 != a9_bt_tx_index1) //if transmit buffer is not empty
    {
      //start next DMA transmit segment
      GPDMA_CH3_SRC = (unsigned long)&a9_bt_tx_buffer[a9_bt_tx_index2][0];	//source is transmit buffer in memory
      GPDMA_CH3_DEST = (unsigned long)&HSU2_TX;									//destination is high-speed serial port 2
      
      //Set up GPDMA_CH3_CTRL. Do not OR bits with reserved bits!	Do not write ones to them, either.
 	    GPDMA_CH3_CTRL = A9_BT_TX_SEG_SIZE  //bytes to transmit
      | 1<<12				//Source burst size of 4  (**** Is this right?)
      | 4<<15				//Destination burst size of 32 bytes
      //| 0<<18			 	//Source width (HS UART 2) = 8 bits
      //| 0<<21				//Destination width (memory) = 8 bits
      //| 1<<24				//source AHB bus master 0
      //| 1<<25				//destination AHB bus master 0
      | 1<<26				//Increment the source address after each transfer
      //| 1<<27			//Increment the destination address after each transfer
      //28,29,30 reserved
      | ((unsigned long int)1<<31) 			//terminal count interrupt enable bit.
      ;
  
      //Set up GPDMA_CH3_CFG.	Do not OR bits with reserved bits!	Do not write ones to them, either.
      GPDMA_CH3_CFG = 1	//Enable CH3
      | (7<<6)			//Destination peripheral is HS UART2 transmit
      | (1<<11)			//memory to peripheral, with DMA flow control
      // 000 gives memory to memory transfer
      //| (1<<14)			//Enable error interrupt
      | (1<<15)			//Enable terminal count interrupt
      //| (1<<16)			//Enable locked transfers
      ;
  
      //DMA transfer setup is complete, move to new segment in buffer
      if (++a9_bt_tx_index2 == A9_BT_TX_SEG_NUM) {a9_bt_tx_index2 = 0;}
    }    
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
#endif //BT_HSU2_BLUETOOTH



#ifdef BT_HSU1_SERIAL_PORT

volatile unsigned long old_time = 0;
volatile unsigned short a9_bt_tx_write_index = 10; // **** TEST CODE **** set = to out-of-range value to start
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void a9_bt_start_ch3_dma_transfer(void)
{
  if (!(GPDMA_ENABLED_CHNS & (1<<3)))	//if DMA channel 3 inactive (BlueTooth serial transmit)
  {    
    if (a9_bt_tx_index2 != a9_bt_tx_index1) //if transmit buffer is not empty
    {
      //start next DMA transmit segment
      GPDMA_CH3_SRC = (unsigned long)&a9_bt_tx_buffer[a9_bt_tx_index2][0];	//source is transmit buffer in memory
      GPDMA_CH3_DEST = (unsigned long)&HSU1_TX;	 //destination is high-speed serial port 1
      
      //Set up GPDMA_CH3_CTRL. Do not OR bits with reserved bits!	Do not write ones to them, either.
 	    GPDMA_CH3_CTRL = A9_BT_TX_SEG_SIZE  //bytes to transmit
      | 1<<12				//Source burst size of 4  (**** Is this right?)
      | 4<<15				//Destination burst size of 32 bytes
      //| 0<<18			 	//Source width (HS UART 2) = 8 bits
      //| 0<<21				//Destination width (memory) = 8 bits
      //| 1<<24				//source AHB bus master 0
      //| 1<<25				//destination AHB bus master 0
      | 1<<26				//Increment the source address after each transfer
      //| 1<<27			//Increment the destination address after each transfer
      //28,29,30 reserved
      | ((unsigned long int)1<<31) 			//terminal count interrupt enable bit.
      ;
  
      //Set up GPDMA_CH3_CFG.	Do not OR bits with reserved bits!	Do not write ones to them, either.
      GPDMA_CH3_CFG = 1	//Enable CH3
      | (5<<6)			//Destination peripheral is HS UART1 transmit
      | (1<<11)			//memory to peripheral, with DMA flow control
      // 000 gives memory to memory transfer
      //| (1<<14)
      | (1<<15)			//Enable terminal count interrupt
      //| (1<<16)			//Enable locked transfers
      ;
      
      //DMA transfer setup is complete, move to new segment in buffer
      if (++a9_bt_tx_index2 == A9_BT_TX_SEG_NUM) {a9_bt_tx_index2 = 0;}
    }    
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
#endif //BT_HSU1_SERIAL_PORT

#ifdef BT_HSU2_BLUETOOTH
/////////////////////////////////////////////////////////////////////////////////////////////////
unsigned short int a9_bt_push_io_data_point(DATA_FRAME * data_point)
{
  static short unsigned int j = 0;
  short unsigned int checksum, i, data_id;
  short int tempindex1, tempindex2; 
  long unsigned int value, timestamp;
   
    if (j == 0)  //Start of new segment - is this segment available yet?
    {
      //Check that the DMA process is finished reading this segment before writing to it. 
      //index1 must stay at least two segments before index2 in the ring buffer,
      //since the DMA may be reading the segment immediately before index2.
      tempindex1 = a9_bt_tx_index1;
      tempindex2 = a9_bt_tx_index2;
      if (++tempindex1 >= A9_BT_TX_SEG_NUM){tempindex1 = 0;}
      if (--tempindex2 < 0){tempindex2 = A9_BT_TX_SEG_NUM - 1;}      
      if (tempindex1 == tempindex2) //DMA transmit buffer full
      {
      
        //Check for serial port stall - force serial port interrupt if necessary
        if ((GPDMA_ENABLED_CHNS & (1<<3)) && !(HSU2_IIR & (1<<0)) && ((HSU2_LEVEL & 0xFF00) == 0))
        { //DMA active, no tx interrupt pending and tx buffer level is at zero
          HSU2_IIR = 1<<6;    //Force transmit interrupt to start DMA transfer
        }
        return 1;
      }
    }
    
    data_id = data_point -> id;
    value = data_point -> payload.ulul.ul1;
    timestamp = data_point -> payload.ulul.ul2;
    
    a9_bt_tx_buffer[a9_bt_tx_index1][j++] = data_id >> 8;
    a9_bt_tx_buffer[a9_bt_tx_index1][j++] = data_id & 0xFF;
    a9_bt_tx_buffer[a9_bt_tx_index1][j++] = value >> 24;
    a9_bt_tx_buffer[a9_bt_tx_index1][j++] = (value >> 16) & 0xFF;
    a9_bt_tx_buffer[a9_bt_tx_index1][j++] = (value >> 8) & 0xFF;
    a9_bt_tx_buffer[a9_bt_tx_index1][j++] = value & 0xFF;     
    a9_bt_tx_buffer[a9_bt_tx_index1][j++] = timestamp >> 24;
    a9_bt_tx_buffer[a9_bt_tx_index1][j++] = (timestamp >> 16) & 0xFF;
    a9_bt_tx_buffer[a9_bt_tx_index1][j++] = (timestamp >> 8) & 0xFF;
    a9_bt_tx_buffer[a9_bt_tx_index1][j++] = timestamp & 0xFF;
    
    //End of packet? Calculate checksum and move to new segment.
    if (j == A9_BT_TX_SEG_SIZE - 2)
    {
      //generate checksum
      checksum = 1;   //Checksum = sum of bytes plus one
      for (i = 0; i < A9_BT_TX_SEG_SIZE - 2; ++i)
      {
        checksum += a9_bt_tx_buffer[a9_bt_tx_index1][i];
      }
      a9_bt_tx_buffer[a9_bt_tx_index1][A9_BT_TX_SEG_SIZE - 2] = checksum >> 8;
      a9_bt_tx_buffer[a9_bt_tx_index1][A9_BT_TX_SEG_SIZE - 1] = checksum & 0xFF;

      //segment is finished, advance index1
      //index1 must never leave the range 0 to SEG_NUM - 1
      //or index2 might sneak by it.
      if (a9_bt_tx_index1 >= A9_BT_TX_SEG_NUM - 1)
      {
        a9_bt_tx_index1 = 0;
      }
      else
      {
        ++a9_bt_tx_index1;
      }
      
      if (!(GPDMA_ENABLED_CHNS & (1<<3)))   //DMA channel not active
      {
        SW_INT = (A9_SW_INT_DMA_CH3 << 1) | 1;   //Set software interrupt to start data transfer if stopped.
      }
      j = 0;  //start new packet
    }
    return 0;
  }
/////////////////////////////////////////////////////////////////////////////////////////////////
#endif //BT_HSU2_BLUETOOTH

#ifdef BT_HSU1_SERIAL_PORT
/////////////////////////////////////////////////////////////////////////////////////////////////
unsigned short int a9_bt_push_io_data_point(DATA_FRAME * data_point)
{
  static short unsigned int j = 0;
  short unsigned int checksum, i, data_id;
  short int tempindex1, tempindex2; 
  long unsigned int value, timestamp;
   
    if (j == 0)  //Start of new segment - is this segment available yet?
    {
      //Check that the DMA process is finished reading this segment before writing to it. 
      //index1 must stay at least two segments before index2 in the ring buffer,
      //since the DMA may be reading the segment immediately before index2.
      tempindex1 = a9_bt_tx_index1;
      tempindex2 = a9_bt_tx_index2;
      if (++tempindex1 >= A9_BT_TX_SEG_NUM){tempindex1 = 0;}
      if (--tempindex2 < 0){tempindex2 = A9_BT_TX_SEG_NUM - 1;}      
      if (tempindex1 == tempindex2) //DMA transmit buffer full
      {
        //Check for serial port stall - force serial port interrupt if necessary
        if ((GPDMA_ENABLED_CHNS & (1<<3)) && !(HSU1_IIR & (1<<0)) && ((HSU1_LEVEL & 0xFF00) == 0))
        { //DMA active, no tx interrupt pending and tx buffer level is at zero
          HSU1_IIR = 1<<6;    //Force transmit interrupt to start DMA transfer
        }
        return 1;
      }
    }
    
    data_id = data_point -> id;
    value = data_point -> payload.ulul.ul1;
    timestamp = data_point -> payload.ulul.ul2;
    
    a9_bt_tx_buffer[a9_bt_tx_index1][j++] = data_id >> 8;
    a9_bt_tx_buffer[a9_bt_tx_index1][j++] = data_id & 0xFF;
    a9_bt_tx_buffer[a9_bt_tx_index1][j++] = value >> 24;
    a9_bt_tx_buffer[a9_bt_tx_index1][j++] = (value >> 16) & 0xFF;
    a9_bt_tx_buffer[a9_bt_tx_index1][j++] = (value >> 8) & 0xFF;
    a9_bt_tx_buffer[a9_bt_tx_index1][j++] = value & 0xFF;     
    a9_bt_tx_buffer[a9_bt_tx_index1][j++] = timestamp >> 24;
    a9_bt_tx_buffer[a9_bt_tx_index1][j++] = (timestamp >> 16) & 0xFF;
    a9_bt_tx_buffer[a9_bt_tx_index1][j++] = (timestamp >> 8) & 0xFF;
    a9_bt_tx_buffer[a9_bt_tx_index1][j++] = timestamp & 0xFF;
    
    //End of packet? Calculate checksum and move to new segment.
    if (j == A9_BT_TX_SEG_SIZE - 2)
    {
      //generate checksum
      checksum = 1;   //Checksum = sum of bytes plus one
      for (i = 0; i < A9_BT_TX_SEG_SIZE - 2; ++i)
      {
        checksum += a9_bt_tx_buffer[a9_bt_tx_index1][i];
      }
      a9_bt_tx_buffer[a9_bt_tx_index1][A9_BT_TX_SEG_SIZE - 2] = checksum >> 8;
      a9_bt_tx_buffer[a9_bt_tx_index1][A9_BT_TX_SEG_SIZE - 1] = checksum & 0xFF;

      //segment is finished, advance index1
      //index1 must never leave the range 0 to SEG_NUM - 1
      //or index2 might sneak by it.
      if (a9_bt_tx_index1 >= A9_BT_TX_SEG_NUM - 1)
      {
        a9_bt_tx_index1 = 0;
      }
      else
      {
        ++a9_bt_tx_index1;
      }
      
      if (!(GPDMA_ENABLED_CHNS & (1<<3)))   //DMA channel not active
      {
        SW_INT = (A9_SW_INT_DMA_CH3 << 1) | 1;   //Set software interrupt to start data transfer if stopped.
      }
      j = 0;  //start new packet
    }
    return 0;
  }
/////////////////////////////////////////////////////////////////////////////////////////////////
#endif //BT_HSU1_SERIAL_PORT

////////////////////////////////////////////////////////////////////////////////
// UART2 receive data parser. Pops all incoming parameter packets from the UART2 receive buffer,
// and puts the results in the mb_io_data array with time stamps.
void mb_bt_pop_receive_packets(void)
{
  unsigned short int cal_checksum, rec_checksum, temp_index, data_id;
  unsigned long int data;
		
// 8 bytes in BlueTooth receive packet:
//16-bit data ID, 32-bit parameter, 16-bit checksum

  //Only look for a packet to read if the buffer is not empty
  if (a9_bt_rx_index2 == a9_bt_rx_index1)
  {
    return;   //no data in buffer
  }

  //Parse all data in the BlueTooth receive buffer. To set a max packet number, use a for loop
  while (1)
  {
    //Calculate checksum and save potential incoming data
    temp_index = a9_bt_rx_index2;
    
		cal_checksum = a9_bt_rx_buffer[temp_index] + 1; //Start checksum - add one (checksum of zero should not be zero)
    data_id = a9_bt_rx_buffer[temp_index];	//First byte is (may be) MSB of ID;
    
    if (++temp_index == A9_BT_RX_BUFFER_SIZE) {temp_index = 0;}
    if (temp_index == a9_bt_rx_index1) {return;}  // Stop parsing packets when buffer is empty
    cal_checksum += a9_bt_rx_buffer[temp_index];
		data_id = (data_id << 8) | a9_bt_rx_buffer[temp_index]; //OR in the LSB of the ID;
		
    if (++temp_index == A9_BT_RX_BUFFER_SIZE) {temp_index = 0;}
    if (temp_index == a9_bt_rx_index1) {return;}
    cal_checksum += a9_bt_rx_buffer[temp_index];
    data = a9_bt_rx_buffer[temp_index];         // MSB of incoming 32-bit data
    
    if (++temp_index == A9_BT_RX_BUFFER_SIZE) {temp_index = 0;}
    if (temp_index == a9_bt_rx_index1) {return;}  // Stop parsing packets when buffer is empty
    cal_checksum += a9_bt_rx_buffer[temp_index];
    data = (data << 8) | a9_bt_rx_buffer[temp_index];         // OR in second byte of incoming 32-bit data
    
    if (++temp_index == A9_BT_RX_BUFFER_SIZE) {temp_index = 0;}
    if (temp_index == a9_bt_rx_index1) {return;}  // Stop parsing packets when buffer is empty
    cal_checksum += a9_bt_rx_buffer[temp_index];
    data = (data << 8) | a9_bt_rx_buffer[temp_index];         // OR in third byte of incoming 32-bit data
    
    if (++temp_index == A9_BT_RX_BUFFER_SIZE) {temp_index = 0;}
    if (temp_index == a9_bt_rx_index1) {return;}  // Stop parsing packets when buffer is empty
    cal_checksum += a9_bt_rx_buffer[temp_index];
    data = (data << 8) | a9_bt_rx_buffer[temp_index];         // OR in fourth (LSB) byte of incoming 32-bit data

    if (++temp_index == A9_BT_RX_BUFFER_SIZE) {temp_index = 0;}
    if (temp_index == a9_bt_rx_index1) {return;}  // Stop parsing packets when buffer is empty
    rec_checksum = a9_bt_rx_buffer[temp_index];         // MSB of incoming 16-bit checksum
    
    if (++temp_index == A9_BT_RX_BUFFER_SIZE) {temp_index = 0;}
    if (temp_index == a9_bt_rx_index1) {return;}  // Stop parsing packets when buffer is empty
    rec_checksum = (rec_checksum << 8) | a9_bt_rx_buffer[temp_index]; // OR in LSB of incoming 16-bit checksum
      	
		//Valid packet found? Put received values into appropriate location
		if (cal_checksum == rec_checksum)
		{	
      //Write received parameter and present time to mb_io_data
      mb_io_set_ul(data_id, data);    
      
      //Move index to start of next packet
      if (++temp_index == A9_BT_RX_BUFFER_SIZE) {temp_index = 0;}
      a9_bt_rx_index2 = temp_index;         //Update global index (frees space for new packets)
      if (a9_bt_rx_index2 == a9_bt_rx_index1) {return;}   // Stop parsing packets when buffer is empty
		}
		
		else	//not a valid packet - move ahead a byte in the buffer and try again.
		{
			//Error - BlueTooth checksum checksum received did not match
      mb_error_occurred(ERROR_MB_SER_CHKSUM);
      a9_dn_red_led_blink(50);
      
      //Move to next potential packet starting byte

      //This construction is needed to keep the index from ever going outside the bounds of the ring buffer,
      //even just briefly, to avoid race conditions. Mostly a problem if an isr is involved (later upgrade)
      if (a9_bt_rx_index2 == A9_BT_RX_BUFFER_SIZE - 1){a9_bt_rx_index2 = 0;} 
      else {++a9_bt_rx_index2;}
      if (a9_bt_rx_index2 == a9_bt_rx_index1) {return;} // Stop parsing packets when buffer is empty
		}	
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
