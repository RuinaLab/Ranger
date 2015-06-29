#include <mb_includes.h>

//Global variables

////////////////////////////////////////////////////////////////
//Global variables for SSP/DMA software receive buffer//////////
////////////////////////////////////////////////////////////////

#define A9_DN_SSP_SEG_SIZE 31
#define A9_DN_SSP_RX_SEG_NUM  16

short unsigned int a9_dn_ssp_rx_buffer[A9_DN_SSP_RX_SEG_NUM][A9_DN_SSP_SEG_SIZE + 8]; //8 larger than seg size to allow buffer to empty if needed.
volatile short int a9_dn_ssp_rx_index1 = 0; //points to segment being written by ssp dma
volatile short int a9_dn_ssp_rx_index2 = 0; //points to segment being read by software

//when a9_dn_ssp_rx_index1 = a9_dn_ssp_rx_index2, buffer has no data available
//when a9_dn_ssp_rx_index2 is one before a9_dn_ssp_rx_index1, buffer is full

////////////////////////////////////////////////////////////////
// Global variables for LED blinking (actually GPIO-0 and GPIO-1)
////////////////////////////////////////////////////////////////
volatile short unsigned int a9_dn_red_led_time, a9_dn_green_led_time;

////////////////////////////////////////////////////////////////
//Global variables for SSP/DMA software transmit buffer//////////
////////////////////////////////////////////////////////////////

#define A9_DN_SSP_TX_SEG_NUM  16

//list of addresses of data to be sent via SSP-CAN router, terminated in 0xFFFF;
//short unsigned int a9_dn_ssp_tx_list[ID_LAST+2];
//short unsigned int a9_dn_ssp_tx_list[] = {
#include <init_txlist.h>  
//0xFFFF,
//0xFFFF
//}
//;
/*
//list of addresses of data to be received via SSP-CAN router, terminated in 0xFFFF;
short unsigned int a9_dn_ssp_rx_list[ID_LAST + 2] = 
{

};
*/

short unsigned int a9_dn_ssp_tx_buffer[A9_DN_SSP_TX_SEG_NUM][A9_DN_SSP_SEG_SIZE];
volatile short unsigned int a9_dn_ssp_tx_index1 = 0; //points to segment being written by software
volatile short unsigned int a9_dn_ssp_tx_index2 = 0; //points to segment being read by ssp dma

//when a9_dn_ssp_tx_index1 = a9_dn_ssp_tx_index2, buffer has no data available
//when a9_dn_ssp_tx_index2 is one before a9_dn_ssp_tx_index1, buffer is full

////////////////////////////////////////////////////////////////////////////////////
// ARM7 synchronization code
// You should run this in software setup immediately before enabling interrupts
// What it does: the ARM7 and ARM9 confirm bidirectional communications via SSP 
// (exchanging pairs of preset codes), then the
// ARM9 sends the ARM7 a "go" command.
////////////////////////////////////////////////////////////////////////////////////
void a9_ssp_synchronize_arm7(void)
{ 
  unsigned long i = 0;
  unsigned short temp = 0;
    
  while (temp != 0xFAAF) // look for first code from ARM7 while sending first ARM9 code
  {
    //LED blink code, slow
    ++i;
    if (i > 500000) {i = 0;}
    if (i > 250000) {A9_DN_GREEN_LED_ON;} // **** TEST CODE ****
    else {A9_DN_GREEN_LED_OFF;}
    
    if (SSP1SR & (1<<2)) //SSP1 receive FIFO not empty, read in code
    {
      temp = SSP1DR;
    }
    if (SSP1SR & (1<<0)) //SSP1 transmit FIFO empty, send out code
    {
      SSP1DR = 0x0550;
    }   
  }

  //First code received from ARM7 - it is sending. Is it receiving?
  //Look for second code from ARM7, sent in response to first ARM9 code above.
  while (temp != 0x0550) // look for second code from ARM7 and continue sending first ARM9 code
  {
    //LED blink code, fast
    ++i;
    if (i > 50000) {i = 0;}
    if (i > 25000) {A9_DN_GREEN_LED_ON;}
    else {A9_DN_GREEN_LED_OFF;}
    
    if (SSP1SR & (1<<2)) //SSP1 receive FIFO not empty, read in code
    {
      temp = SSP1DR;
    }
    if (SSP1SR & (1<<0)) //SSP1 transmit FIFO empty, send out code
    {
      SSP1DR = 0x0550;
    }   
  }
  
  // Both codes received from ARM7, confirming 2-way communication. Send start code.
  SSP1DR = 0xFAAF;  //Send first ready signal to ARM7

// Drain SSP receive buffer in preparation for normal operation  
  while (SSP1SR & (1<<4)) //Wait until SSP is no longer busy
  {
    ++temp;
  }
  
  while (SSP1SR & (1<<2)) //Receive FIFO is not empty
  {
    temp = SSP1DR;    //Empty receive buffer
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////


/*
////////////////////////////////////////////////////////////////////////////////////
// ARM7 synchronization code
// You should run this in software setup immediately before enabling interrupts
// What it does: the ARM7 and ARM9 confirm bidirectional communications via SSP 
// (exchanging pairs of preset codes), then the
// ARM9 sends the ARM7 a "go" command.
////////////////////////////////////////////////////////////////////////////////////
void a9_ssp_synchronize_arm7(void)
{ 
  short unsigned int i = 0, temp, code_received_1 = 0, code_received_2 = 0;
  
  A9_DN_GREEN_LED_OFF;   // **** TEST CODE ****
  
  while (!(code_received_1 && code_received_2))
  {
    ++i;
    if (i > 50000) {i = 0;}
    if (i > 25000) {A9_DN_GREEN_LED_ON;} // **** TEST CODE ****
    else {A9_DN_GREEN_LED_OFF;}
    
    if (SSP1SR & (1<<2)) //SSP1 receive FIFO not empty
    {
      temp = SSP1DR;
      if (temp == 0xAAAA)       //First ready signal from ARM7.
      {
        code_received_1 = 1;
      }
      else if (temp == 0x5555)  //Second ready signal from ARM7.
      {
        code_received_2 = 1;
      }
    }  
  }
  
  A9_DN_GREEN_LED_OFF;
  
  SSP1DR = 0x5555;  //Send first ready signal to ARM7
  SSP1DR = 0xAAAA;  //Send second ready signal to ARM7
  
  while (SSP1SR & (1<<4)) //Wait until SSP is no longer busy
  {
    ++temp;
  }
  
  while (SSP1SR & (1<<2)) //Receive FIFO is not empty
  {
    temp = SSP1DR;    //Empty receive buffer
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////
*/
 /*
/////////////////////////////////////////////////////////////////////////////////////////////////
// Note: TO DO category - this function and the associated rx_list will not be usefule until RTR is
// available for SSP.
void a9_dn_ssp_init_rx_list(void)
{
  short unsigned int i;
  
  //initialize to all 0xFFFF (forms termination character for list).
  //(0xFFFF is never a valid data variable address/CAN ID) 
  for (i = 0; i <= ID_LAST; ++i)
  {
    a9_dn_ssp_rx_list[i] = 0xFFFF;
  }
  
  a9_dn_ssp_rx_list[0] =  ID_MCH_ERROR;
  a9_dn_ssp_rx_list[1] =  ID_MCFO_ERROR;
  a9_dn_ssp_rx_list[2] =  ID_MCFI_ERROR;
  a9_dn_ssp_rx_list[3] =  ID_MCSO_ERROR;
  a9_dn_ssp_rx_list[4] =  ID_MCSI_ERROR;
  a9_dn_ssp_rx_list[5] =  ID_UI_ERROR;
  a9_dn_ssp_rx_list[6] =  ID_MCH_MOTOR_VELOCITY;
  a9_dn_ssp_rx_list[7] =  ID_MCH_MOTOR_CURRENT;
  a9_dn_ssp_rx_list[8] =  ID_MCH_MOTOR_POSITION;
  a9_dn_ssp_rx_list[9] =  ID_MCH_ANGLE;
  a9_dn_ssp_rx_list[10] = ID_MCH_BATT_POWER;
  a9_dn_ssp_rx_list[11] = ID_MCFO_MOTOR_VELOCITY;
  a9_dn_ssp_rx_list[12] = ID_MCFO_MOTOR_CURRENT;
  a9_dn_ssp_rx_list[13] = ID_MCFO_MOTOR_POSITION;
  a9_dn_ssp_rx_list[14] = ID_MCFO_RIGHT_ANKLE_ANGLE;
  a9_dn_ssp_rx_list[15] = ID_MCFO_RIGHT_LS;
  a9_dn_ssp_rx_list[16] = ID_MCFO_LEFT_LS;
  a9_dn_ssp_rx_list[17] = ID_MCFO_RIGHT_HS;
  a9_dn_ssp_rx_list[18] = ID_MCFO_LEFT_HS;
  a9_dn_ssp_rx_list[19] = ID_MCFO_BATT_POWER;

}

/////////////////////////////////////////////////////////////////////////////////////////////////
 
/////////////////////////////////////////////////////////////////////////////////////////////////
// TO DO: set up RTR capability for SSP (future work). This function needs an RTR push call at the end, etc.
// Presently it does nothing.
void a9_dn_ssp_rx_list_subscribe(void)
{
  short unsigned int i = 0;
  DATA_FRAME frame;
  
  // Send out an RTR (Remote Transmission Request) packet for each DATA_ID in the can-ssp rx list,
  // thus subscribing to the desired data with the CAN-SSP router.
  
  frame.rtr = 1;
  frame.dlc = 8;
  frame.payload.ll.ll = 0;
  
  while (a9_dn_ssp_rx_list[i] != 0xFFFF)
  {
    frame.id = i;
    
  }
  
  ///////////// **** TEST CODE **** TO DO
  // Needs function here to add packet to transmit buffer
  /////////////////////////////////////////////
  
}

/////////////////////////////////////////////////////////////////////////////////////////////////
 */

/////////////////////////////////////////////////////////////////////////////////////////////////
void a9_dn_ssp_init_tx_list(void)
{
//  short unsigned int i;
//  
//  //initialize to all 0xFFFF (forms termination character for list).
//  //(0xFFFF is never a valid data variable address/CAN ID) 
//  for (i = 0; i <= ID_LAST; ++i)
//  {
//    a9_dn_ssp_tx_list[i] = 0xFFFF;
//  }
//  a9_dn_ssp_tx_list[0] = ID_MCH_MOTOR_TARGET_CURRENT; 
//  a9_dn_ssp_tx_list[1] = ID_MCFO_MOTOR_TARGET_CURRENT;
//  a9_dn_ssp_tx_list[2] = ID_MCFI_MOTOR_TARGET_CURRENT;
//  a9_dn_ssp_tx_list[3] = ID_MCSO_MOTOR_TARGET_CURRENT;
//  a9_dn_ssp_tx_list[4] = ID_MCSI_MOTOR_TARGET_CURRENT;
//  a9_dn_ssp_tx_list[5] = ID_UI_SET_LCD_QUAD_1;
//  a9_dn_ssp_tx_list[6] = ID_UI_SET_LCD_QUAD_2;
//  a9_dn_ssp_tx_list[7] = ID_UI_SET_LCD_QUAD_3;
//  a9_dn_ssp_tx_list[8] = ID_UI_SET_LCD_QUAD_4;
//  a9_dn_ssp_tx_list[9] = ID_UI_SET_LED_1;
//  a9_dn_ssp_tx_list[10] = ID_UI_SET_LED_2;
//  a9_dn_ssp_tx_list[11] = ID_UI_SET_LED_3;
//  a9_dn_ssp_tx_list[12] = ID_UI_SET_LED_4;
//  a9_dn_ssp_tx_list[13] = ID_UI_SET_LED_5;
//  a9_dn_ssp_tx_list[14] = ID_UI_SET_LED_6;
//  a9_dn_ssp_tx_list[15] = ID_UI_SET_BUZZER_FREQ;
//  a9_dn_ssp_tx_list[16] = ID_UI_SET_BUZZER_AMPL;
//  a9_dn_ssp_tx_list[17] = ID_MB_STATUS;
//  a9_dn_ssp_tx_list[18] = ID_MCH_COMMAND_CURRENT;
//  a9_dn_ssp_tx_list[19] = ID_MCFO_COMMAND_CURRENT;
//  a9_dn_ssp_tx_list[20] = ID_MCFI_COMMAND_CURRENT;
//  a9_dn_ssp_tx_list[21] = ID_MCSO_COMMAND_CURRENT;
//  a9_dn_ssp_tx_list[22] = ID_MCSI_COMMAND_ANG;
//  a9_dn_ssp_tx_list[23] = ID_MCH_STIFFNESS;
//  a9_dn_ssp_tx_list[24] = ID_MCFO_STIFFNESS; 
//  a9_dn_ssp_tx_list[25] = ID_MCFI_STIFFNESS;
//  a9_dn_ssp_tx_list[26] = ID_MCSO_STIFFNESS; 
//  a9_dn_ssp_tx_list[27] = ID_MCSI_PROP_COEFF;
//  a9_dn_ssp_tx_list[28] = ID_MCH_DAMPNESS;
//  a9_dn_ssp_tx_list[29] = ID_MCFO_DAMPNESS;
//  a9_dn_ssp_tx_list[30] = ID_MCFI_DAMPNESS;
//  a9_dn_ssp_tx_list[31] = ID_MCSO_DAMPNESS;
//  a9_dn_ssp_tx_list[32] = ID_MCSI_INT_COEFF;
//  a9_dn_ssp_tx_list[33] = ID_MCS
 
  
 // a9_dn_ssp_tx_list[33] = ;
 // a9_dn_ssp_tx_list[34] = ;
 // a9_dn_ssp_tx_list[35] = ;
 // a9_dn_ssp_tx_list[36] = ;
 // a9_dn_ssp_tx_list[37] = ;
 // a9_dn_ssp_tx_list[38] = ;    

}

/////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////
void a9_dn_red_led_blink(short unsigned int time)
{
  a9_dn_red_led_time = time;
}

void a9_dn_green_led_blink(short unsigned int time)
{
  a9_dn_green_led_time = time;
}

void a9_dn_update_leds(void)
{
  if (a9_dn_red_led_time > 0)
  {
    --a9_dn_red_led_time;
    A9_DN_RED_LED_ON;
  }
  else
  {
    A9_DN_RED_LED_OFF;
  }
  
  if (a9_dn_green_led_time > 0)
  {
    --a9_dn_green_led_time;
    A9_DN_GREEN_LED_ON;
  }
  else
  {
    A9_DN_GREEN_LED_OFF;
  }    
}
/////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////
void a9_dn_ssp_send_data(void)
{
  short unsigned int checksum, address, i, j = 0, data_id, list_index = 0;
  short int tempindex1, tempindex2; 
  long unsigned int data, timestamp;
  DATA_FRAME * data_pointer;
  
  while (a9_dn_ssp_tx_list[list_index] != 0xFFFF)
  {
    data_id = a9_dn_ssp_tx_list[list_index];

    if (mb_io_data_was_read(data_id, SSP)) // Data was already sent by SSP since it was last written
    {
      ++list_index; // Don't send data that hasn't been updated, go to next data in list
      continue;   
    }
    data_pointer = mb_io_get_pointer(data_id); 
    data = data_pointer -> payload.ulul.ul1;
    timestamp = data_pointer -> payload.ulul.ul2;
    address = data_pointer -> id;
    
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // TO DO - replace local variables here with pointer dereference, for speed
    // Or is this faster? TBD
    
    a9_dn_ssp_tx_buffer[a9_dn_ssp_tx_index1][j++] = address | (0xF << 11);
    a9_dn_ssp_tx_buffer[a9_dn_ssp_tx_index1][j++] = data >> 16;
    a9_dn_ssp_tx_buffer[a9_dn_ssp_tx_index1][j++] = data & 0xFFFF; 
    a9_dn_ssp_tx_buffer[a9_dn_ssp_tx_index1][j++] = timestamp >> 16;
    a9_dn_ssp_tx_buffer[a9_dn_ssp_tx_index1][j++] = timestamp & 0xFFFF;
    
    //End of packet?
    if (j == A9_DN_SSP_SEG_SIZE - 1)
    {
      //generate checksum
      checksum = 1;
      for (i = 0; i < A9_DN_SSP_SEG_SIZE - 1; ++i)
      {
        checksum += a9_dn_ssp_tx_buffer[a9_dn_ssp_tx_index1][i];
      }
      a9_dn_ssp_tx_buffer[a9_dn_ssp_tx_index1][j] = checksum;
      
      //go to next available segment in transmit buffer
      //index1 must stay at least two segments before index2 in the ring buffer,
      //since the DMA may be transmitting the segment immediately before index2.
      tempindex1 = a9_dn_ssp_tx_index1;
      tempindex2 = a9_dn_ssp_tx_index2;
      if (++tempindex1 >= A9_DN_SSP_TX_SEG_NUM){tempindex1 = 0;}
      if (--tempindex2 < 0){tempindex2 = A9_DN_SSP_TX_SEG_NUM - 1;}      
      if (tempindex1 == tempindex2)
      {
        //error - DMA transmit buffer full
        mb_error_occurred(ERROR_MB_SSP_TX_FULL);
        a9_dn_red_led_blink(50);
        return;
      }
      a9_dn_ssp_tx_index1 = tempindex1;
      j = 0;  //start new packet
    }

    //Mark data_id as read (sent); don't send again until rewritten
    mb_io_mark_as_read(a9_dn_ssp_tx_list[list_index], SSP);    

    //update address list index
    ++list_index;
  }
  
  if (j > 0)  //Segment not finished
  {
    //fill unused spots in last segment with zeroes
    for (; j < A9_DN_SSP_SEG_SIZE - 1; ++j)
    {
      a9_dn_ssp_tx_buffer[a9_dn_ssp_tx_index1][j] = 0;
    }
  
    //generate checksum
    checksum = 1;
    for (i = 0; i < A9_DN_SSP_SEG_SIZE - 1; ++i)
    {
      checksum += a9_dn_ssp_tx_buffer[a9_dn_ssp_tx_index1][i];
    }
    a9_dn_ssp_tx_buffer[a9_dn_ssp_tx_index1][j] = checksum;
      
    //go to next available segment in transmit buffer
    //index1 must stay at least two segments before index2 in the ring buffer,
    //since the DMA may be transmitting the segment immediately before index2.
    tempindex1 = a9_dn_ssp_tx_index1;
    tempindex2 = a9_dn_ssp_tx_index2;
    if (++tempindex1 >= A9_DN_SSP_TX_SEG_NUM){tempindex1 = 0;}
    if (--tempindex2 < 0){tempindex2 = A9_DN_SSP_TX_SEG_NUM - 1;}      
    if (tempindex1 == tempindex2)
    {
      //error - DMA transmit buffer full
      a9_dn_red_led_blink(50);
      mb_error_occurred(ERROR_MB_SSP_TX_FULL);
      return;
    }
    a9_dn_ssp_tx_index1 = tempindex1;
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////
void a9_dn_ssp_parse(void)
{
  unsigned short int i, cal_checksum;
  DATA_FRAME * data_pointer;
  unsigned short data_id;
  
  while (a9_dn_ssp_rx_index2 != a9_dn_ssp_rx_index1)  //rx buffer not empty
  {
    //Calculate checksum (don't include received checksum in total)
    cal_checksum = 1; //Add one to get non-zero checksum for zero data
    for (i = 0; i < A9_DN_SSP_SEG_SIZE - 1; ++i)
    {
      cal_checksum += a9_dn_ssp_rx_buffer[a9_dn_ssp_rx_index2][i];
    }
      
    //Do checksums match?
    if (cal_checksum == a9_dn_ssp_rx_buffer[a9_dn_ssp_rx_index2][A9_DN_SSP_SEG_SIZE - 1])
    {
      for (i = 0; i < A9_DN_SSP_SEG_SIZE - 5; i+=5)
      {
        //For valid non-rtr addresses, store data in io_data array
        if ((a9_dn_ssp_rx_buffer[a9_dn_ssp_rx_index2][i] & 0xF800) == 0x7800)
        {
 //         a9_dn_vars[a9_dn_ssp_rx_buffer[a9_dn_ssp_rx_index2][i] & 0x7FF] =
          data_id = a9_dn_ssp_rx_buffer[a9_dn_ssp_rx_index2][i] & 0x7FF;
          data_pointer = mb_io_get_pointer(data_id);
          if (data_pointer != NULL)
          {
            data_pointer -> payload.ulul.ul1 = 
            (a9_dn_ssp_rx_buffer[a9_dn_ssp_rx_index2][i + 1] << 16) | //MS 16 bits first
            a9_dn_ssp_rx_buffer[a9_dn_ssp_rx_index2][i + 2];
            
            data_pointer -> payload.ulul.ul2 =
            (a9_dn_ssp_rx_buffer[a9_dn_ssp_rx_index2][i + 3] << 16) | //MS 16 bits first
            a9_dn_ssp_rx_buffer[a9_dn_ssp_rx_index2][i + 4];

            mb_io_mark_as_unread_by_all(data_id);
            
            a9_dn_green_led_blink(50);
          }
          else
          {
            mb_error_occurred(ERROR_MB_SSP_ID_OOR); //Received invalid DATA_ID from SSP bus
          }
        }
        else if ((a9_dn_ssp_rx_buffer[a9_dn_ssp_rx_index2][i] & 0xF800) == 0xF800)
        {
           //Add rtr processing here
        }
        else
        {
          break;  //Stop parsing segment when a zero address is reached; go to new segment
        }
      }
    }
    else
    {
      mb_error_occurred(ERROR_MB_SSP_CHKSUM);
      a9_dn_red_led_blink(50);
      //error - bad SSP checksum
    } 
 
    //Segment read finished, move index2 to next unread segment
    //(check for index2 != index1 assumed to be in top-level while loop)
    if (a9_dn_ssp_rx_index2 == A9_DN_SSP_RX_SEG_NUM - 1)
    {
      a9_dn_ssp_rx_index2 = 0;
    }
    else
    {
      ++a9_dn_ssp_rx_index2;
    }
  } 
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void a9_dn_ssel_dma_isr(void)
{ 
  unsigned short i = 0;
  
  const unsigned short dummy = 0;    //dummy variable for sending blank segments (all zero)
  
  #ifdef DEBUG
    if (P3_INP_STATE & 1<<1)
    {
      MIC_ER = 0;
      return;
    }
  #endif
  
  //Falling edge of SSEL - start receive DMA transfer on CH0 (SSEL now low)
  if (!(P3_INP_STATE & (1<<14))) 
  {    
    //set DMA transfer halt bits
    GPDMA_CH2_CFG = (GPDMA_CH2_CFG | (1<<18)) & ~((unsigned)0x1FFF<<19);  //set DMA CH0 halt bit
    
    SIC2_APR = (SIC2_APR | (1<<4)) & SIC2_APR_RBMASK;   //Set bit 4 for rising-edge GPIO4 FIQ
    SIC2_RSR = 1<<4;  //Clear SSEL1 (GPIO4) edge interrupt
    while (GPDMA_CH2_CFG & (1<<17))  //wait until DMA active bits go low
    {
      if (++i > 100)
      {
        mb_error_occurred_fiq(ERROR_MB_SSP_DMA_JAM);
        break;    
      }
    }
    GPDMA_CH2_CFG = (GPDMA_CH2_CFG & ~(1<<0)) & ~((unsigned)0x1FFF<<19); //disable CH0 DMA transfer
    
    //DMA transfer complete, move to next data segment. (First segment is junk)
    //(FIQ-only increment algorithm)
    if (++a9_dn_ssp_rx_index1 == A9_DN_SSP_RX_SEG_NUM) {a9_dn_ssp_rx_index1 = 0;}
    
    //Check for buffer overflow
    if (a9_dn_ssp_rx_index1 == a9_dn_ssp_rx_index2)
    {
      //Overflow? Move back to previous segment
      if (--a9_dn_ssp_rx_index1 < 0) {a9_dn_ssp_rx_index1 = A9_DN_SSP_RX_SEG_NUM - 1;}
      
      //Error - buffer overflow
      mb_error_occurred_fiq(ERROR_MB_SSP_RX_FULL);
      a9_dn_red_led_blink(50);
    }

    //Start new DMA CH0 receive transfer
    GPDMA_CH2_SRC = (unsigned long)&SSP1DR;				//source is ssp1
    GPDMA_CH2_DEST = (unsigned long)&a9_dn_ssp_rx_buffer[a9_dn_ssp_rx_index1][0];	//destination is data array in memory
  
    //Set up GPDMA_CH0_CTRL. Do not OR bits with reserved bits!	Do not write ones to them, either.
 	  GPDMA_CH2_CTRL = A9_DN_SSP_SEG_SIZE + 8//short ints to receive, up to 4095. Extra 8 to drain buffer if needed.
    | 1<<12				//Source burst size of 4
    | 1<<15				//Destination burst size of 4
    | 1<<18			 	//Source width (SSP) = 16 bits
    | 1<<21				//Destination width (memory) = 16 bits
    //| 1<<24				//source AHB bus master 1
    //| 1<<25				//destination AHB bus master 1
    //| 1<<26				//Increment the source address after each transfer
    | 1<<27			//Increment the destination address after each transfer
    //28,29,30 reserved
    //| ((unsigned long int)1<<31) 			//terminal count interrupt enable bit.
    ;
  
    //Set up GPDMA_CH0_CFG.	Do not OR bits with reserved bits!	Do not write ones to them, either.
    GPDMA_CH2_CFG = 1	//Enable CH0
    | (3<<1)			    //source peripheral is SSP1 receive
    | (2<<11)			  //peripheral to memory, with DMA flow control
    //| (6<<11)			    //peripheral to memory, with peripheral flow control
    // 000 gives memory to memory transfer
    //| (1<<14)			  //Enable error interrupt
    //| (1<<15)			  //Enable terminal count interrupt
    //| (1<<16)			  //Enable locked transfers
    ;
  }
  
  //Rising edge of SSEL - start transmit DMA transfer on CH1 (SSEL now high)
  else  //P3_INP_STATE & (1<<14) 
  {   
    /////////////////////////////////////////////////////////////////
    //ARM7-synchronized clock tick for scheduler and elapsed mS timer
    /////////////////////////////////////////////////////////////////
//    a9_clock_tick();    
    
    //check if transfer is complete; halt if necessary
    if (GPDMA_CH1_CFG & (1<<0)) 
    {
      GPDMA_CH1_CFG = (GPDMA_CH1_CFG | (1<<18)) & ~((unsigned)0x1FFF<<19);  //set DMA CH1 halt bit
      while (GPDMA_CH1_CFG & (1<<17)) // Wait for DMA active bit to go low
      {
        if (++i > 100)
        {
          mb_error_occurred_fiq(ERROR_MB_SSP_DMA_JAM);
          //possible DMA error
          break;    
        }
      }
      GPDMA_CH1_CFG = (GPDMA_CH1_CFG & ~(1<<0)) & ~((unsigned)0x1FFF<<19); //disable CH1 DMA transfer     
    }
    
    SIC2_APR = (SIC2_APR & ~(1<<4)) & SIC2_APR_RBMASK;   //Set bit 4 for falling-edge GPIO4 FIQ
    SIC2_RSR = 1<<4;  //Clear SSEL1 (GPIO4) edge interrupt
       
    //start next DMA transmit segment
    if (a9_dn_ssp_tx_index2 == a9_dn_ssp_tx_index1) //transmit all-zero segment
    {
      GPDMA_CH1_SRC = (unsigned long)&dummy;  //source is dummy variable set to zero
      GPDMA_CH1_DEST = (unsigned long)&SSP1DR;		  //destination is ssp1
      //Set up GPDMA_CH1_CTRL. Do not OR bits with reserved bits!	Do not write ones to them, either.
      //Don't increment either the source or destination when sending zeroes.
 	    GPDMA_CH1_CTRL = A9_DN_SSP_SEG_SIZE  //short ints to transmit, up to 4095
      | 1<<12				//Source burst size of 4
      | 1<<15				//Destination burst size of 4
      | 1<<18			 	//Source width (SSP) = 16 bits
      | 1<<21				//Destination width (memory) = 16 bits
 //   | 1<<24				//source AHB bus master 1
 //   | 1<<25				//destination AHB bus master 1
  //  | 1<<26				//Increment the source address after each transfer
    //| 1<<27			//Increment the destination address after each transfer
    //28,29,30 reserved
    //| ((unsigned long int)1<<31) 			//terminal count interrupt enable bit.
    ;
      
    //Set up GPDMA_CH1_CFG.	Do not OR bits with reserved bits!	Do not write ones to them, either.
    GPDMA_CH1_CFG = 1	//Enable CH1
    | (11<<6)			//Destination peripheral is SSP1 transmit
   // | (5<<11)			//memory to peripheral, with peripheral flow control
    | (1<<11)			//memory to peripheral, with dma flow control
    // 000 gives memory to memory transfer
    //| (1<<14)			//Enable error interrupt
    //| (1<<15)			//Enable terminal count interrupt
    //| (1<<16)			//Enable locked transfers
    ;
    }
    else  //Initiate transfer of next data segment
    {
      GPDMA_CH1_SRC = (unsigned long)&a9_dn_ssp_tx_buffer[a9_dn_ssp_tx_index2][0];  //source is data array in memory
      GPDMA_CH1_DEST = (unsigned long)&SSP1DR;		  //destination is ssp1
      //Set up GPDMA_CH1_CTRL. Do not OR bits with reserved bits!	Do not write ones to them, either.
 	    GPDMA_CH1_CTRL = A9_DN_SSP_SEG_SIZE  //short ints to transmit, up to 4095
      | 1<<12				//Source burst size of 4
      | 1<<15				//Destination burst size of 4
      | 1<<18			 	//Source width (SSP) = 16 bits
      | 1<<21				//Destination width (memory) = 16 bits
 //   | 1<<24				//source AHB bus master 1
 //   | 1<<25				//destination AHB bus master 1
      | 1<<26				//Increment the source address after each transfer
    //| 1<<27			//Increment the destination address after each transfer
      //28,29,30 reserved
     //| ((unsigned long int)1<<31) 			//terminal count interrupt enable bit.
     ;
     
    //Set up GPDMA_CH1_CFG.	Do not OR bits with reserved bits!	Do not write ones to them, either.
    GPDMA_CH1_CFG = 1	//Enable CH1
    | (11<<6)			//Destination peripheral is SSP1 transmit
   // | (5<<11)			//memory to peripheral, with peripheral flow control
    | (1<<11)			//memory to peripheral, with dma flow control
    // 000 gives memory to memory transfer
    //| (1<<14)			//Enable error interrupt
    //| (1<<15)			//Enable terminal count interrupt
    //| (1<<16)			//Enable locked transfers
    ;
     
     //Segment transmission is complete, move to new segment
      if (++a9_dn_ssp_tx_index2 == A9_DN_SSP_TX_SEG_NUM) {a9_dn_ssp_tx_index2 = 0;}      
    }
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////

