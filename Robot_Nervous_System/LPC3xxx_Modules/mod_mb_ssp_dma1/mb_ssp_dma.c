#include <mb_includes.h>

//Global variables

#define A9_DN_VAR_ARRAY_SIZE  2048

long unsigned int * a9_dn_var_ptrs[A9_DN_VAR_ARRAY_SIZE];
long unsigned int a9_dn_vars[A9_DN_VAR_ARRAY_SIZE];
long unsigned int a9_dn_timestamps[A9_DN_VAR_ARRAY_SIZE]; 

volatile long unsigned int a9_dn_seg_counter_rx = 0; //**** TEST CODE ****
volatile long unsigned int a9_dn_seg_counter_tx = 0;  //**** TEST CODE ****

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

////////////////////////////////////////////////////////////////
//Global variables for SSP/DMA software transmit buffer//////////
////////////////////////////////////////////////////////////////

#define A9_DN_SSP_TX_SEG_NUM  16

//list of addresses of data to be sent, terminated in 0xFFFF;
short unsigned int a9_dn_ssp_tx_list[A9_DN_VAR_ARRAY_SIZE + 1];

short unsigned int a9_dn_ssp_tx_buffer[A9_DN_SSP_TX_SEG_NUM][A9_DN_SSP_SEG_SIZE];
volatile short unsigned int a9_dn_ssp_tx_index1 = 0; //points to segment being written by software
volatile short unsigned int a9_dn_ssp_tx_index2 = 0; //points to segment being read by ssp dma

//when a9_dn_ssp_tx_index1 = a9_dn_ssp_tx_index2, buffer has no data available
//when a9_dn_ssp_tx_index2 is one before a9_dn_ssp_tx_index1, buffer is full

/////////////////////////////////////////////////////////////////////////////////////////////////
void a9_dn_ssp_init_tx_list(void)
{
  short unsigned int i;
  
  for (i = 0; i <= A9_DN_VAR_ARRAY_SIZE; ++i)
  {
    a9_dn_ssp_tx_list[i] = 0xFFFF;
  }
  
  //test code - add test values to array
  
  a9_dn_ssp_tx_list[0] = 19;
  a9_dn_ssp_tx_list[1] = 18;
  a9_dn_ssp_tx_list[2] = 17;
  a9_dn_ssp_tx_list[3] = 16;
  a9_dn_ssp_tx_list[4] = 15;
  a9_dn_ssp_tx_list[5] = 14;
  a9_dn_ssp_tx_list[6] = 13;
  a9_dn_ssp_tx_list[7] = 12;
  a9_dn_ssp_tx_list[8] = 11;
  a9_dn_ssp_tx_list[9] = 10;
  a9_dn_ssp_tx_list[10] = 9;
  a9_dn_ssp_tx_list[11] = 8;
  a9_dn_ssp_tx_list[12] = 7;
  a9_dn_ssp_tx_list[13] = 6;
  a9_dn_ssp_tx_list[14] = 5;
  a9_dn_ssp_tx_list[15] = 4;
  a9_dn_ssp_tx_list[16] = 3;
  a9_dn_ssp_tx_list[17] = 2;
  a9_dn_ssp_tx_list[18] = 1;
  a9_dn_ssp_tx_list[19] = 0;
  
  a9_dn_vars[0] = 0x00000000;
  a9_dn_vars[1] = 0x11111111;
  a9_dn_vars[2] = 0x22222222;
  a9_dn_vars[3] = 0x33333333;
  a9_dn_vars[4] = 0x44444444;
  a9_dn_vars[5] = 0x55555555;
  a9_dn_vars[6] = 0x66666666;
  a9_dn_vars[7] = 0x77777777;
  a9_dn_vars[8] = 0x88888888;
  a9_dn_vars[9] = 0x99999999;
  a9_dn_vars[10] = 0x10101010;
  a9_dn_vars[11] = 0x11111111;
  a9_dn_vars[12] = 0x12121212;
  a9_dn_vars[13] = 0x13131313;
  a9_dn_vars[14] = 0x14141414;
  a9_dn_vars[15] = 0x15151515;
  a9_dn_vars[16] = 0x16161616;
  a9_dn_vars[17] = 0x17171717;
  a9_dn_vars[18] = 0x18181818;
  a9_dn_vars[19] = 0x19191919;
  
  a9_dn_timestamps[0] = 0x00000000;
  a9_dn_timestamps[1] = 0x11111111;
  a9_dn_timestamps[2] = 0x22222222;
  a9_dn_timestamps[3] = 0x33333333;
  a9_dn_timestamps[4] = 0x44444444;
  a9_dn_timestamps[5] = 0x55555555;
  a9_dn_timestamps[6] = 0x66666666;
  a9_dn_timestamps[7] = 0x77777777;
  a9_dn_timestamps[8] = 0x88888888;
  a9_dn_timestamps[9] = 0x99999999;
  a9_dn_timestamps[10] = 0x10101010;
  a9_dn_timestamps[11] = 0x11111111;
  a9_dn_timestamps[12] = 0x12121212;
  a9_dn_timestamps[13] = 0x13131313;
  a9_dn_timestamps[14] = 0x14141414;
  a9_dn_timestamps[15] = 0x15151515;
  a9_dn_timestamps[16] = 0x16161616;
  a9_dn_timestamps[17] = 0x17171717;
  a9_dn_timestamps[18] = 0x18181818;
  a9_dn_timestamps[19] = 0x19191919;
  
}

/////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////
void a9_dn_ssp_send_data(void)
{
  short unsigned int checksum, address, i, j = 0, list_index = 0;
  short int tempindex1, tempindex2; 
  long unsigned int data, timestamp;
  
  while (a9_dn_ssp_tx_list[list_index] != 0xFFFF)
  {
    address = a9_dn_ssp_tx_list[list_index];
    data = a9_dn_vars[address];
    timestamp = a9_dn_timestamps[address];
    
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
      if (++tempindex1 == A9_DN_SSP_TX_SEG_NUM){tempindex1 = 0;}
      if (--tempindex2 < 0){tempindex2 = A9_DN_SSP_TX_SEG_NUM - 1;}      
      if (tempindex1 == tempindex2)
      {
        //error - DMA transmit buffer full
        P3_OUTP_SET = 1<<26;
        return;
      }
      a9_dn_ssp_tx_index1 = tempindex1;
      j = 0;  //start new packet
    }
    
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
    if (++tempindex1 == A9_DN_SSP_TX_SEG_NUM){tempindex1 = 0;}
    if (--tempindex2 < 0){tempindex2 = A9_DN_SSP_TX_SEG_NUM - 1;}      
    if (tempindex1 == tempindex2)
    {
      //error - DMA transmit buffer full
      P3_OUTP_SET = 1<<26;
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
  
  //T1TCR = (T1TCR | (1<<0)) & 3;		//TEST CODE - start timer 1
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
        //For valid non-rtr addresses, store data in vars array
        if ((a9_dn_ssp_rx_buffer[a9_dn_ssp_rx_index2][i] & 0xF800) == 0x7800)
        {
          a9_dn_vars[a9_dn_ssp_rx_buffer[a9_dn_ssp_rx_index2][i] & 0x7FF] =
          (a9_dn_ssp_rx_buffer[a9_dn_ssp_rx_index2][i + 1] << 16) | //MS 16 bits first
          a9_dn_ssp_rx_buffer[a9_dn_ssp_rx_index2][i + 2];
            
          a9_dn_timestamps[a9_dn_ssp_rx_buffer[a9_dn_ssp_rx_index2][i] & 0x7FF] =
          (a9_dn_ssp_rx_buffer[a9_dn_ssp_rx_index2][i + 3] << 16) | //MS 16 bits first
          a9_dn_ssp_rx_buffer[a9_dn_ssp_rx_index2][i + 4];
            
          P3_OUTP_SET = 1<<25;  // **** TEST CODE **** turn on green LED
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
      P3_OUTP_SET = 1<<26;  // **** TEST CODE **** turn on RED LED
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
//  T1TCR = (T1TCR & (~(1<<0))) & 3;	//TEST CODE - stop timer 1
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void a9_dn_ssel_dma_isr(void)
{
  short unsigned int i = 0, temp = 0, temp1, temp2;
  
  static short unsigned int dummy = 0, data_sent = 0;    //dummy variable for sending blank segments (all zero)
  
  //Falling edge of SSEL - start receive DMA transfer on CH0 (SSEL now low)
  if (!(P3_INP_STATE & (1<<14))) 
  {    
    //set DMA transfer halt bits
    GPDMA_CH2_CFG = (GPDMA_CH2_CFG | (1<<18)) & ~((unsigned)0x1FFF<<19);  //set DMA CH0 halt bit
    
    SIC2_APR = (SIC2_APR | (1<<4)) & SIC2_APR_RBMASK;   //Set bit 4 for rising-edge GPIO4 FIQ
    SIC2_RSR = 1<<4;  //Clear SSEL1 (GPIO4) edge interrupt
    while (GPDMA_CH2_CFG & (1<<17)) {++i;} //wait until DMA active bits go low
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
      P3_OUTP_SET = 1<<26;  // **** TEST CODE **** turn on red LED 
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
    
    temp = (GPDMA_CH1_CFG & (1<<0));
    //check if transfer is complete; halt if necessary
    if (GPDMA_CH1_CFG & (1<<0)) 
    {
      GPDMA_CH1_CFG = (GPDMA_CH1_CFG | (1<<18)) & ~((unsigned)0x1FFF<<19);  //set DMA CH1 halt bit
      temp1 = (GPDMA_CH1_CFG & (1<<18));
      for (i=0;i<10;++i)
      {
        temp2 = (GPDMA_CH1_CFG & (1<<17));
      }
      
      //wait until DMA active bit goes low
  //    while (GPDMA_CH1_CFG & (1<<17)) {;}
      GPDMA_CH1_CFG = (GPDMA_CH1_CFG & ~(1<<0)) & ~((unsigned)0x1FFF<<19); //disable CH1 DMA transfer 
      
      temp = (GPDMA_CH1_CFG & (1<<0));     
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
//  T1TCR = (T1TCR & (~(1<<0))) & 3;	//TEST CODE - stop timer 1
}
/////////////////////////////////////////////////////////////////////////////////////////////////

