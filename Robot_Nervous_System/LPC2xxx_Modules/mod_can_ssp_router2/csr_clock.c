#include <includes.h>

unsigned long int csr_ms_counter = 0;

// CAN transmit frame count global variables:
extern volatile unsigned short can_tx_frame_count_1;
extern volatile unsigned short can_tx_frame_count_2;
extern volatile unsigned short can_tx_frame_count_3;
extern volatile unsigned short can_tx_frame_count_4;

////////////////////////////////////////////////////////////////////////////////////
// ARM7 synchronization code
// Run this in software setup immediately before enabling interrupts
////////////////////////////////////////////////////////////////////////////////////
void csr_synchronize_arm9(void)
{ 
  short unsigned int i = 0, temp = 0;
    
  while (temp != 0x0550)  // Look for first code from ARM9, while sending out first ARM7 code
  {
    //LED blink code, slow
    ++i;
    if (i > 100000) {i = 0;}
    if (i > 50000) {MCU_LED_RED_ON;}
    else {MCU_LED_RED_OFF;}
    //Update display of "green ARM9 LED" on local MCU green LED (Hardware to do: add real ARM9 LED to board)
    if (FIO0PIN & 1<<15)	   //LPC3250 GPIO_0
  	{
	    MCU_LED_GREEN_ON; 	 	//turn on green ARM7 MCU LED
	  }
	  else
	  {
      MCU_LED_GREEN_OFF;    //turn off green ARM7 MCU LED
	  }
    
    if (!(SSPSR & (1<<4))) //SSP not busy
    {
      SSPDR = 0xFAAF;  //Send first ready signal to ARM9 
    }
    
    while (SSPSR & (1<<4)) //Wait until SSP is no longer busy
    {
      ++temp;
    }
       
    while (SSPSR & (1<<2)) //SSP receive FIFO not empty
    {
      temp = SSPDR;
    }  
  }
  
  while (temp != 0xFAAF)  // Look for second code from ARM9, while sending out second ARM7 code
  {
    //LED blink code, fast
    ++i;
    if (i > 20000) {i = 0;}
    if (i > 10000) {MCU_LED_RED_ON;}
    else {MCU_LED_RED_OFF;}
    //Update display of "green ARM9 LED" on local MCU green LED (Hardware to do: add real ARM9 LED to board)
    if (FIO0PIN & 1<<15)	   //LPC3250 GPIO_0
  	{
	    MCU_LED_GREEN_ON; 	 	//turn on green ARM7 MCU LED
	  }
	  else
	  {
      MCU_LED_GREEN_OFF;    //turn off green ARM7 MCU LED
	  }
    
    if (!(SSPSR & (1<<4))) //SSP not busy
    {
      SSPDR = 0x0550;  //Send first ready signal to ARM9 
    }
    
    while (SSPSR & (1<<4)) //Wait until SSP is no longer busy
    {
      ++temp;
    }
       
    while (SSPSR & (1<<2)) //SSP receive FIFO not empty
    {
      temp = SSPDR;
    }  
  }
  
  while (SSPSR & (1<<4)) //Wait until SSP is no longer busy
  {
    ++temp;
  }
  
  while (SSPSR & (1<<2)) //Receive FIFO is not empty
  {
    temp = SSPDR;    //Empty receive buffer
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////


/*
////////////////////////////////////////////////////////////////////////////////////
// ARM7 synchronization code
// Run this in software setup immediately before enabling interrupts
////////////////////////////////////////////////////////////////////////////////////
void csr_synchronize_arm9(void)
{ 
  short unsigned int i = 0, temp, code_received_1 = 0, code_received_2 = 0;
  
  MCU_LED_RED_OFF;  // **** TEST CODE ****
    
  while (!(code_received_1 && code_received_2))
  {
    ++i;
    if (i > 30000) {i = 0;}
    if (i > 15000) {MCU_LED_RED_ON;} // **** TEST CODE ****
    else {MCU_LED_RED_OFF;}
    if (FIO0PIN & 1<<15)	   //LPC3250 GPIO_0
  	{
	    MCU_LED_GREEN_ON; 	 	//turn on green ARM7 MCU LED
	  }
	  else
	  {
      MCU_LED_GREEN_OFF;    //turn off green ARM7 MCU LED
	  }
    
    if (!(SSPSR & (1<<4))) //SSP not busy
    {
      SSPDR = 0xAAAA;  //Send first ready signal to ARM9 
      SSPDR = 0x5555;  //Send second ready signal to ARM9  
    }
    
    while (SSPSR & (1<<4)) //Wait until SSP is no longer busy
    {
      ++temp;
    }
       
    while (SSPSR & (1<<2)) //SSP receive FIFO not empty
    {
      temp = SSPDR;
      if (temp == 0x5555)       //First ready signal from ARM9.
      {
        code_received_1 = 1;
      }
      else if (temp == 0xAAAA)  //Second ready signal from ARM9.
      {
        code_received_2 = 1;
      }
    }  
  }
  
  MCU_LED_RED_OFF; // **** TEST CODE ****
  if (FIO0PIN & 1<<15)	   //LPC3250 GPIO_0
  {
	  MCU_LED_GREEN_ON; 	 	//turn on green ARM7 MCU LED
	}
	else
	{
   MCU_LED_GREEN_OFF;    //turn off green ARM7 MCU LED
	}
   
  while (SSPSR & (1<<4)) //Wait until SSP is no longer busy
  {
    ++temp;
  }
  
  while (SSPSR & (1<<2)) //Receive FIFO is not empty
  {
    temp = SSPDR;    //Empty receive buffer
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////
*/

////////////////////////////////////////////////////////////////////////////////////////////////
void csr_clock_tick(void)
{
  static short int i = 0;
  
  ++i;
  if (i >= CSR_TIMER_TICKS_PER_MS)
  {
    i = 0;
    ++csr_ms_counter;
    csr_send_can_time_packets(csr_ms_counter);
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////
unsigned long int csr_elapsed_ms(void)
{
  return csr_ms_counter;
}
////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////

void csr_send_can_time_packets(long unsigned int time)
{       
  //Transmitter buffer 3 should be available for time packet transmit; error if not
  if (C1SR & (1<<18))
  {
    C1TFI3 = 1<<19;   //Time stamp has length 8 bytes
    C1TID3 = 0x0;     //Time stamp CAN_ID is 0
    C1TDA3 = time;
    C1TDB3 = time;    //Send time value in milliseconds
    C1CMR = (1<<0) | (1<<1)  | (1<<7);  //Select TX buffer 3 for transmission with
                                         //simultaneous transmit request and abort.
                                         //This is supposed to give a single transmit
                                         //attempt 
    can_tx_frame_count_1++;
  }
  else
  {
    //Error - buffer 3 not available for CAN1. Previous packet unsent?
  }
  
  
  //Transmitter buffer 3 should be available for time packet transmit; error if not
  if (C2SR & (1<<18))

 { 
    C2TFI3 = 1<<19;   //Time stamp has length 8 bytes
    C2TID3 = 0x0;     //Time stamp CAN_ID is 0
    C2TDA3 = time;    //Time stamp float value is 0
    C2TDB3 = time;    //Send time value in milliseconds
    
    C2CMR = (1<<0) | (1<<1) | (1<<7);     //Select TX buffer 3 for transmission with
                                          //simultaneous transmit request and abort.
                                          //This is supposed to give a single transmit
                                          //attempt 
    can_tx_frame_count_2++;
  }
  else
  {
    //Error - buffer 3 not available for CAN2. Previous packet unsent?
  }
  
  //Transmitter buffer 3 should be available for time packet transmit; error if not
  if (C3SR & (1<<18))
  {
    C3TFI3 = 8<<16;   //Time stamp has length 8 bytes
    C3TID3 = 0x0;     //Time stamp CAN_ID is 0
    C3TDA3 = time;     //Time stamp float value is 0
    C3TDB3 = time;    //Send time value in milliseconds
    
    C3CMR = (1<<0) | (1<<1) | (1<<7);     //Select TX buffer 3 for transmission with
                                          //simultaneous transmit request and abort.
                                          //This is supposed to give a single transmit
                                          //attempt 
    can_tx_frame_count_3++;
  }
  else
  {
    //Error - buffer 3 not available for CAN3. Previous packet unsent?
  }
  
    
    //Transmitter buffer 3 should be available for time packet transmit; error if not
  if (C4SR & (1<<18))
  {
    C4TFI3 = 8<<16;   //Time stamp has length 8 bytes
    C4TID3 = 0x0;     //Time stamp CAN_ID is 0
    C4TDA3 = time;     //Time stamp float value is 0
    C4TDB3 = time;    //Send time value in milliseconds
    
    C4CMR = (1<<0) | (1<<1) | (1<<7);   //Select TX buffer 3 for transmission with
                                        //simultaneous transmit request and abort.
                                        //This is supposed to give a single transmit
                                        //attempt
    can_tx_frame_count_4++; 
  }
  else
  {
    //Error - buffer 4 not available for CAN1. Previous packet unsent?
  } 
  
}

