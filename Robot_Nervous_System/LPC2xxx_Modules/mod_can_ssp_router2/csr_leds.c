#include <includes.h>

//Global LED blink variables
volatile unsigned short csr_red_can1_time = 0, csr_green_can1_time = 0;
volatile unsigned short csr_red_can2_time = 0, csr_green_can2_time = 0;
volatile unsigned short csr_red_can3_time = 0, csr_green_can3_time = 0;
volatile unsigned short csr_red_can4_time = 0, csr_green_can4_time = 0;

volatile unsigned short csr_red_mcu_time = 0, csr_green_mcu_time = 0, csr_blue_mcu_time = 0;

//CAN bus loading globals
volatile short unsigned int csr_can1_packet_rate = 0;
volatile short unsigned int csr_can2_packet_rate = 0;
volatile short unsigned int csr_can3_packet_rate = 0;
volatile short unsigned int csr_can4_packet_rate = 0; 

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void csr_can1_packet_count(void)
{
  ++csr_can1_packet_rate;
}

void csr_can2_packet_count(void)
{
  ++csr_can2_packet_rate;
}

void csr_can3_packet_count(void)
{
  ++csr_can3_packet_rate;
}

void csr_can4_packet_count(void)
{
  ++csr_can4_packet_rate;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void csr_update_mcu_leds(void)
{
/*
  if (FIO1PIN & 25) //ARM9 controls MCU G/R LEDs while blue LED is off
  {
    if (FIO0PIN & 1<<15)	   //LPC3250 GPIO_0
  	{
	   MCU_LED_GREEN_ON; 	 	//turn on green ARM7 MCU LED
	  }
	  else
	  {
     MCU_LED_GREEN_OFF;    //turn off green ARM7 MCU LED
	  }

	  if (FIO0PIN & 1<<16)	   //LPC3250 GPIO_1
	  {
	    MCU_LED_RED_ON; 	 	// turn on red MCU LED
	  }
	  else
	  {
      MCU_LED_RED_OFF;    // turn off red MCU LED
	  }  
  }
  else  //ARM7 controls MCU G/R LEDs while blue LED is on
  */  // *** test code
  
  {
    if (csr_red_mcu_time > 0)
    {
      --csr_red_mcu_time;
      MCU_LED_RED_ON;
    }
    else
    {
      MCU_LED_RED_OFF;
    }
  
    if (csr_green_mcu_time > 0)
    {
      --csr_green_mcu_time;
      MCU_LED_GREEN_ON;
    }
    else
    {
      MCU_LED_GREEN_OFF;
    }
    
    if (csr_blue_mcu_time > 0)
    {
      --csr_blue_mcu_time;
      MCU_LED_BLUE_ON;
    }
    else
    {
      MCU_LED_BLUE_OFF;
    }
    
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void csr_mcu_red_led_blink(short unsigned int time)
{
  csr_red_mcu_time = time;
}

void csr_mcu_green_led_blink(short unsigned int time)
{
  csr_green_mcu_time = time;
}

void csr_mcu_blue_led_blink(short unsigned int time)
{
  csr_blue_mcu_time = time;
}

void csr_can1_red_led_blink(short unsigned int time)
{
  csr_red_can1_time = time;
}

void csr_can1_green_led_blink(short unsigned int time)
{
  csr_green_can1_time = time;
}

void csr_can2_red_led_blink(short unsigned int time)
{
  csr_red_can2_time = time;
}

void csr_can2_green_led_blink(short unsigned int time)
{
  csr_green_can2_time = time;
}

void csr_can3_red_led_blink(short unsigned int time)
{
  csr_red_can3_time = time;
}

void csr_can3_green_led_blink(short unsigned int time)
{
  csr_green_can3_time = time;
}

void csr_can4_red_led_blink(short unsigned int time)
{
  csr_red_can4_time = time;
}

void csr_can4_green_led_blink(short unsigned int time)
{
  csr_green_can4_time = time;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void csr_update_can_leds(void)
{
  static short unsigned int i = 0;

  if (csr_red_can1_time > 0)
  {
    --csr_red_can1_time;
    CAN1_LED_RED_ON;
  }
  else
  {
    CAN1_LED_RED_OFF;
  }
  
  if (csr_green_can1_time > 0)
  {
    --csr_green_can1_time;
    CAN1_LED_GREEN_ON;
  }
  else
  {
    CAN1_LED_GREEN_OFF;
  }
  
  if (csr_red_can2_time > 0)
  {
    --csr_red_can2_time;
    CAN2_LED_RED_ON;
  }
  else
  {
    CAN2_LED_RED_OFF;
  }
  
  if (csr_green_can2_time > 0)
  {
    --csr_green_can2_time;
    CAN2_LED_GREEN_ON;
  }
  else
  {
    CAN2_LED_GREEN_OFF;
  }
  
  if (csr_red_can3_time > 0)
  {
    --csr_red_can3_time;
    CAN3_LED_RED_ON;
  }
  else
  {
    CAN3_LED_RED_OFF;
  }
  
  if (csr_green_can3_time > 0)
  {
    --csr_green_can3_time;
    CAN3_LED_GREEN_ON;
  }
  else
  {
    CAN3_LED_GREEN_OFF;
  }
  
  if (csr_red_can4_time > 0)
  {
    --csr_red_can4_time;
    CAN4_LED_RED_ON;
  }
  else
  {
    CAN4_LED_RED_OFF;
  }
  
  if (csr_green_can4_time > 0)
  {
    --csr_green_can4_time;
    CAN4_LED_GREEN_ON;
  }
  else
  {
    CAN4_LED_GREEN_OFF;
  }
  
  
  //Light up green LEDs for a time proportional to CAN bus loading
  ++i;
  if (i >= 10)
  {
    i = 0;
    if (csr_can1_packet_rate > 0){csr_green_can1_time = (csr_can1_packet_rate >> 4);}
    if (csr_can2_packet_rate > 0){csr_green_can2_time = (csr_can2_packet_rate >> 4);}
    if (csr_can3_packet_rate > 0){csr_green_can3_time = (csr_can3_packet_rate >> 4);}
    if (csr_can4_packet_rate > 0){csr_green_can4_time = (csr_can4_packet_rate >> 4);}
    
    //Error checks - send error or warning if CAN loading too high
  
    //Reset packet rate/CAN loading variables
    csr_can1_packet_rate = 0;
    csr_can2_packet_rate = 0;
    csr_can3_packet_rate = 0;
    csr_can4_packet_rate = 0;     
  }    
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
