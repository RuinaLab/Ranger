#include <includes.h>

//Global LED blink variables
volatile unsigned short b10a_red_can1_time = 0, b10a_green_can1_time = 0;
volatile unsigned short b10a_red_can2_time = 0, b10a_green_can2_time = 0;
volatile unsigned short b10a_red_can3_time = 0, b10a_green_can3_time = 0;
volatile unsigned short b10a_red_can4_time = 0, b10a_green_can4_time = 0;

volatile unsigned short b10a_red_mcu_time = 0, b10a_green_mcu_time = 0, b10a_blue_mcu_time = 0;

//CAN bus loading globals
volatile short unsigned int b10a_can1_packet_rate = 0;
volatile short unsigned int b10a_can2_packet_rate = 0;
volatile short unsigned int b10a_can3_packet_rate = 0;
volatile short unsigned int b10a_can4_packet_rate = 0; 

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void b10a_can_packet_count(unsigned short chan)
{
  if      (chan == 1) {++b10a_can1_packet_rate;}
  else if (chan == 2) {++b10a_can2_packet_rate;}
  else if (chan == 3) {++b10a_can3_packet_rate;}
  else if (chan == 4) {++b10a_can4_packet_rate;}
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void b10a_update_mcu_leds(void)
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
    if (b10a_red_mcu_time > 0)
    {
      --b10a_red_mcu_time;
      MCU_LED_RED_ON;
    }
    else
    {
      MCU_LED_RED_OFF;
    }
  
    if (b10a_green_mcu_time > 0)
    {
      --b10a_green_mcu_time;
      MCU_LED_GREEN_ON;
    }
    else
    {
      MCU_LED_GREEN_OFF;
    }
    
    if (b10a_blue_mcu_time > 0)
    {
      --b10a_blue_mcu_time;
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

void b10a_blink_red_can_led(unsigned short chan)
{
  const unsigned short red_blink_time = 50;   // LEDs will flash for time = to 50 function calls of LED update
  
  if      (chan == 1) {b10a_red_can1_time = red_blink_time;}
  else if (chan == 2) {b10a_red_can2_time = red_blink_time;}
  else if (chan == 3) {b10a_red_can3_time = red_blink_time;}
  else if (chan == 4) {b10a_red_can4_time = red_blink_time;}
}

void b10a_blink_green_can_led(unsigned short chan)
{
  const unsigned short green_blink_time = 50;   // LEDs will flash for time = to 50 function calls of LED update
  
  if      (chan == 1) {b10a_green_can1_time = green_blink_time;}
  else if (chan == 2) {b10a_green_can2_time = green_blink_time;}
  else if (chan == 3) {b10a_green_can3_time = green_blink_time;}
  else if (chan == 4) {b10a_green_can4_time = green_blink_time;}
}

void b10a_mcu_red_led_blink(short unsigned int time)
{
  b10a_red_mcu_time = time;
}

void b10a_mcu_green_led_blink(short unsigned int time)
{
  b10a_green_mcu_time = time;
}

void b10a_mcu_blue_led_blink(short unsigned int time)
{
  b10a_blue_mcu_time = time;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void b10a_update_can_leds(void)
{
  static short unsigned int i = 0;

  if (b10a_red_can1_time > 0)
  {
    --b10a_red_can1_time;
    CAN1_LED_RED_ON;
  }
  else
  {
    CAN1_LED_RED_OFF;
  }
  
  if (b10a_green_can1_time > 0)
  {
    --b10a_green_can1_time;
    CAN1_LED_GREEN_ON;
  }
  else
  {
    CAN1_LED_GREEN_OFF;
  }
  
  if (b10a_red_can2_time > 0)
  {
    --b10a_red_can2_time;
    CAN2_LED_RED_ON;
  }
  else
  {
    CAN2_LED_RED_OFF;
  }
  
  if (b10a_green_can2_time > 0)
  {
    --b10a_green_can2_time;
    CAN2_LED_GREEN_ON;
  }
  else
  {
    CAN2_LED_GREEN_OFF;
  }
  
  if (b10a_red_can3_time > 0)
  {
    --b10a_red_can3_time;
    CAN3_LED_RED_ON;
  }
  else
  {
    CAN3_LED_RED_OFF;
  }
  
  if (b10a_green_can3_time > 0)
  {
    --b10a_green_can3_time;
    CAN3_LED_GREEN_ON;
  }
  else
  {
    CAN3_LED_GREEN_OFF;
  }
  
  if (b10a_red_can4_time > 0)
  {
    --b10a_red_can4_time;
    CAN4_LED_RED_ON;
  }
  else
  {
    CAN4_LED_RED_OFF;
  }
  
  if (b10a_green_can4_time > 0)
  {
    --b10a_green_can4_time;
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
    if (b10a_can1_packet_rate > 0){b10a_green_can1_time = (b10a_can1_packet_rate >> 3);}
    if (b10a_can2_packet_rate > 0){b10a_green_can2_time = (b10a_can2_packet_rate >> 3);}
    if (b10a_can3_packet_rate > 0){b10a_green_can3_time = (b10a_can3_packet_rate >> 3);}
    if (b10a_can4_packet_rate > 0){b10a_green_can4_time = (b10a_can4_packet_rate >> 3);}
    
    //Error checks - send error or warning if CAN loading too high
  
    //Reset packet rate/CAN loading variables
    b10a_can1_packet_rate = 0;
    b10a_can2_packet_rate = 0;
    b10a_can3_packet_rate = 0;
    b10a_can4_packet_rate = 0;     
  }    
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
