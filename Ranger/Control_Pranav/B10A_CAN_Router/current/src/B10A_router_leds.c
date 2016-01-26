#include <includes.h>

//Global LED blink variables
volatile unsigned short b10a_red_can1_time = 0, b10a_green_can1_time = 0;
volatile unsigned short b10a_red_can2_time = 0, b10a_green_can2_time = 0;
volatile unsigned short b10a_red_can3_time = 0, b10a_green_can3_time = 0;
volatile unsigned short b10a_red_can4_time = 0, b10a_green_can4_time = 0;

volatile unsigned short b10a_red_mcu_time = 0, b10a_green_mcu_time = 0, b10a_blue_mcu_time = 0;

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
    if (FIO0PIN & 1<<16)	   //LPC3250 GPIO_1
	  {
	    b10a_red_mcu_time = 50; 	 	// turn on red MCU LED
	  }
    
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

void b10a_blink_red_can_led(unsigned short chan, unsigned short time)
{
  // LEDs flash duration = to "time" function calls of LED update
  if      (chan == 1) {b10a_red_can1_time = time;}
  else if (chan == 2) {b10a_red_can2_time = time;}
  else if (chan == 3) {b10a_red_can3_time = time;}
  else if (chan == 4) {b10a_red_can4_time = time;}
}

void b10a_blink_green_can_led(unsigned short chan, unsigned short time)
{
  // LEDs flash duration = to "time" function calls of LED update
  if      (chan == 1) {b10a_green_can1_time = time;}
  else if (chan == 2) {b10a_green_can2_time = time;}
  else if (chan == 3) {b10a_green_can3_time = time;}
  else if (chan == 4) {b10a_green_can4_time = time;}
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
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
