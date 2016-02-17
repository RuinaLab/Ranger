/**
	@file ui_sync_led.c
	Providing control of the flash synchronize LED on the UI board since 2010.

  @author Nan(Sarah) Xiao
  @date April 2010

*/

#include <includes.h>

#define FLASH_PERIOD_HALFQUARTER 125 //125ms
#define FLASH_PERIOD_QUARTER 250 //250ms
#define FLASH_PERIOD_HALF 500 //500ms
#define FLASH_PERIOD_SEC 1000 //1000ms
#define LED_ON_PERIOD 50  //50ms

int led_on_flag = 0;
int ts = 0;

/**
  A function which returns the current timestamp on the board.
*/
static INT_VOID_F led_get_timestamp = intvoid;

/**
  Initializes the @c ui_sync_led module. Must call this function first.
  @param timestamp_in A function which takes no parameters and returns the timestamp as an int.
*/
void ui_sync_led_init(INT_VOID_F timestamp_in)
{
  led_get_timestamp = timestamp_in;
  ui_sync_led_off();
}		

/**
  Turns on the sync led.
*/
void ui_sync_led_on(void)
{
	ui_led_rgb(1, 100,0,0);   //LED1, red, at back
 // FIO0SET = 1<<16;    //Green top LED on
    FIO0CLR = 1<<16;      //Note: MattS changed logic backwards due to P-channel mosfet to drive the bright LED
}

/**
  Turns off the sync led.
*/
void ui_sync_led_off(void)
{
	ui_led_rgb(1, 0,0,0); //LED1, red, at back
 // FIO0CLR = 1<<16;      //Green top LED off
  FIO0SET = 1<<16;    //Note: MattS changed logic backwards due to P-channel mosfet to drive the bright LED
}

/**
  Updates the flashing of the sync led based on the timestamp.
  Call this every row of the schedule.
*/
void ui_sync_led_flash(void)
{  
  static unsigned short flag = 0;
  unsigned long remainder;

  remainder = led_get_timestamp() % FLASH_PERIOD_HALFQUARTER;
  
  //This logic is intended to detect the first iteration at which the remainder
  //has passed zero, and to run the ui_sync_led_flash_pattern at that time only.
  //Note that the remainder is non-negative.
  if (remainder < (FLASH_PERIOD_HALFQUARTER >> 1))
  {
    if (flag)
    {
      ui_sync_led_flash_pattern();
    }
    flag = 0;	  		
  }
  else
  {
    flag = 1;
  }	
}

/**
  Calculates the flash pattern for the current timestamp and
  blinks the sync led appropriately.
*/
static void ui_sync_led_flash_pattern(void)
{
	static int sync_flag = 0;
  static unsigned short flag = 0;
  unsigned long remainder = led_get_timestamp() % FLASH_PERIOD_SEC;
	int sync_count = (led_get_timestamp() / FLASH_PERIOD_HALFQUARTER) % 8;

  if(led_get_timestamp() - ts > LED_ON_PERIOD) led_on_flag = 0;

  //This logic is intended to detect the first iteration at which the remainder
  //has passed zero, and to run the ui_sync_led_flash_pattern at that time only.
  //Note that the remainder is non-negative.
  if (remainder < (FLASH_PERIOD_SEC >> 1))
  {
		if (flag)
    {
      sync_flag++;
      if (sync_flag >= 10)
      {
			  sync_flag = 0;
		  }			
      flag = 0;
    }      	
  }
  else
  {
    flag = 1;
  }
	switch (sync_flag) {	
		case 0:
			if ((sync_count == 0) || (sync_count == 6)){
				led_on_flag = 1;
				ts = led_get_timestamp();
			} 
//			else if (sync_count == 2){
//				ui_sync_led_off();
//			}
			break;	 
		case 1:
			if (sync_count == 2){
				led_on_flag = 1;
				ts = led_get_timestamp();
			} 
//			else if (sync_count == 4){
//				ui_sync_led_off();
//			} 
			break;		 
		case 2:
			if ((sync_count == 0) || (sync_count == 4)){
				led_on_flag = 1;
				ts = led_get_timestamp();
			} 
//			else if ((sync_count == 2) || (sync_count == 6)){
//				ui_sync_led_off();
//			}  
			break;	   
		case 3:
			if ((sync_count == 0) || (sync_count == 7)){
				//ui_sync_led_on();
				led_on_flag = 1;
				ts = led_get_timestamp();
			} 
//			else if (sync_count == 2){
//				ui_sync_led_off();
//			} 
			break;
		case 4:
			if ((sync_count == 2) || (sync_count == 6)){
				//ui_sync_led_on();
				led_on_flag = 1;
				ts = led_get_timestamp();
			} 
//			if ((sync_count == 0) || (sync_count == 4)){
//				ui_sync_led_off();
//			} 
			break;
		case 5:
			if (sync_count == 2){
				///ui_sync_led_on();
				led_on_flag = 1;
				ts = led_get_timestamp();
			} 
//			else if ((sync_count == 0) || (sync_count == 4)){
//				ui_sync_led_off();
//			} 
			break;
		case 6:
			if ((sync_count == 0) || (sync_count == 4)){
				ui_sync_led_on();
			} 
//			else if ((sync_count == 2) || (sync_count == 6)){
//				ui_sync_led_off();
//			}  
			break;
		case 7:
			if ((sync_count == 0) || (sync_count == 4)){
				//ui_sync_led_on();
				led_on_flag = 1;
				ts = led_get_timestamp();
			} 
//			else if ((sync_count == 2) || (sync_count == 6)){
//				ui_sync_led_off();
//			} 
			break;
		case 8:
			if ((sync_count == 0) || (sync_count == 4) || (sync_count == 6)){
				//ui_sync_led_on();
				led_on_flag = 1;
				ts = led_get_timestamp();
			} 
//			else if ((sync_count == 2) || (sync_count == 5) || (sync_count == 7)){
//				ui_sync_led_off();
//			}
			break;
			
		case 9:
			if ((sync_count == 0) || (sync_count == 2) || (sync_count == 4) || (sync_count == 6)){
				//ui_sync_led_on();
				led_on_flag = 1;
				ts = led_get_timestamp();
			} 
//			else if ((sync_count == 1) || (sync_count == 3) || (sync_count == 5) || (sync_count == 7)){
//				ui_sync_led_off();
//			}
			break;				   
	}

	//flash LED
	if(led_on_flag == 1) ui_sync_led_on();
	else ui_sync_led_off();
}




