#include <includes.h>

//overarching flow control
int global_running = 1; 
int global_sched_running = 1;

// **** TEST CODE ****
unsigned long old_test_timer = 0;
extern QDC_DATA qdc_tmr0_data;


void init(void)
{
  init_hardware();
  init_software(); 
  init_can();
  init_interrupts();
  init_values();
}

int main (void)
{
  
 /////////////////////////////////////////////
 //Debugger switch setup. Execution stops here until switch is flipped.
   #ifdef DEBUG
    SCS = (1<<0)|(1<<1);       // enable high-speed GPIO0 (bit 0) and GPIO1 (bit 1) (Fast GPIO)
    FIO1DIR |= (1<<24);   // set P1.24 to be output (Red LED)
    while (FIO0PIN & 1<<14)
    {
      FIO0CLR = 1<<24;   //Turn on MCU red LED
    }
  #endif
 //////////////////////////////////////////////
  
  init();
    
  printf("\n\nstarting...\n");

  while (global_running)
  {
    asched_run();

/*    // **** TEST CODE ****
    if (T0TC <= old_test_timer)
    {
      if (qdc_tmr0_data.overflow_01 < 30000)
      {
        mcu_led_red_blink(50);
      }
      qdc_tmr0_data.overflow_01 = 0;
      old_test_timer =  T0TC;
    }*/
  } 
  
  mcu_led_red_on();

}

