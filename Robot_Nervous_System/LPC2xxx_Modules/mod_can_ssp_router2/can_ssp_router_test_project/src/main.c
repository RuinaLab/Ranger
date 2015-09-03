#include <includes.h>

void FIQ_Handler(void) __irq 
{
  csr_ssp_isr();
}

void init(void)
{
  setup_hardware(); 
  setup_software();
}

int main (void)
{
  
 /////////////////////////////////////////////
 //Debugger switch setup. Execution stops here until switch is flipped.
   #ifdef DEBUG
    while (FIO0PIN & 1<<14)
    {
      FIO0CLR = 1<<24;   //Turn on MCU red LED
    }
  #endif
 //////////////////////////////////////////////

  init();
 
  while(1)
  {
	  asched_run();
	  route_frames(); 
  }
}

