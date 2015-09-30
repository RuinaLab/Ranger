#include <includes.h>

void FIQ_Handler(void) __irq 
{
  if (VICFIQStatus & 1<<26)
  {
    csr_can_rx1_fiq();
  }
  if (VICFIQStatus & 1<<27)
  {
    csr_can_rx2_fiq();
  }
  if (VICFIQStatus & 1<<28)
  {
    csr_can_rx3_fiq();
  }
  if (VICFIQStatus & 1<<29)
  {
    csr_can_rx4_fiq();
  }
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
    csr_can1_tx();
    route_frames();
    csr_can2_tx();
    route_frames();
    csr_can3_tx();
    route_frames();
    csr_can4_tx(); 
  }
}

