#include <includes.h>

void FIQ_Handler(void) __irq {

}

void init(void)
{
  setup_hardware(); 
  setup_software();
}

int main (void)
{
  init();
  printf("\n\nstarting...\n\n");

//  can_probe_go();
//  can_probe_spit();
  while(1){
// 	  schedule_run();
    can_probe_spit();
   };

}

