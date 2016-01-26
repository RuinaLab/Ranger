#include <includes.h>

//overarching flow control
int global_running = 1; 
int global_sched_running = 1;

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
  init();   //Run initialization code
    
  while(global_running)
  { //the satellite is running
    while (global_sched_running)
    { //the scheduler is running
      asched_run();
    }
  }
}

