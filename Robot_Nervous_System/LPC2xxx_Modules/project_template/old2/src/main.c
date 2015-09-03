#include <includes.h>

//overarching flow control
int global_running = 1; 
int global_sched_running = 1;


void init(void)
{
  init_hardware();
  init_software(); 
  init_interrupts();
  init_values();
}

int main (void)
{
  init();

  printf("\n\nstarting...\n");
  
  while(global_running){ //the satellite is running
    while (global_sched_running){ //the scheduler is running
      schedule_run();
    }
  }
  MCU_LED_GREEN_OFF;
  MCU_LED_RED_ON;
}

