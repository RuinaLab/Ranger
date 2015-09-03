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

  st_main();

}

