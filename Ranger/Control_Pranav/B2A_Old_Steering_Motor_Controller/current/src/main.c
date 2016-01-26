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
  init();

  while (global_running){
    asched_run();
    i2c_color_update();
  }

}

