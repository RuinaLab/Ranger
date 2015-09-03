#include <includes.h>

int global_running = 1;
int tog = 1;

void init(void)
{
	setup_hardware();
	setup_software(); 
}

int main (void)
{
	init();
	printf("\n\nstarting...\n");
	while(1){

		if (global_running) {
			schedule_run();
		} else { //stopped for some reason

		}
	} 	
}

