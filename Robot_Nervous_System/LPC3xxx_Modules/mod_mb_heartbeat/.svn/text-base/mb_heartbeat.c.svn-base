#include <mb_includes.h>

#define DEFAULT_HEARTBEAT_PERIOD (250-1) //in units of schedule periods (usually 1 ms)

/*******************
 * Heartbeat Init Section
 * Set pin direction to output for driving LEDs
 * Set initial LED state
 ******************/

/* 
FIO1DIR = 1<<23|1<<25;
FIO1SET = 1<<23|1<<25; 
*/

/********* End Heartbeat Init Section ************/

int hb_phase;
int hb_period = DEFAULT_HEARTBEAT_PERIOD;

void mb_set_heartbeat_period_ms(int period){
  hb_period = period - 1;
}
		 
void mb_task_heartbeat(void){
	if(hb_phase==0 || hb_phase > hb_period){
		mb_hb_toggle();
		hb_phase=hb_period;
	}else{
		hb_phase--;
	}
}

void mb_hb_toggle(void){

 	P3_OUTP_CLR = 1<<25;
      
//  } else {
//  	P3_OUTP_SET = 1<<25;
//  }
}
