/*

	state.c
	
	The State module controls program flow for different states of the satellite, such as
	different commands when it's starting up versus when it's running or exiting.
	
	Nicolas Williamson - January 2010
 
*/
#include <includes.h>

static unsigned long int st_on;
static volatile ST_STATE st_state;
static const TASK_PTR *st_wait_sched;
static const TASK_PTR *st_run_sched;
static const TASK_PTR *st_exit_sched;

void st_init(const TASK_PTR *wait_sched, const TASK_PTR *run_sched, const TASK_PTR *exit_sched){
  mcu_led_green_on();
  st_on = 1;
  st_state = ST_STARTING;
  st_wait_sched = wait_sched;
  st_run_sched = run_sched;
  st_exit_sched = exit_sched;
}

ST_STATE st_get_state(void){
  return st_state;
}

float st_get_status(void){
  switch (st_state){
    case ST_WAITING: return 1.0; //ready
    case ST_RUNNING: return 2.0; //running
    default: error_occurred(ERROR_STATE_INVALID);
      return 0.0; 
  }
}

int st_is_on(void){
  return st_on;
}

//When exiting, reset the scheduler and change state
void st_exit(void){
  sched_reset(st_exit_sched);
  st_state = ST_EXITING;
}

//When entering the waiting period, reset the scheduler and change state
void st_wait(void){
  sched_reset(st_wait_sched);
  st_state = ST_WAITING;
}

//When entering the main run loop, reset the scheduler and change state
void st_run(void){
  mcu_led_green_off();
  sched_reset(st_run_sched);
  st_state = ST_RUNNING;
}

void st_off(void){
  mcu_led_all_off();
  mcu_led_red_on();
  st_on = 0;
  PCON = (1<<1); //Shutoff all power and clocks on the chip
}

void st_main(void){
  st_wait();
  
  while (st_on){ //the board is on
    switch (st_state){
      case ST_STARTING: break; //do nothing until main brain ready packet received
      case ST_WAITING: st_run_waiting(); break;
      case ST_RUNNING: st_run_running(); break;
      case ST_EXITING: st_run_exiting(); break;
    }
  }
}

void st_run_waiting(void){
  sched_run();
}

void st_run_running(void){
  sched_run();
}

void st_run_exiting(void){
  sched_run();
}









