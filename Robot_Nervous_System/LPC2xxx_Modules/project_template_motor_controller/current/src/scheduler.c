#include <includes.h>

const TASK_PTR * sched_schedule;

int sched_divider;
int sched_div_counter;

volatile int sched_slot_start_index;
int sched_saved_slot_start_index;
volatile int sched_next_task_index;

/*long long int sched_ticks_per_msec = 1;
long long int sched_global_time = 0;*/

int sched_aux_count;

int sched_timestamp;

SCHED_COMPLETION sched_completion = SCHED_DONE;
SCHED_STATUS sched_status = SCHED_NOT_RUNNING;

/**
 * Initialize the scheduler.
 */
void schedule_init(const TASK_PTR * sched, int tick_divider) {
//  printf("%i\t%i\t%i\n",pwm_match0, sched_timestamp_match, sched_tick_match);
  
	sched_schedule = sched;
	sched_divider = tick_divider;
	sched_div_counter = 0;
	sched_slot_start_index = -1;
	sched_saved_slot_start_index = -1;
	sched_next_task_index = -1;
//	sched_ticks_per_msec = ticks_per_msec;

  sched_aux_count = 0;
  
  sched_timestamp = 0;

//	resched_flag = SCHEDULE_RUNNING;
}

/**
 * Schedule the next tasks.  ISR context!
 */
void schedule_tick(void) {
	int search_idx;
	
	if(sched_div_counter >= (sched_divider - 1)){
		//1) check if all tasks have successfully completed from the previous iteration
		if(sched_completion != SCHED_DONE){
      error_occurred(ERROR_SCHED_OVERRUN, PRIORITY_HIGH);
		}	
	  
		//2) schedule next tasks
		if(sched_next_task_index >= 0){		
			search_idx = sched_next_task_index;
			while(sched_schedule[search_idx] != (TASK_PTR)NULL){//find end of current time slot in case of overrun
				search_idx++; 
			}
			search_idx++;//advance to first task of next slot
			//if we have reached the end of the schedule (double NULL), wrap to the beginning
			if(sched_schedule[search_idx] == (TASK_PTR)NULL){
				search_idx = 0; 
			}
		} else {
      sched_status = SCHED_RUNNING_SYNC;
			search_idx = 0; //first run of the scheduler
		}
		sched_slot_start_index = search_idx;//schedule the beginning of the next time slot
		
    sched_completion = SCHED_NOT_DONE;		
//		completion_flag = SCHEDULE_NOTDONE;

		//3) reset divider
		sched_div_counter = 0;
	} else {
		sched_div_counter++;
	}
	
/*	//Increment the global counter
	sched_global_time++;*/
	
}

/*
 * Called by CAN timestamp packet
 * Sets the timestamp and alerts the scheduler of sync packet
 */
void schedule_set_timestamp(int timestamp){
  sched_timestamp = timestamp;
  schedule_sync();
}

void schedule_auxiliary_tick(void){
  sched_aux_count++;
  if (sched_aux_count < SCHED_SPEED) {
    schedule_tick();
  } else if (sched_aux_count == SCHED_SPEED){
    sched_timestamp++;
    if (sched_status == SCHED_RUNNING_ASYNC){
      schedule_tick();
      sched_aux_count = 0;
    }
  } else if (sched_aux_count > SCHED_SPEED) {
    error_occurred(ERROR_SCHED_ASYNC, PRIORITY_HIGH);
    sched_status = SCHED_RUNNING_ASYNC;
    sched_aux_count = 0;
    schedule_tick();
  }
}

void schedule_sync(void){
  if (sched_status == SCHED_RUNNING_ASYNC){
    error_occurred(ERROR_SCHED_RESYNC, PRIORITY_HIGH);
    sched_status = SCHED_RUNNING_SYNC;
  }
  sched_aux_count = 0;
  schedule_tick();
}

/**
 * Execute scheduled tasks.
 */
void schedule_run(void) {
 	//execute this timestep
	TASK_PTR next_task;

	//we do not want things to run before schedule_tick has been called for the first time.
	if(sched_slot_start_index >= 0) {
		while(1){
			if((sched_slot_start_index != sched_saved_slot_start_index)){   //check if rescheduling has occured,
				sched_saved_slot_start_index = sched_slot_start_index;
				sched_next_task_index = sched_slot_start_index;
			}
			next_task = sched_schedule[sched_next_task_index];
			if(next_task != (TASK_PTR)NULL) {
				next_task();//execute next task
				sched_next_task_index++;
			} else {//bah, race conditions!!! ???
        sched_completion = SCHED_DONE;
				break;
			}
		}
	}
	//not sure how sleep scheduling can be done w/o race, does it require a hardware sleep_enable bit?
//	if(SCHEDULE_DONE) if(SLEEP_ENABLED)sleep(); else;
}

int schedule_get_timestamp(void){
  return sched_timestamp;
}

/*long long int schedule_get_global_time(void) {
	return (sched_global_time/sched_ticks_per_msec);
}*/
