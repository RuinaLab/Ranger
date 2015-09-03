#include <includes.h>

const TASK_PTR * sched_schedule;
int sched_divider;

int sched_div_counter;
volatile int sched_slot_start_index;
int sched_saved_slot_start_index;
volatile int sched_next_task_index;

long long int sched_ticks_per_msec = 1;
long long int sched_global_time = 0;

/**
 * Initialize the scheduler.
 */
void schedule_init(const TASK_PTR * sched, int tick_divider, long long int ticks_per_msec) {
	sched_schedule = sched;
	sched_divider = tick_divider;
	sched_div_counter = 0;
	sched_slot_start_index = -1;
	sched_saved_slot_start_index = -1;
	sched_next_task_index = -1;
	sched_ticks_per_msec = ticks_per_msec;
//	resched_flag = SCHEDULE_RUNNING;
}

/**
 * Schedule the next tasks.  ISR context!
 */
void schedule_tick(void) {
	int search_idx;
	
	if(sched_div_counter >= (sched_divider - 1)){
		//1) check if all tasks have successfully completed from the previous iteration
//		if(completion_flag != SCHEDULE_DONE){
			//TODO: alert about this, it's an error condition on the part of user code
//			FIO1CLR_bit.P1_24=1;	 // turn on red LED
//		}	
	
		//2) schedule next tasks
		if(sched_next_task_index >= 0){		
			search_idx = sched_next_task_index;
			//find end of current time slot in case of overrun
			while(sched_schedule[search_idx] != (TASK_PTR)NULL){
				search_idx++; 
			}
			//advance to first task of next slot
			search_idx++;
			//if we have reached the end of the schedule (double NULL), wrap to the beginning
			if(sched_schedule[search_idx] == (TASK_PTR)NULL){
				search_idx = 0; 
			}
		} else {
			//first run of the scheduler
			search_idx = 0;
		}
		//schedule the beginning of the next time slot
		sched_slot_start_index = search_idx;
				
//		completion_flag = SCHEDULE_NOTDONE;

		//3) reset divider
		sched_div_counter = 0;
	} else {
		sched_div_counter++;
	}
	
	//Increment the global counter
	sched_global_time++;
	
}

/**
 * Execute scheduled tasks.
 */
void schedule_run(void) {
 	//execute this timestep
	TASK_PTR next_task;

	//Copy this to ensure the value we use doesn't change mid-execution
	int curr_sched_slot_start_index;

	//we do not want things to run before schedule_tick has been called for the first time.
	if(sched_slot_start_index >= 0) {
		while(1){
            curr_sched_slot_start_index = sched_slot_start_index;
			
            if((curr_sched_slot_start_index != sched_saved_slot_start_index)){   //check if rescheduling has occured,
				sched_saved_slot_start_index = curr_sched_slot_start_index;
				sched_next_task_index = curr_sched_slot_start_index;
			}
			
            next_task = sched_schedule[sched_next_task_index];
			
            if(next_task != (TASK_PTR)NULL) {
				next_task();//execute next task
				sched_next_task_index++;
			} else {//bah, race conditions!!! ???
				break;
			}
		}
	}
	//not sure how sleep scheduling can be done w/o race, does it require a hardware sleep_enable bit?
//	if(SCHEDULE_DONE) if(SLEEP_ENABLED)sleep(); else;
}

long long int schedule_get_global_time(void) {
	return (sched_global_time/sched_ticks_per_msec);
}
