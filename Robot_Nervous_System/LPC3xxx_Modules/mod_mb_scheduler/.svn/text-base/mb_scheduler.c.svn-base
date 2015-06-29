#include <mb_includes.h>

const TASK_PTR * sched_schedule;
int sched_divider;

int sched_div_counter;
volatile int sched_slot_start_index;
int sched_saved_slot_start_index;
volatile int sched_next_task_index;

/**
 * Initialize the scheduler.
 */
void mb_schedule_init(const TASK_PTR * sched, int tick_divider) {
  sched_schedule = sched;
  sched_divider = tick_divider;
  sched_div_counter = 0;
	sched_slot_start_index = -1;
	sched_saved_slot_start_index = -1;
	sched_next_task_index = -1;
//	resched_flag = SCHEDULE_RUNNING;
}

/**
 * Schedule the next tasks.  ISR context!
 */
void mb_schedule_tick(void) {
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
	
}

/**
 * Execute scheduled tasks.
 */
void mb_schedule_run(void) {
 	//execute this timestep
	TASK_PTR next_task;
	//we do not want things to run before schedule_tick has been called for the first time.
	if(sched_slot_start_index >= 0) {
		while(1){
			if((sched_slot_start_index != sched_saved_slot_start_index)){   //check if rescheduling has occurred,
				sched_saved_slot_start_index = sched_slot_start_index;
				sched_next_task_index = sched_slot_start_index;
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

