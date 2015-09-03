/*
	async_scheduler.c
	
	Simpler asynchronous scheduler - Tommy's Original
*/


#include <includes.h>

static const TASK_PTR * asched_schedule;

static int asched_divider;
static int asched_div_counter;

static volatile int asched_slot_start_index;
static int asched_saved_slot_start_index;
static volatile int asched_next_task_index;

static volatile long int asched_timestamp;

static ASCHED_COMPLETION asched_completion;

/**
 * Initialize the scheduler.
 */
void asched_init(const TASK_PTR * sched, int tick_divider) { 
	asched_schedule = sched;
	asched_divider = tick_divider;
	asched_div_counter = 0;
	asched_slot_start_index = -1;
	asched_saved_slot_start_index = -1;
	asched_next_task_index = -1;
  
  asched_completion = ASCHED_DONE;
}

/**
 * Schedule the next tasks.  ISR context!
 */
void asched_tick(void) {
	int search_idx;
	
	if(asched_div_counter >= (asched_divider - 1)){
		//1) check if all tasks have successfully completed from the previous iteration
		if(asched_completion != ASCHED_DONE){
      error_occurred(ERROR_ASCHED_DNF);
		}	
	  
		//2) schedule next tasks
		if(asched_next_task_index >= 0){		
			search_idx = asched_next_task_index;
			while(asched_schedule[search_idx] != (TASK_PTR)NULL){//find end of current time slot in case of overrun
				search_idx++; 
			}
			search_idx++;//advance to first task of next slot
			//if we have reached the end of the schedule (double NULL), wrap to the beginning
			if(asched_schedule[search_idx] == (TASK_PTR)NULL){
				search_idx = 0; 
			}
		} else {
			search_idx = 0; //first run of the scheduler
		}
		asched_slot_start_index = search_idx;//schedule the beginning of the next time slot
		
    asched_completion = ASCHED_NOT_DONE;	

		//3) reset divider
		asched_div_counter = 0;
	} else {
		asched_div_counter++;
	}
	
}

/*
 * Called by CAN timestamp packet
 * Sets the timestamp and alerts the scheduler of sync packet
 */
void asched_set_timestamp(int timestamp){
  asched_timestamp = timestamp;
}

/**
 * Execute scheduled tasks.
 */
void asched_run(void) {
 	//execute this timestep
	TASK_PTR next_task;

	//we do not want things to run before asched_tick has been called for the first time.
	if(asched_slot_start_index >= 0) {
		while(1){
			if((asched_slot_start_index != asched_saved_slot_start_index)){   //check if rescheduling has occured,
				asched_saved_slot_start_index = asched_slot_start_index;
				asched_next_task_index = asched_slot_start_index;
			}
			next_task = asched_schedule[asched_next_task_index];
			if(next_task != (TASK_PTR)NULL) {
				next_task();//execute next task
				asched_next_task_index++;
			} else {//bah, race conditions!!! ???
        asched_completion = ASCHED_DONE;
				break;
			}
		}
	}
}

int asched_get_timestamp(void){
  return asched_timestamp;
}

void asched_isr(void) __irq{
  PWMIR = 0XFFFF;
  asched_tick();
  VICVectAddr = 0;
}
