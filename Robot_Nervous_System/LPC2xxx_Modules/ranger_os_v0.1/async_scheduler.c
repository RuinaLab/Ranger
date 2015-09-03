/**
	@file async_scheduler.c
  Simple asynchronous scheduler. 
	
	@author Tommy Craig
*/


#include <includes.h>

#ifndef __VERSION_0_1__
#warning RangerOS mismatch, expected v0.1. 
#endif

/**  
  This array is a 2D table of tasks. 
  One row of the array will be run for every call to asched_tick.
  Each row is @c NULL terminated, and the array is doubly @c NULL terminated.
  A @c VOID_VOID_F is a pointer to a function that takes @c void parameters and returns @c void.
*/
static const VOID_VOID_F * asched_schedule; 

static int asched_divider;
static int asched_div_counter;

static volatile int asched_slot_start_index; 
static int asched_saved_slot_start_index;
static volatile int asched_next_task_index;

static volatile long int asched_timestamp;

static ASCHED_COMPLETION asched_completion;

/** 
  Initializes the scheduler. Needs to be called in software setup.
  @param sched A 2D array representing the schedule.
  @param tick_divider Calls to @ref asched_tick will count to @c tick_divider - 1 before calling the next row. 
*/
void asched_init(const VOID_VOID_F * sched, int tick_divider) { 
	asched_schedule = sched;
	asched_divider = tick_divider;
	asched_div_counter = 0;
	asched_slot_start_index = -1;
	asched_saved_slot_start_index = -1;
	asched_next_task_index = -1;
  
  asched_completion = ASCHED_DONE;
}

/** 
  Signals the scheduler to start executing the next row of tasks in the schedule.
  If the previous row did not finish before @ref asched_tick was called again, this will 
  cause an @c ERROR_ASCHED_DNF error, and will wait for the previous row to finish executing.
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
			while(asched_schedule[search_idx] != (VOID_VOID_F)NULL){//find end of current time slot in case of overrun
				search_idx++; 
			}
			search_idx++;//advance to first task of next slot
			//if we have reached the end of the schedule (double NULL), wrap to the beginning
			if(asched_schedule[search_idx] == (VOID_VOID_F)NULL){
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

/** 
  This function sets the current timestamp to @c timestamp
  @param timestamp The current timestamp
*/
void asched_set_timestamp(int timestamp){
  asched_timestamp = timestamp;
}

/** 
  Executes the next row of scheduled tasks. 
  Call this function continuously from your @c main while loop.
*/
void asched_run(void) {
 	//execute this timestep
	VOID_VOID_F next_task;

	//we do not want things to run before asched_tick has been called for the first time.
	if(asched_slot_start_index >= 0) {
		while(1){
			if((asched_slot_start_index != asched_saved_slot_start_index)){   //check if rescheduling has occured,
				asched_saved_slot_start_index = asched_slot_start_index;
				asched_next_task_index = asched_slot_start_index;
			}
			next_task = asched_schedule[asched_next_task_index];
			if(next_task != (VOID_VOID_F)NULL) {
				next_task();//execute next task
				asched_next_task_index++;
			} else {//bah, race conditions!!! ???
        asched_completion = ASCHED_DONE;
				break;
			}
		}
	}
}


/** 
  Gets the current timestamp of the board.
  The timestamp is usually updated by the main brain.
  @return The current timestamp
*/
int asched_get_timestamp(void){
  return asched_timestamp;
}

