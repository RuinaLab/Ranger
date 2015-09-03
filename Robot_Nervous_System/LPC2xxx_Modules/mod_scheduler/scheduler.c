/*

  scheduler.c
  
  The scheduler module decides what task is to be run next.
  
  Tommy Craig - 2008
  Syncing added Jan. 2010 by Nicolas Williamson
  
*/

#include <includes.h>

static MUTEX sched_mutex;

static const TASK_PTR * sched_schedule;

static int sched_divider;
static int sched_div_counter;

static volatile int sched_slot_start_index;
static int sched_saved_slot_start_index;
static volatile int sched_next_task_index;

/*long long int sched_ticks_per_msec = 1;
long long int sched_global_time = 0;*/

static volatile unsigned int sched_aux_count;
static volatile unsigned int sched_sync_count;
static volatile unsigned int sched_timestamp;
static unsigned int sched_sync_match; 
static unsigned int sched_aux_match;

static SCHED_TIMER_RESET sched_sync_reset_timer = voidvoid;
static SCHED_TIMER_RESET sched_aux_reset_timer = voidvoid;

static SCHED_COMPLETION sched_completion;
static SCHED_STATUS sched_status;

/**
 * Initialize the scheduler.
 */
void sched_init(const TASK_PTR * sched, int tick_divider, int sync_rate, SCHED_TIMER_RESET sync_reset, SCHED_TIMER_RESET aux_reset) { //sync_rate is in milliseconds
	sched_schedule = sched;
	sched_divider = tick_divider;
	sched_div_counter = 0;
	sched_slot_start_index = -1;
	sched_saved_slot_start_index = -1;
	sched_next_task_index = -1;
//	sched_ticks_per_msec = ticks_per_msec;

  sched_aux_count = 0;
  sched_sync_count = 0;
  sched_timestamp = 0;
  sched_sync_match = sync_rate * SCHED_SPEED;
  sched_aux_match = SCHED_SPEED;
  sched_sync_reset_timer = sync_reset;
  sched_aux_reset_timer = aux_reset;
  
  sched_completion = SCHED_DONE;
  sched_status = SCHED_NOT_RUNNING;

//	resched_flag = sched_RUNNING;
}

/*
 * Reset the scheduler to run with the new schedule
 */
void sched_reset(const TASK_PTR * sched){
	sched_schedule = sched;
	sched_div_counter = 0;
	sched_slot_start_index = -1;
	sched_saved_slot_start_index = -1;
	sched_next_task_index = -1;

  sched_aux_count = 0;
  sched_sync_count = 0;
  sched_timestamp = 0;
  
  sched_completion = SCHED_DONE;
  sched_status = SCHED_NOT_RUNNING;
}

/**
 * Schedule the next tasks.  ISR context!
 */
void sched_tick(void) {
	int search_idx;
  
  if (!mutex_check(&sched_mutex)){ //mutex is unlocked
  
    mutex_lock(&sched_mutex);
	
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
  
  		//3) reset divider
  		sched_div_counter = 0;
  	} else {
  		sched_div_counter++;
  	}
    
    mutex_unlock(&sched_mutex);
    
  } else {//mutex previously locked by some other process
    //do nothing
  }
	
}

/*
 * Called by CAN timestamp packet
 * Sets the timestamp and alerts the scheduler of sync packet
 */
void sched_set_timestamp(int timestamp){
  sched_timestamp = timestamp;
}

/*
 * Called by the auxiliary timer to provide
 * backup ticks if asynchronous
 */
void sched_auxiliary_tick(){
  
  sched_aux_count++;
  sched_sync_count++;
  
  if (sched_status == SCHED_NOT_RUNNING){
    sched_status = SCHED_RUNNING_SYNC;
  }
  
  // ******** SYNC ********* //
  if (sched_status == SCHED_RUNNING_SYNC){
    //increase timestamp if match
    if (sched_aux_count >= sched_aux_match){
      sched_timestamp++;
      sched_aux_count = 0;
    } //in sync, don't expect sync yet, so sched_tick
    if (sched_sync_count < sched_sync_match){ 
      sched_aux_reset_timer();
      sched_tick(); 
    } //expected sync, go async and sched_tick
    else if (sched_sync_count > sched_sync_match){ 
      sched_aux_reset_timer();
      error_occurred(ERROR_SCHED_ASYNC, PRIORITY_HIGH);
      sched_status = SCHED_RUNNING_ASYNC;
      sched_aux_count = 0;
      sched_tick();
    } 
  } //******* ASYNC *******//
  else if (sched_status == SCHED_RUNNING_ASYNC){
    sched_aux_reset_timer();
    //increase timestamp if match
    if (sched_aux_count >= sched_aux_match){
      sched_timestamp++;
      sched_aux_count = 0;
    } //running async, so always generate tick
    sched_tick();
  }
}

/*
 * Called when a sync timestamp packet
 * is received from the main brain. Resyncs the
 * scheduler if it is asynchronous.
 */
void sched_sync(void){
  if (sched_status == SCHED_RUNNING_ASYNC){
    error_occurred(ERROR_SCHED_RESYNC, PRIORITY_HIGH);
    sched_status = SCHED_RUNNING_SYNC;
  }
  sched_aux_count = 0;
  sched_sync_count = 0;
  sched_tick();
}

/**
 * Execute scheduled tasks.
 */
void sched_run(void) {
 	//execute this timestep
	TASK_PTR next_task;

	//we do not want things to run before sched_tick has been called for the first time.
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
//	if(sched_DONE) if(SLEEP_ENABLED)sleep(); else;
}

int sched_get_timestamp(void){
  return sched_timestamp;
}

SCHED_STATUS sched_get_status(void){
  return sched_status;
}

/*long long int sched_get_global_time(void) {
	return (sched_global_time/sched_ticks_per_msec);
}*/
