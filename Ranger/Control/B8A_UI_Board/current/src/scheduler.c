#include <includes.h>

const TASK_PTR * sched_schedule;

int sched_divider;
int sched_div_counter;

volatile int sched_slot_start_index;
int sched_saved_slot_start_index;
volatile int sched_next_task_index;

/*long long int sched_ticks_per_msec = 1;
long long int sched_global_time = 0;*/

int sched_timer_count = 0; //pwm timer counts; 0.01ms per count (1e-5)
int sched_tick_match; //when the timer counter generates a tick
int sched_timestamp_match; //when the timer counter waits for a timestamp CAN packet
int sched_out_of_sync = 0; //if the schedule is out of sync 
int sched_sync_delay = 2; //how many timer_counts past ms we wait for CAN sync packet
int sched_late_count = 0;
int sched_late_limit = 10; //limit the CAN sync packets can run late before some sort of safe shutdown function

int sched_timestamp = 0;

SCHED_COMPLETION sched_completion = SCHED_DONE;
SCHED_STATUS sched_status = SCHED_NOT_RUNNING;

/**
 * Initialize the scheduler.
 */
void schedule_init(const TASK_PTR * sched, int tick_divider) {
  int pwm_match0 = PWMMR0; //600 -> timer_count = 0.01ms
  sched_timestamp_match = (60000/pwm_match0); //expect a timestamp every ms
  sched_tick_match = (sched_timestamp_match/SCHED_SPEED); //timer generated tick at sched_speed
  
//  printf("%i\t%i\t%i\n",pwm_match0, sched_timestamp_match, sched_tick_match);
  
	sched_schedule = sched;
	sched_divider = tick_divider;
	sched_div_counter = 0;
	sched_slot_start_index = -1;
	sched_saved_slot_start_index = -1;
	sched_next_task_index = -1;
//	sched_ticks_per_msec = ticks_per_msec;

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
 * ISR generated from PWM timer
 */
void schedule_isr(void) __irq {
  sched_timer_count++;
  schedule_alert(SCHED_TIMER);
  PWMIR = 1; //clear match0 interrupt flag
  VICVectAddr = 0;    // Clear interrupt in VIC.
}

/*
 * Called by CAN timestamp packet
 * Sets the timestamp and alerts the scheduler of sync packet
 */
void schedule_set_timestamp(int timestamp){
  sched_timestamp = timestamp;
  schedule_alert(SCHED_CAN);
}

/*
 * alert that CAN packet has been received or timer generated interrupt
 */
void schedule_alert(SCHED_ALERT_SOURCE source){
  
  // **** TIMER ****
  if (source == SCHED_TIMER){ //timer generated an alert
   //if we aren't yet waiting for a sync packet and counter matches, generate tick
    if (sched_timer_count < sched_timestamp_match &&
        sched_timer_count % sched_tick_match == 0){
      schedule_tick();
    } else if (sched_timer_count > (sched_timestamp_match + sched_sync_delay)){ //sync late -> out of sync
      error_occurred(ERROR_SCHED_SYNC_LATE, PRIORITY_HIGH);
      sched_status = SCHED_RUNNING_ASYNC;
      sched_timer_count = 0;
      sched_late_count++;
      if (sched_late_count > sched_late_limit){
        error_occurred(ERROR_SCHED_SYNC_CHRONICALLY_LATE, PRIORITY_HIGH);
        //some sort of safe shutdown because probably lost communication to main brain
      }
      schedule_tick();
    }
  } 
  
  // **** CAN ****
  else if (source == SCHED_CAN){ //CAN sync packet generated alert 
    if (sched_status == SCHED_RUNNING_SYNC){ //running in sync with main brain
      if (sched_timer_count < sched_timestamp_match - (sched_tick_match/2)){ //grossly early
        error_occurred(ERROR_SCHED_SYNC_EARLY, PRIORITY_HIGH);
      }
      schedule_tick();
      sched_timer_count = 0;
    } else if (sched_status == SCHED_RUNNING_ASYNC){ //late CAN sync packets
      if (sched_timer_count <= sched_timestamp_match + sched_sync_delay) { //back within sync limit
        sched_status = SCHED_RUNNING_SYNC;
        schedule_tick();
        sched_timer_count = 0;
        sched_late_count = 0;
      } 
    }
  }
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
