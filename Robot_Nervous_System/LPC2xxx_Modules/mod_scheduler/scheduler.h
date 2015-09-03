#ifndef __SCHEDULER_H__
#define __SCHEDULER_H__

typedef void(*TASK_PTR)(void);
typedef void(*SCHED_TIMER_RESET)(void); //function which resets the timer to be in sync with mb timestamp packets

typedef enum sched_stati { //stati = plural of status??
  SCHED_NOT_RUNNING = 0, //asleep or other
  SCHED_RUNNING_SYNC, //running correctly, executing schedule calls
  SCHED_RUNNING_ASYNC //running out of sync with the main brain
} SCHED_STATUS;

typedef enum sched_flags {
  SCHED_DONE = 0, //Completed the current row
  SCHED_NOT_DONE //has not completed the current row. Error if did not complete previous row.
} SCHED_COMPLETION;

void sched_tick(void);
void sched_auxiliary_tick(void);
void sched_sync(void);
void sched_init(const TASK_PTR * sched, int tick_divider, int sync_rate, SCHED_TIMER_RESET sync_reset, SCHED_TIMER_RESET aux_reset);//, long long int ticks_per_msec);
void sched_reset(const TASK_PTR * sched);
void sched_run(void);
void sched_set_timestamp(int timestamp);
int sched_get_timestamp(void);

SCHED_STATUS sched_get_status(void);

#endif //__SCHEDULER_H__
