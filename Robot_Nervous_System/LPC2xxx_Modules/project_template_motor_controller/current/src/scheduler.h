#ifndef __SCHEDULER_H__
#define __SCHEDULER_H__

typedef void(*TASK_PTR)(void);

typedef enum sched_stati {
  SCHED_NOT_RUNNING = 0, //asleep or other
  SCHED_RUNNING_SYNC, //running correctly, executing schedule calls
  SCHED_RUNNING_ASYNC //running out of sync with the main brain
} SCHED_STATUS;

typedef enum sched_flags {
  SCHED_DONE = 0, //Completed the current row
  SCHED_NOT_DONE //has not completed the current row. Error if did not complete previous row.
} SCHED_COMPLETION;

typedef enum sched_sources {
  SCHED_CAN = 0, //timestamp can packet generated tick
  SCHED_TIMER //PWM timer (or other) generated tick
} SCHED_ALERT_SOURCE;

void schedule_tick(void);
void schedule_auxiliary_tick(void);
void schedule_sync(void);
void schedule_init(const TASK_PTR * sched, int tick_divider);//, long long int ticks_per_msec);
void schedule_run(void);
void schedule_alert(SCHED_ALERT_SOURCE source);
void schedule_set_timestamp(int timestamp);
int schedule_get_timestamp(void);
void schedule_isr(void);// __irq;
long long int schedule_get_global_time(void);

#endif //__SCHEDULER_H__
