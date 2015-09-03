/*
	async_scheduler.h
	
	Simpler asynchronous scheduler - Tommy's Original
*/

#ifndef __ASYNC_SCHEDULER_H__
#define __ASYNC_SCHEDULER_H__

typedef void(*TASK_PTR)(void);

typedef enum asched_flags {
  ASCHED_DONE = 0, //Completed the current row
  ASCHED_NOT_DONE //has not completed the current row. Error if did not complete previous row.
} ASCHED_COMPLETION;

void asched_tick(void);
void asched_init(const TASK_PTR * sched, int tick_divider);
void asched_run(void);
void asched_set_timestamp(int timestamp);
int asched_get_timestamp(void);

#endif //__ASYNC_SCHEDULER_H__

