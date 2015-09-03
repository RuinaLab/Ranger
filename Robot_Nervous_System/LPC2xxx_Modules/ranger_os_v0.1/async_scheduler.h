/*
	@file async_scheduler.h
*/

#ifndef __ASYNC_SCHEDULER_H__
#define __ASYNC_SCHEDULER_H__

/**
  Represents the state of the scheduler.
*/
typedef enum asched_flags {
  ASCHED_DONE = 0, /**< Completed the current row. */
  ASCHED_NOT_DONE /**< Has not completed the current row. */
} ASCHED_COMPLETION;

void asched_tick(void);
void asched_init(const VOID_VOID_F * sched, int tick_divider);
void asched_run(void);
void asched_set_timestamp(int timestamp);
int asched_get_timestamp(void);

#endif //__ASYNC_SCHEDULER_H__

