#ifndef __SCHEDULER_H__
#define __SCHEDULER_H__

typedef void(*TASK_PTR)(void);

void mb_schedule_tick(void);
void mb_schedule_init(const TASK_PTR * sched, int tick_divider);
void mb_schedule_run(void);

#endif //__SCHEDULER_H__
