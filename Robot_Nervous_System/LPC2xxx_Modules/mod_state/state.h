/*

	state.h
	
	Nicolas Williamson - January 2010
	
*/

#ifndef __STATE_H__
#define __STATE_H__

typedef enum st_states{
  ST_STARTING = 0,
  ST_WAITING = 1,
  ST_RUNNING = 2,
  ST_SLEEPING = 3,
  ST_EXITING = 4
} ST_STATE;

void st_init(const TASK_PTR *wait_sched, const TASK_PTR *run_sched, const TASK_PTR *exit_sched);
ST_STATE st_get_state(void);
int st_is_on(void);
void st_exit(void);
void st_wait(void);
void st_run(void);
void st_off(void);
void st_main(void);
void st_run_waiting(void);
void st_run_running(void);
void st_run_exiting(void);
float st_get_status(void); //returns status in a form useable by main brain and CAN

#endif
