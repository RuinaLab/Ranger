/*
  @file heartbeat.h
  
  @author Nicolas Williamson
  @see heartbeat.c
*/

#ifndef  TASK_HEARTBEAT_H
#define  TASK_HEARTBEAT_H
  
/* Public Functions */
//intitializes the heartbeat with the given period in units of schedule periods
void hb_init(int period, VOID_VOID_F func, INT_VOID_F get_time);
//called every schedule period
//put into every row of the schedule
void hb_beat(void);
int hb_get_count(void);
static void hb_update(int count);

#endif// TASK_HEARTBEAT_H

