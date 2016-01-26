#ifndef __FOOT_INNER_FSM_CONDITIONS_H__
#define __FOOT_INNER_FSM_CONDITIONS_H__


// define the states
enum states 
{
  STATE_FI_stance, 
  STATE_FI_prepush,
  STATE_FI_afterpush, 
  STATE_FI_flipup, 
  STATE_FI_flipdown, 
  STATE_FI_stop
};
// define the inputs
enum sensor 
{
  COND_FI_stance_to_prepush,
  COND_FI_stance_to_afterpush,  //Emergency transition
  COND_FI_prepush_to_afterpush,
  COND_FI_afterpush_to_flipup,
  COND_FI_flipup_to_flipdown,
  COND_FI_flipdown_to_stance,
  COND_FI_stop
};

void get_foot_inner_input(int* sensor_array, int sensor_array_length);

#endif   // __FOOT_INNER_FSM_CONDITIONS_H__

