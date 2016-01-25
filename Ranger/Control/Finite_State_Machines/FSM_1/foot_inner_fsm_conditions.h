#ifndef __FOOT_INNER_FSM_CONDITIONS_H__
#define __FOOT_INNER_FSM_CONDITIONS_H__

// define the states
enum states 
{
  STATE_FOOT_INNER_stance, 
  STATE_FOOT_INNER_pushoff, 
  STATE_FOOT_INNER_flipup, 
  STATE_FOOT_INNER_flipdown, 
  STATE_FOOT_INNER_stop
};
// define the inputs
enum sensor 
{
  COND_FOOT_INNER_stance_to_pushoff,
  COND_FOOT_INNER_pushoff_to_flipup,
  COND_FOOT_INNER_flipup_to_flipdown,
  COND_FOOT_INNER_flipdown_to_stance,
  COND_FOOT_INNER_stop
};

void get_foot_inner_input(int* sensor_array, int sensor_array_length);

#endif   // __FOOT_INNER_FSM_CONDITIONS_H__

