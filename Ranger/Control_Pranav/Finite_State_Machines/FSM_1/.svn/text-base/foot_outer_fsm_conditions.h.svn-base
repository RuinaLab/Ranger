#ifndef __FOOT_OUTER_FSM_CONDITIONS_H__
#define __FOOT_OUTER_FSM_CONDITIONS_H__

// define the states
enum states 
{
  STATE_FOOT_OUTER_stance, 
  STATE_FOOT_OUTER_pushoff, 
  STATE_FOOT_OUTER_flipup, 
  STATE_FOOT_OUTER_flipdown, 
  STATE_FOOT_OUTER_stop
};
// define the inputs
enum sensor 
{
  COND_FOOT_OUTER_stance_to_pushoff,
  COND_FOOT_OUTER_pushoff_to_flipup,
  COND_FOOT_OUTER_flipup_to_flipdown,
  COND_FOOT_OUTER_flipdown_to_stance,
  COND_FOOT_OUTER_stop
};

void get_foot_outer_input(int* sensor_array, int sensor_array_length);

#endif   // __FOOT_OUTER_FSM_CONDITIONS_H__

