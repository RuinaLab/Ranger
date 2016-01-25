#ifndef __FOOT_INNER_FSM_CONDITIONS_H__
#define __FOOT_INNER_FSM_CONDITIONS_H__

// define the states
enum states 
{
  STATE_FI_free,
  STATE_FI_stance,  
  STATE_FI_flipup, 
  STATE_FI_stop
};
// define the inputs
enum sensor 
{
  COND_FI_free_to_stance,
  COND_FI_free_to_flipup,  
  COND_FI_stance_to_free,
  COND_FI_flipup_to_free,
  COND_FI_stop
};

void get_foot_inner_input(int* sensor_array, int sensor_array_length);

#endif   // __FOOT_INNER_FSM_CONDITIONS_H__

