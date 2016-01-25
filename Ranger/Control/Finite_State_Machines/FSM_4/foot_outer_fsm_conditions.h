#ifndef __FOOT_OUTER_FSM_CONDITIONS_H__
#define __FOOT_OUTER_FSM_CONDITIONS_H__

// define the states
enum states 
{
  STATE_FO_free,
  STATE_FO_stance,  
  STATE_FO_flipup, 
  STATE_FO_stop
};
// define the inputs
enum sensor 
{
  COND_FO_free_to_stance,
  COND_FO_free_to_flipup, 
  COND_FO_stance_to_free,
  COND_FO_flipup_to_free, 
  COND_FO_stop
};

void get_foot_outer_input(int* sensor_array, int sensor_array_length);

#endif   // __FOOT_OUTER_FSM_CONDITIONS_H__

