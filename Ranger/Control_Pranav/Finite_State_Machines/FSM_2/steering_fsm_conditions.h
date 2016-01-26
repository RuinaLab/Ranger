#ifndef __STEERING_FSM_CONDITIONS_H__
#define __STEERING_FSM_CONDITIONS_H__

// define the states
enum states 
{
  STATE_S_innerlegfront, 
  STATE_S_innerlegstance,
  STATE_S_innerlegback, 
  STATE_S_innerlegswing,  
  STATE_S_stop
};
// define the inputs
enum sensor 
{
  COND_S_inlegfront_to_inlegstance,
  COND_S_inlegstance_to_inlegback, 
  COND_S_inlegback_to_inlegswing,
  COND_S_inlegswing_to_inlegfront,
  COND_S_inlegfront_to_stop,
  COND_S_inlegstance_to_stop,
  COND_S_inlegback_to_stop,
  COND_S_inlegswing_to_stop,

};

void get_steering_input(int* sensor_array, int sensor_array_length);

#endif   // __FOOT_INNER_FSM_CONDITIONS_H__

