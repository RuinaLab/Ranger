#ifndef __HIP_FSM_CONDITIONS_H__

#define __HIP_FSM_CONDITIONS_H__

// define the states
//enum states 
enum states 
{
  STATE_HIP_inner_free, 
  STATE_HIP_inner_swing, 
  STATE_HIP_outer_free, 
  STATE_HIP_outer_swing, 
  STATE_HIP_stop
};
// define the inputs
//enum conditions
enum sensor 
{
  COND_HIP_outer_swing_to_outer_free, 
  COND_HIP_outer_free_to_inner_swing, 
  COND_HIP_inner_swing_to_inner_free, 
  COND_HIP_inner_free_to_outer_swing, 
  COND_HIP_stop
};
void get_hip_sensor_input(int* sensor_array, int sensor_array_length);

#endif  //__HIP_FSM_CONDITIONS_H__
