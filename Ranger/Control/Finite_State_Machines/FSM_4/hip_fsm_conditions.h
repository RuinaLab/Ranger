#ifndef __HIP_FSM_CONDITIONS_H__

#define __HIP_FSM_CONDITIONS_H__

// define the states
//enum states 
enum states 
{
  STATE_H_free, 
  STATE_HI_ehold,
  STATE_HI_swing, 
  STATE_HO_ehold, 
  STATE_HO_swing,
  STATE_H_stop,
};
// define the inputs
//enum conditions
enum sensor 
{
  COND_H_free_to_HI_ehold,    //Start with leg at some hold angle
  COND_HI_ehold_to_HI_swing, 
  COND_HI_swing_to_HI_free, 
  COND_H_free_to_HO_ehold, 
  COND_HI_free_to_HI_swing,
  COND_HO_ehold_to_HO_swing,
  COND_HO_swing_to_HO_free,
  COND_HI_ehold_to_H_free,
  COND_H_stop
};
void get_hip_sensor_input(int* sensor_array, int sensor_array_length);

#endif  //__HIP_FSM_CONDITIONS_H__
