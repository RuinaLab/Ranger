#ifndef __HIP_FSM_CONDITIONS_H__

#define __HIP_FSM_CONDITIONS_H__

// define the states
//enum states 
enum states 
{
  STATE_HI_starthold, // Starting hold state
  STATE_HI_preswing,
  STATE_HI_premid, 
  STATE_HI_aftermid, 
  STATE_HO_preswing,
  STATE_HO_premid, 
  STATE_HO_aftermid, 
  STATE_H_stop,
  STATE_HI_ehold,    // Inner swing leg emergency hold state (don't want leg to swing too far back)
  STATE_HO_ehold     // Outer swing leg emergency hold state (don't want leg to swing too far back)
};
// define the inputs
//enum conditions
enum sensor 
{
  COND_HI_starthold_to_HO_preswing,    //Start with leg at some hold angle
  COND_HO_preswing_to_HO_premid, 
  COND_HO_premid_to_HO_aftermid, 
  COND_HO_aftermid_to_HI_preswing,
  COND_HO_aftermid_to_HO_ehold,      // Detect case in which outer leg angle is too small for the given stance angle
  COND_HO_ehold_to_HI_preswing, 
  COND_HI_preswing_to_HI_premid, 
  COND_HI_premid_to_HI_aftermid,
  COND_HI_aftermid_to_HO_preswing,
  COND_HI_aftermid_to_HI_ehold,      // Detect case in which inner leg angle is too small for the given stance angle
  COND_HI_ehold_to_HO_preswing, 
  COND_H_stop
};
void get_hip_sensor_input(int* sensor_array, int sensor_array_length);

#endif  //__HIP_FSM_CONDITIONS_H__
