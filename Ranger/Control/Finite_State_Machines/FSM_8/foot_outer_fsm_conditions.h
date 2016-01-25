#ifndef __FOOT_OUTER_FSM_CONDITIONS_H__
#define __FOOT_OUTER_FSM_CONDITIONS_H__

// define the states
enum states 
{
  STATE_FO_stance, 
  STATE_FO_prepush,
  STATE_FO_afterpush, 
  STATE_FO_flipup, 
  STATE_FO_flipdown, 
  STATE_FO_stop,
  STATE_FO_startstance  // Starting state
};
// define the inputs
enum sensor 
{
  COND_FO_stance_to_prepush,
  COND_FO_stance_to_afterpush,    // Emergency transition
  COND_FO_prepush_to_afterpush,
  COND_FO_afterpush_to_flipup,
  COND_FO_flipup_to_flipdown,
  COND_FO_flipdown_to_stance,
  COND_FO_stop,
  COND_FO_startstance_to_afterpush, // Transition out of starting state at heelstrike
  COND_FO_startstance //Get to start stance on reset
};

void get_foot_outer_input(int* sensor_array, int sensor_array_length);

#endif   // __FOOT_OUTER_FSM_CONDITIONS_H__

