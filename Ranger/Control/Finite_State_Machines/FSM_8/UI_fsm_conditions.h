#ifndef __UI_FSM_CONDITIONS_H__

#define __UI_FSM_CONDITIONS_H__

// define the states
enum states {STATE_UI_calibrate, STATE_UI_standby, STATE_UI_walk, STATE_UI_stop};
// define the inputs
enum conditions { COND_UI_standby_to_calibrate, COND_UI_standby_to_walk,
                  COND_UI_walk_to_standby, COND_UI_calibrate_to_walk, COND_UI_stop};
void get_UI_conditions_input(int* sensor_array, int sensor_array_length);

bool detect_UI_button_input(int button_num);

#endif  //__UI_FSM_CONDITIONS_H__
