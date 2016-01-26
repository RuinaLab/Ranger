
#ifndef _TOP_FSM_SENSORS_H

#define _TOP_FSM_SENSORS_H

// define the states
enum states {walking, stopping, ready_to_exit};
// define the inputs
enum sensor {walk_command, stop_command};

void get_top_sensor_input(int* sensor_array, int sensor_array_length);

#endif
