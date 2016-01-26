#include "ankle_fsm_sensors.h"
#include "global_sensors.h"


void get_ankle_sensor_input(int* sensor_array, int sensor_array_length)
{
   int num_sensors = sensor_array_length;

// this function should contain code to convert the global sensor values into 
// the specific sensory input for the ankle_fsm. The sensory states possible
// are listed in ankle_fsm_sensors.h

// currently generating random sensor input

 //int current_sensor_reading = rand() % num_sensors;
 int current_sensor_reading = 2;

 if (g_ankle_fsm_stop_command == 1) 
    current_sensor_reading = stop_command;

//cout << "->ankle_fsm: getting sensor input " << current_sensor_reading << endl;

 for (int i=0;i<num_sensors;i++) {     
      sensor_array[i] = 0;
 }
      sensor_array[current_sensor_reading] =  1;
      
}


