#include "top_fsm_sensors.h"
#include "global_sensors.h"
#include "global_params.h"
//#include <cstdlib>
//#include <iostream>

//using namespace std;


void get_top_sensor_input(int* sensor_array, int sensor_array_length)
{
int current_sensor_reading; 
int num_sensors = sensor_array_length;

// this function should contain code to convert the global sensory 
// information into input for the top_fsm.

// currently generating random sensor information

// temporary code 
//int temp = rand() % 100;
int temp = 5;

if (temp < 95) current_sensor_reading = 0;
else {       
       current_sensor_reading = 1;
       g_top_fsm_stop_command = 1;
      }

// good code
 
    if (g_top_fsm_stop_command == 1) {
       current_sensor_reading = stop_command;
    }

 //cout << "->top_fsm: getting sensor input " << current_sensor_reading << endl;
 

 for (int i=0;i<num_sensors;i++)      // for the inactive sensors the 
   sensor_array[i] = 0;                // values will be 0

   sensor_array[current_sensor_reading] = 1; // for the active sensors
                                             // the value will be 1

}


