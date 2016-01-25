#include <mb_includes.h>
#include "hip_fsm_conditions.h"
#include "global_sensors.h"
//#include <cstdlib>
//#include <iostream>

//using namespace std;

void get_hip_sensor_input(int* sensor_array, int sensor_array_length)
{
    int num_sensors = sensor_array_length;

   // this function should contain code to convert the global sensor values into 
   // the specific sensory input for the hip_fsm. The sensory states possible
   // are listed in hip_fsm_sensors.h
   
    for (int i=0;i<num_sensors;i++)      // for the inactive sensors the           
      sensor_array[i] =0;             // the values will be zero

   // currently generating random sensor input

// int current_sensor_reading = rand() % num_sensors;
// int current_sensor_reading = 2;
 
 if (g_hip_fsm_stop_command == 1)
//	{current_sensor_reading = stop_command;}
{}
  
 else
 {
  if ((get_io_float(ID_MCH_ANGLE)) > 0.1)
  {
    sensor_array[COND_HIP_inner_swing_to_inner_free] = 1;
  }
  
  if (get_io_float(ID_MCFI_LEFT_HS) + get_io_float(ID_MCFI_RIGHT_HS) >= 1.0)
  {
    sensor_array[COND_HIP_inner_free_to_outer_swing] = 1;
  }

  if ((get_io_float(ID_MCH_ANGLE)) < -0.1)
  {
    sensor_array[COND_HIP_outer_swing_to_outer_free] = 1;
  }
  
  if (get_io_float(ID_MCFO_LEFT_HS) + get_io_float(ID_MCFO_RIGHT_HS) >= 1.0)
  {
    sensor_array[COND_HIP_outer_free_to_inner_swing] = 1;
  }
  
  if (get_io_float(ID_UI_BUTTONS) != 0.0)
  {
    sensor_array[COND_HIP_stop] = 1;
  }
  
 }

// cout << "->hip_fsm: getting sensor input " << current_sensor_reading << endl;




 //  sensor_array[current_sensor_reading] = 1;

}

