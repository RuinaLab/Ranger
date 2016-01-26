//#include <mb_includes.h>
//#include "system_init_fsm_conditions.h"
//#include "global_sensors.h"
//#include <cstdlib>
//#include <iostream>

//using namespace std;
enum states {
             STATE_SI_system_start, 
             STATE_SI_system_run, 
             STATE_SI_stop
             };
             
// define the inputs
enum conditions {
                 COND_SI_all_ready, 
                 COND_SI_stop
                 };


void get_system_init_conditions_input(int* sensor_array, int sensor_array_length)
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
 
 if (0)
//	{current_sensor_reading = stop_command;}
{}
  
 else
 {  
  if (   (get_io_float(ID_MCH_STATUS) == 1)
      && (get_io_float(ID_MCFO_STATUS) == 1)
      && (get_io_float(ID_MCFI_STATUS) == 1)
      && (get_io_float(ID_MCSO_STATUS) == 1)
      && (get_io_float(ID_MCSI_STATUS) == 1)
    //  && (get_io_float(ID_UI_STATUS) == 1)
    )
  {
    sensor_array[COND_SI_all_ready] = 1;
  }
  
  if (1)
  {
    sensor_array[COND_SI_stop] = 0;
  }
  
 }

// cout << "->hip_fsm: getting sensor input " << current_sensor_reading << endl;




 //  sensor_array[current_sensor_reading] = 1;

}

