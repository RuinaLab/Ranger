#include <mb_includes.h>
#include "foot_inner_fsm_conditions.h"
#include "global_sensors.h"
//#include <cstdlib>
//#include <iostream>

//using namespace std;

extern float fi_counts_flipup, fi_counts_flipdown, fi_counts_stance;
const static float fi_total_counts_flipup = 300, fi_total_counts_flipdown = 400, fi_total_counts_stance = 700;
//const static float fi_total_counts_flipup = 500, fi_total_counts_flipdown = 700, fi_total_counts_stance = 200;

void get_foot_inner_input(int* sensor_array, int sensor_array_length)
{
    int num_sensors = sensor_array_length;

   // this function should contain code to convert the global sensor values into 
   // the specific sensory input for the hip_fsm. The sensory states possible
   // are listed in hip_fsm_sensors.h
   
    for (int i=0;i<num_sensors;i++)      // for the inactive sensors the           
      sensor_array[i] =0;             // the values will be zero

   // currently generating random sensor input
 
 if (g_foot_inner_fsm_stop_command == 1)
//	{current_sensor_reading = stop_command;}
{}
  
 else
 {

  //if (fi_counts_stance >= fi_total_counts_stance)    
    if(0)                                                                 
  {
    sensor_array[COND_FI_stance_to_flipup] = 1;
  }


  if (fi_counts_flipup >= fi_total_counts_flipup)
  {
    sensor_array[COND_FI_flipup_to_flipdown] = 1;
  }
  
  if (fi_counts_flipdown >= fi_total_counts_flipdown)
  {
    sensor_array[COND_FI_flipdown_to_stance] = 1;
  }
  
 // if (get_io_float(ID_UI_BUTTONS) != 0.0) //The UI_Button seems to be set to non zero giving exit 4/5/2010
   if (0)
  {
    sensor_array[COND_FI_stop] = 1;
  }
  
 }

// cout << "->hip_fsm: getting sensor input " << current_sensor_reading << endl;




 //  sensor_array[current_sensor_reading] = 1;

}

