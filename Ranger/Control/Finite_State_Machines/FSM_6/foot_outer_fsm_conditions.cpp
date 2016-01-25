#include <mb_includes.h>
#include "foot_outer_fsm_conditions.h"
#include "global_sensors.h"
//#include <cstdlib>
//#include <iostream>

//using namespace std;
extern float fo_counts_flipup, fo_counts_flipdown, fo_counts_stance;
const static float fo_total_counts_flipup = 300, fo_total_counts_flipdown = 400, fo_total_counts_stance = 700;
//const static float fo_total_counts_flipup = 500, fo_total_counts_flipdown = 700, fo_total_counts_stance = 200;

void get_foot_outer_input(int* sensor_array, int sensor_array_length)
{
    int num_sensors = sensor_array_length;

   // this function should contain code to convert the global sensor values into 
   // the specific sensory input for the hip_fsm. The sensory states possible
   // are listed in hip_fsm_sensors.h
   
    for (int i=0;i<num_sensors;i++)      // for the inactive sensors the           
      sensor_array[i] =0;             // the values will be zero

   // currently generating random sensor input
 
 if (g_foot_outer_fsm_stop_command == 1)
//	{current_sensor_reading = stop_command;}
{}
  
 else
 {

 //if (fo_counts_stance >= fo_total_counts_stance)   
 if(0)                                                               
  {
    sensor_array[COND_FO_stance_to_flipup] = 1;
  }

  if (fo_counts_flipup >= fo_total_counts_flipup)
  {
    sensor_array[COND_FO_flipup_to_flipdown] = 1;
  }
  
  if (fo_counts_flipdown >= fo_total_counts_flipdown)
  {
    sensor_array[COND_FO_flipdown_to_stance] = 1;
  }  
  
 // if (get_io_float(ID_UI_BUTTONS) != 0.0) //The UI_Button seems to be set to non zero giving exit 4/5/2010
   if (0)
  {
        sensor_array[COND_FO_stop] = 1;
  }


 }

// cout << "->hip_fsm: getting sensor input " << current_sensor_reading << endl;




 //  sensor_array[current_sensor_reading] = 1;

}

