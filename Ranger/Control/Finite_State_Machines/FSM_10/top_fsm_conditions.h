//#include <mb_includes.h>
//#include "UI_fsm_conditions.h"
//#include <cstdlib>
//#include <iostream>

//using namespace std;

// define the states
enum states {STATE_top_walk_normal, 
             STATE_top_walk_slow, 
             STATE_top_walk_fast, 
             STATE_top_walk_stop,
             STATE_top_walk_start,
             STATE_top_temp,
             STATE_top_stop
             };
             
// define the inputs
enum conditions { COND_top_walk_normal_to_walk_stop,
                  COND_top_walk_normal_to_walk_fast,
                  COND_top_walk_normal_to_walk_slow,
                  COND_top_walk_stop_to_walk_normal,
                  COND_top_walk_stop_to_walk_start,
                  COND_top_walk_start_to_walk_normal,
                  COND_top_walk_normal_to_temp,
                  COND_top_temp_to_walk_stop,
                  COND_top_temp_to_walk_slow,
                  COND_top_walk_normal, 
                  COND_top_stop
                };



void get_top_conditions_input(int* sensor_array, int sensor_array_length)
{
    int num_sensors = sensor_array_length;

    for (int i=0;i<num_sensors;i++)      // for the inactive sensors the           
      sensor_array[i] =0;             // the values will be zero

 if (0)
  {}
  
 else
 {  
  if (get_io_float(ID_A_T_RC1_NORM) < 0.05 )  
  {
    //sensor_array[COND_top_walk_normal_to_walk_stop] = 1;
    sensor_array[COND_top_temp_to_walk_stop] = 1;
  }
  

  
  /*if (get_io_float(ID_A_T_RC1_NORM) > 0.95 )  
  {
    sensor_array[COND_top_walk_stop_to_walk_start] = 1;
  }*/
  
   if ( (get_io_float(ID_A_T_RC1_NORM) > 0.1)
        &&
       (get_io_float(ID_A_T_RC1_NORM) < 0.35)  ) 
  {
    sensor_array[COND_top_walk_normal_to_temp] = 1;
  } 
  

 if ( (get_io_float(ID_A_T_RC1_NORM) > 0.6)
        &&
       (get_io_float(ID_A_T_RC1_NORM) < 0.9)  ) 
  {
    sensor_array[COND_top_walk_normal_to_walk_fast] = 1;
  } 
  
    if ( (get_io_float(ID_A_T_RC1_NORM) > 0.6)
        &&
       (get_io_float(ID_A_T_RC1_NORM) < 0.9)  ) 
  {
    sensor_array[COND_top_temp_to_walk_slow] = 1;
  } 


  if ( (get_io_float(ID_A_T_RC1_NORM) > 0.4)
        &&
       (get_io_float(ID_A_T_RC1_NORM) < 0.6)  ) 
  {
    sensor_array[COND_top_walk_normal] = 1;
  } 
  
  if ( (int) get_io_float(ID_FSM_RESET) == 1 )    // Reset to walk normal
  {
    sensor_array[COND_top_walk_stop_to_walk_normal] = 1;
    sensor_array[COND_top_walk_start_to_walk_normal] = 1; //This also needs to be initialized once robot is ready to transition from stop walking.
  } 
  
  if (0)
  {
    sensor_array[COND_top_stop] = 0;
  }
  
 }

}

