//#include <mb_includes.h>
//#include "steering_fsm_conditions.h"
//#include <cstdlib>
//#include <iostream>

//using namespace std;


// define the states
enum states 
{
  STATE_S_innerlegfront, 
  STATE_S_innerlegstance,
  STATE_S_innerlegback, 
  STATE_S_innerswingpremid,
  STATE_S_innerswingaftermid,
  STATE_S_stop
};
// define the conditions
enum sensor 
{
  COND_S_inlegfront_to_inlegstance,
  COND_S_inlegstance_to_inlegback, 
  COND_S_inlegback_to_inswingpremid,
  COND_S_inswingpremid_to_inswingaftermid,
  COND_S_inswingaftermid_to_inlegfront,
  COND_S_stop,
  COND_S_reset
};


void get_steering_input(int* sensor_array, int sensor_array_length)
{
    int num_sensors = sensor_array_length;

   // this function should contain code to convert the global sensor values into 
   // the specific sensory input for the hip_fsm. The sensory states possible
   // are listed in hip_fsm_sensors.h

    for (int i=0;i<num_sensors;i++)      // for the inactive sensors the           
        sensor_array[i] =0;             // the values will be zero


    if (0)
        {}
        
    else
    {

        if ((int) get_io_float(ID_E_DBL_STANCE) == 0 ) // double stance is over
        {
          sensor_array[COND_S_inlegfront_to_inlegstance] = 1;
        }
   
//        if ((get_io_float(ID_MCH_ANGLE)) < get_io_float(ID_P_S_NULL_H_TANG))
        if ((int) get_io_float(ID_E_SWING_LEG) == 1 ) //INNER LEG IS SWING LEG
        {
          sensor_array[COND_S_inlegstance_to_inlegback] = 1;
        }

        if ((int) get_io_float(ID_E_DBL_STANCE) == 0 ) // double stance is over
        {
          sensor_array[COND_S_inlegback_to_inswingpremid] = 1;
        }
        
        if (get_io_float(ID_E_LI_ABSANG) < 0.0)     // inner legs are in swing and pass the vertical position
        {
          sensor_array[COND_S_inswingpremid_to_inswingaftermid] = 1;
        }
 
        if ((int) get_io_float(ID_E_SWING_LEG) == 0 ) //OUTER LEG IS SWING LEG
        {
          sensor_array[COND_S_inswingaftermid_to_inlegfront] = 1;
        }
        
        if ((int) get_io_float(ID_FSM_RESET) == 1)    // reset the fsm to the starting state
        {
          sensor_array[COND_S_reset] = 1;
        }

        if (0)
        {
          sensor_array[COND_S_stop] = 1;
        }
    }

}


/*
// define the states
enum states 
{
  STATE_S_innerlegfront, 
  STATE_S_innerlegstance,
  STATE_S_innerlegback, 
  STATE_S_innerlegswing,  
  STATE_S_stop
};
// define the conditions
enum sensor 
{
  COND_S_inlegfront_to_inlegstance,
  COND_S_inlegstance_to_inlegback, 
  COND_S_inlegback_to_inlegswing,
  COND_S_inlegswing_to_inlegfront,
  COND_S_inlegfront_to_stop,
  COND_S_inlegstance_to_stop,
  COND_S_inlegback_to_stop,
  COND_S_inlegswing_to_stop
};



void get_steering_input(int* sensor_array, int sensor_array_length)
{
    int num_sensors = sensor_array_length;

   // this function should contain code to convert the global sensor values into 
   // the specific sensory input for the hip_fsm. The sensory states possible
   // are listed in hip_fsm_sensors.h

    for (int i=0;i<num_sensors;i++)      // for the inactive sensors the           
      sensor_array[i] =0;             // the values will be zero


 if (0)
    {}
  
 else
 {

  if (
      ((int) get_io_float(ID_E_SWING_LEG) == 0 ) //OUTER LEG IS SWING LEG
     // &&
      //(get_io_float(ID_MCH_ANGLE)) < get_io_float(ID_P_S_NULL_H_TANG)
      )    
  {
    sensor_array[COND_S_inlegfront_to_inlegstance] = 1;
  }
   
  //if ((get_io_float(ID_MCH_ANGLE)) < -get_io_float(ID_P_S_NULL_H_TANG))
  if ((get_io_float(ID_MCH_ANGLE)) < get_io_float(ID_P_S_NULL_H_TANG))
  {
    sensor_array[COND_S_inlegstance_to_inlegback] = 1;
  }

  if (      
      ((int) get_io_float(ID_E_SWING_LEG) == 1 ) //INNER LEG IS SWING LEG
     // &&
     // (get_io_float(ID_MCH_ANGLE)) > -get_io_float(ID_P_S_NULL_H_TANG)
      )
       
  {
    sensor_array[COND_S_inlegback_to_inlegswing] = 1;
  }
 
  //if ((get_io_float(ID_MCH_ANGLE)) > get_io_float(ID_P_S_NULL_H_TANG)) 
  if ((get_io_float(ID_MCH_ANGLE)) > -get_io_float(ID_P_S_NULL_H_TANG))
  {
    sensor_array[COND_S_inlegswing_to_inlegfront] = 1;
  }

  if (0)
  {
    sensor_array[COND_S_inlegfront_to_stop] = 1;
  }
  
  if (0)
  {
    sensor_array[COND_S_inlegstance_to_stop] = 1;
  }

  if (0)
  {
    sensor_array[COND_S_inlegback_to_stop] = 1;
  }

  if (0)
  {
    sensor_array[COND_S_inlegswing_to_stop] = 1;
  }


 }


}
*/
