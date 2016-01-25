#include <mb_includes.h>
#include "steering_fsm_conditions.h"
#include "global_sensors.h"
//#include <cstdlib>
//#include <iostream>

//using namespace std;

void get_steering_input(int* sensor_array, int sensor_array_length)
{
    int num_sensors = sensor_array_length;

   // this function should contain code to convert the global sensor values into 
   // the specific sensory input for the hip_fsm. The sensory states possible
   // are listed in hip_fsm_sensors.h
   
    for (int i=0;i<num_sensors;i++)      // for the inactive sensors the           
      sensor_array[i] =0;             // the values will be zero

   // currently generating random sensor input
 
 if (g_steering_fsm_stop_command == 1)
{}
  
 else
 {
  //if ((get_io_float(ID_E_LI_ABSANG)) > get_io_float(ID_P_F_PP_L_ABSANG))    // pushoff initiation condition
  if (
      (get_io_float(ID_MCFI_LEFT_HS) + get_io_float(ID_MCFI_RIGHT_HS) >= 1.0)
      &&
      (get_io_float(ID_MCH_ANGLE)) < get_io_float(ID_P_S_NULL_H_TANG)
      )    
  {
    sensor_array[COND_S_inlegfront_to_inlegstance] = 1;
  }

  //if (get_io_float(ID_MCFO_LEFT_HS) + get_io_float(ID_MCFO_RIGHT_HS) >= 1.0)    
  if ((get_io_float(ID_MCH_ANGLE)) < -get_io_float(ID_P_S_NULL_H_TANG))
  {
    sensor_array[COND_S_inlegstance_to_inlegback] = 1;
  }

  if (      
      (get_io_float(ID_MCFO_LEFT_HS) + get_io_float(ID_MCFO_RIGHT_HS) >= 1.0)
      &&
      (get_io_float(ID_MCH_ANGLE)) > -get_io_float(ID_P_S_NULL_H_TANG)
      )
       
  {
    sensor_array[COND_S_inlegback_to_inlegswing] = 1;
  }

  //  if (get_io_float(ID_MCFO_LEFT_HS) + get_io_float(ID_MCFO_RIGHT_HS) >= 1.0)  
  if ((get_io_float(ID_MCH_ANGLE)) > get_io_float(ID_P_S_NULL_H_TANG)) 
  {
    sensor_array[COND_S_inlegswing_to_inlegfront] = 1;
  }
  

 //if (get_io_float(ID_UI_BUTTONS) != 0.0) //The UI_Button seems to be set to non zero giving exit 4/5/2010
  if (0)
  {
    sensor_array[COND_S_inlegfront_to_stop] = 1;
  }
  
  // if (get_io_float(ID_UI_BUTTONS) != 0.0) //The UI_Button seems to be set to non zero giving exit 4/5/2010
  if (0)
  {
    sensor_array[COND_S_inlegstance_to_stop] = 1;
  }

  // if (get_io_float(ID_UI_BUTTONS) != 0.0) //The UI_Button seems to be set to non zero giving exit 4/5/2010
  if (0)
  {
    sensor_array[COND_S_inlegback_to_stop] = 1;
  }

  // if (get_io_float(ID_UI_BUTTONS) != 0.0) //The UI_Button seems to be set to non zero giving exit 4/5/2010
  if (0)
  {
    sensor_array[COND_S_inlegswing_to_stop] = 1;
  }


 }

// cout << "->hip_fsm: getting sensor input " << current_sensor_reading << endl;




 //  sensor_array[current_sensor_reading] = 1;

}

