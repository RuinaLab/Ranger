#include <mb_includes.h>
#include "foot_outer_fsm_conditions.h"
#include "global_sensors.h"
//#include <cstdlib>
//#include <iostream>

//static float discrete_fo_prepush_d_tang; //reduced the push off by this amount if missed prepush
//using namespace std;

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
 //if ( get_io_float(ID_E_LO_ABSANG) > (get_io_float(ID_P_F_PP_L_ABSANG)-discrete_fo_prepush_d_tang) )    // pushoff initiation condition
 if ( get_io_float(ID_E_LO_ABSANG) > get_io_float(ID_P_F_PP_L_ABSANG) )    // pushoff initiation condition
 //if ((get_io_float(ID_MCH_ANGLE)) > get_io_float(ID_P_F_S_H_ANG))    // Based on hip angle temporary, 4/5/2010, 
                                                                     //Maybe, > pos (pos ID_P_F_S_H_ANG) (5/10/2010)
 /* if ( (get_io_float(ID_MCH_ANGLE) > 0.05)  //Inner leg in front
        &&
        (get_io_float(ID_E_H_RATE) <  get_io_float(ID_F_TEST3) ))  //Hip swinging back (negative)  */                                                                    
/*if ( (get_io_float(ID_E_SWING_F_HEIGHT)) <= get_io_float(ID_F_TEST4) 
      &&
       (get_io_float(ID_E_LO_ABSANG)) > get_io_float(ID_P_F_PP_L_ABSANG) ) //inner foot is in front by set amount     */                                                         
  {
    //discrete_fo_prepush_d_tang = 0.0; //set to zero once prepush is triggered
    sensor_array[COND_FO_stance_to_prepush] = 1;
  }

  if ( (get_io_float(ID_MCFI_LEFT_HS) + get_io_float(ID_MCFI_RIGHT_HS) >= 1.0)  && ((get_io_float(ID_MCH_ANGLE))>0.05) )    // Emergency transition
  {
    //discrete_fo_prepush_d_tang = get_io_float(ID_F_TEST4); //set to value for earlier prepush next step
    sensor_array[COND_FO_stance_to_afterpush] = 1;
  } 

  if (get_io_float(ID_MCFI_LEFT_HS) + get_io_float(ID_MCFI_RIGHT_HS) >= 1.0) 
  {
    sensor_array[COND_FO_prepush_to_afterpush] = 1;
  }
  
  if 
     (
      (get_io_float(ID_MCFI_LEFT_HS) + get_io_float(ID_MCFI_RIGHT_HS) >= 1.0)
      && 
      (get_io_float(ID_MCFO_LEFT_HS) + get_io_float(ID_MCFO_RIGHT_HS) <= 0.1)
     )
  {
    sensor_array[COND_FO_afterpush_to_flipup] = 1;
  }

  if ((get_io_float(ID_MCH_ANGLE)) < - get_io_float(ID_P_F_FU_H_ANG))
  {
    sensor_array[COND_FO_flipup_to_flipdown] = 1;
  }
  
  if (get_io_float(ID_MCFO_LEFT_HS) + get_io_float(ID_MCFO_RIGHT_HS) >= 1.0)
  {
    sensor_array[COND_FO_flipdown_to_stance] = 1;
  }  
  
 // if (get_io_float(ID_UI_BUTTONS) != 0.0) //The UI_Button seems to be set to non zero giving exit 4/5/2010
   if (0)
  {
        sensor_array[COND_FO_stop] = 1;
  }

  if (get_io_float(ID_MCFI_LEFT_HS) + get_io_float(ID_MCFI_RIGHT_HS) >= 1.0)    // Start transition
  {
    sensor_array[COND_FO_startstance_to_afterpush] = 1;
  }  

 }

// cout << "->hip_fsm: getting sensor input " << current_sensor_reading << endl;




 //  sensor_array[current_sensor_reading] = 1;

}

