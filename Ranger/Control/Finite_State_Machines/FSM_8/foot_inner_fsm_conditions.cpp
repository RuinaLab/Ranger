#include <mb_includes.h>
#include "foot_inner_fsm_conditions.h"
#include "global_sensors.h"
//#include <cstdlib>
//#include <iostream>

//using namespace std;

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
  if ((get_io_float(ID_E_LI_ABSANG)) > get_io_float(ID_P_F_PP_L_ABSANG))    // pushoff initiation condition                                                  
  /*if ( (get_io_float(ID_E_SWING_F_HEIGHT)) <= get_io_float(ID_F_TEST4) 
      &&
       ((get_io_float(ID_E_LI_ABSANG)) > get_io_float(ID_P_F_PP_L_ABSANG))  )  */                                                          
  {
    sensor_array[COND_FI_stance_to_prepush] = 1;
  }

  if ((get_io_float(ID_MCFO_LEFT_HS) + get_io_float(ID_MCFO_RIGHT_HS) >= 1.0) && ((get_io_float(ID_MCH_ANGLE))<-0.05) )    // Emergency transition
  {
    sensor_array[COND_FI_stance_to_afterpush] = 1;
  } 

  if ( (get_io_float(ID_MCFO_LEFT_HS) + get_io_float(ID_MCFO_RIGHT_HS) >= 1.0) &&  ((get_io_float(ID_MCH_ANGLE))<-0.05) ) 
  {
    sensor_array[COND_FI_prepush_to_afterpush] = 1;
  }
  
  if 
     (
      (get_io_float(ID_MCFO_LEFT_HS) + get_io_float(ID_MCFO_RIGHT_HS) >= 1.0)
      && 
      (get_io_float(ID_MCFI_LEFT_HS) + get_io_float(ID_MCFI_RIGHT_HS) <= 0.1)
      //(get_io_float(ID_MCFI_MID_ANKLE_ANGLE) > get_io_float(ID_P_F_AP_F_ANG))
     )
  {
    sensor_array[COND_FI_afterpush_to_flipup] = 1;
  }

  if ( (get_io_float(ID_MCH_ANGLE) > get_io_float(ID_P_F_FU_H_ANG) )
       //&&
       //( (get_io_float(ID_MCH_ANGLE) - 2.0*get_io_float(ID_E_LO_ABSANG)) > 0.00) 
       )
  {
    sensor_array[COND_FI_flipup_to_flipdown] = 1;
  }
  
  if (get_io_float(ID_MCFI_LEFT_HS) + get_io_float(ID_MCFI_RIGHT_HS) >= 1.0)
  {
    sensor_array[COND_FI_flipdown_to_stance] = 1;
  }
  
    if (get_io_float(ID_MCFI_LEFT_HS) + get_io_float(ID_MCFI_RIGHT_HS) >= 1.0) //Start flip down to stance
  {
    sensor_array[COND_FI_startflipdown_to_stance] = 1;
  }
  
   if ( (int) get_io_float(ID_FSM_RESET) == 1 )    // Reset to start stance
  {
    sensor_array[COND_FI_startflipdown] = 1;
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

