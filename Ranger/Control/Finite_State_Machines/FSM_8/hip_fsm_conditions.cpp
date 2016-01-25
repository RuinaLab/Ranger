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
 if (g_hip_fsm_stop_command == 1)
//	{current_sensor_reading = stop_command;}
{}
  
 else
 {
  if (get_io_float(ID_MCFI_LEFT_HS) + get_io_float(ID_MCFI_RIGHT_HS) >= 1.0)
  {
    sensor_array[COND_HI_starthold_to_HO_preswing] = 1;
  }

  if (1)  // For now, skip preswing state
  {
    sensor_array[COND_HO_preswing_to_HO_premid] = 1;
  }  
 

 
  if ((get_io_float(ID_E_LI_ABSANG)) > 0.0) 
  {
    sensor_array[COND_HO_premid_to_HO_aftermid] = 1;
  }
  
  if (get_io_float(ID_MCFO_LEFT_HS) + get_io_float(ID_MCFO_RIGHT_HS) >= 1.0)
  {
    sensor_array[COND_HO_aftermid_to_HI_preswing] = 1;
  }

//  if (((get_io_float(ID_MCH_ANGLE)) > get_io_float(ID_P_H_EHOLD_ANG)) //logical might need to be reversed and sign appened for postive E_hold_ang
//       && 
//     ((get_io_float(ID_E_LI_ABSANG)) < get_io_float(ID_MCH_ANGLE)* -0.4)) // We want the stance angle to always be less than half the hip angle
//                                                                          // by some finite amount, at least while the hip angle is less than 
//                                                                          // some minimum step size (e.g., 0.15 or 0.2 radians).
//                                                                          // The robot will fall if this condition is not satisfied, either
//                                                                          // by swinging too slowly (scuffing, say) or swinging too far back.
//                                                                          // In either case we propose here to drive the leg with all due haste
//                                                                          // to a reasonable step size when we approach this condition.
//                                                                          // We may eventually want more sophisticated sensing of the approach of this
//                                                                          // condition that includes also the relevant angular velocities.
  //if ((get_io_float(ID_MCH_ANGLE)) < -get_io_float(ID_P_H_EHOLD_ANG)) //Hold angle when swinging forward
     if (
        (get_io_float(ID_MCH_ANGLE) > -get_io_float(ID_P_H_EHOLD_ANG)) //If less than a preset value
        && 
        ((get_io_float(ID_E_H_RATE))>0.0)  //If swinging back
        &&
        (get_io_float(ID_MCH_ANGLE) < 0.0 ) //Only after mid-stance
        ) 
  {
    sensor_array[COND_HO_aftermid_to_HO_ehold] = 1;
    sensor_array[COND_HO_premid_to_HO_ehold] = 1;
  }

  if (get_io_float(ID_MCFO_LEFT_HS) + get_io_float(ID_MCFO_RIGHT_HS) >= 1.0)
  {
    sensor_array[COND_HO_ehold_to_HI_preswing] = 1;
  }

  if (1)  // For now, skip preswing state
  {
    sensor_array[COND_HI_preswing_to_HI_premid] = 1;
  }
  
  if ((get_io_float(ID_E_LO_ABSANG)) > 0.0) 
  {
    sensor_array[COND_HI_premid_to_HI_aftermid] = 1;
  }

  if (get_io_float(ID_MCFI_LEFT_HS) + get_io_float(ID_MCFI_RIGHT_HS) >= 1.0)
  {
    sensor_array[COND_HI_aftermid_to_HO_preswing] = 1;
  }

//  if (((get_io_float(ID_MCH_ANGLE)) < get_io_float(ID_P_H_EHOLD_ANG)) //logical might need to be reversed and sign appened for postive E_hold_ang
//       && 
//     ((get_io_float(ID_E_LO_ABSANG)) < get_io_float(ID_MCH_ANGLE)* 0.4)) // We want the stance angle to always be less than half the hip angle
//                                                                          // by some finite amount, at least while the hip angle is less than 
//                                                                          // some minimum step size (e.g., 0.15 or 0.2 radians).
//                                                                          // The robot will fall if this condition is not satisfied, either
//                                                                          // by swinging too slowly (scuffing, say) or swinging too far back.
//                                                                          // In either case we propose here to drive the leg with all due haste
//                                                                          // to a reasonable step size when we approach this condition.
//                                                                          // We may eventually want more sophisticated sensing of the approach of this
//                                                                          // condition that includes also the relevant angular velocities.
 // if ((get_io_float(ID_MCH_ANGLE)) > get_io_float(ID_P_H_EHOLD_ANG)) //Hold angle when swinging forward
   if (
        (get_io_float(ID_MCH_ANGLE) < get_io_float(ID_P_H_EHOLD_ANG)) //If less than a preset value
        && 
        ((get_io_float(ID_E_H_RATE))<0.0)  //If swinging back
        &&
        (get_io_float(ID_MCH_ANGLE) > 0.0 ) //Only after mid-stance
        ) 
  {
    sensor_array[COND_HI_aftermid_to_HI_ehold] = 1;
    sensor_array[COND_HI_premid_to_HI_ehold] = 1;
  }

  if (get_io_float(ID_MCFI_LEFT_HS) + get_io_float(ID_MCFI_RIGHT_HS) >= 1.0)
  {
    sensor_array[COND_HI_ehold_to_HO_preswing] = 1;
  }

   if ( (int) get_io_float(ID_FSM_RESET) == 1 )    // Reset to start stance
  {
    sensor_array[COND_HI_starthold] = 1;
  }  
  
 // if (get_io_float(ID_UI_BUTTONS) != 0.0) //The UI_Button seems to be set to non zero giving exit 4/5/2010
   if (0)
  {
    sensor_array[COND_H_stop] = 1;
  }
  
 }

// cout << "->hip_fsm: getting sensor input " << current_sensor_reading << endl;




 //  sensor_array[current_sensor_reading] = 1;

}

