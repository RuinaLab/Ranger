
// define the states in the state machine
enum states 
{
  STATE_FO_stance, 
  STATE_FO_prepush,
  STATE_FO_afterpush, 
  STATE_FO_flipup, 
  STATE_FO_flipdown, 
  STATE_FO_stop,
  STATE_FO_startstance  // Starting state
};

// define the conditions in the state machine
enum sensor 
{
  COND_FO_stance_to_prepush,
  COND_FO_stance_to_afterpush,    // Emergency transition
  COND_FO_prepush_to_afterpush,
  COND_FO_afterpush_to_flipup,
  COND_FO_flipup_to_flipdown,
  COND_FO_flipdown_to_stance,
  COND_FO_stop,
  COND_FO_startstance_to_afterpush, // Transition out of starting state at heelstrike
  COND_FO_startstance //Get to start stance on reset
};


void get_foot_outer_input(int* sensor_array, int sensor_array_length)
{
    int num_sensors = sensor_array_length;
   
    for (int i=0;i<num_sensors;i++)      // for the inactive sensors the           
      sensor_array[i] =0;             // the values will be zero

if (0) //Do not go to stop
{}
  
 else
 {
 if ( get_io_float(ID_E_LO_ABSANG) > get_io_float(ID_P_F_PP_L_ABSANG) )    // pushoff initiation condition                                                                       
  {
    sensor_array[COND_FO_stance_to_prepush] = 1;
  }

  if ( (get_io_float(ID_MCFI_LEFT_HS) + get_io_float(ID_MCFI_RIGHT_HS) >= 1.0)  && ((get_io_float(ID_MCH_ANGLE))>0.05) )    // Emergency transition
  {
    sensor_array[COND_FO_stance_to_afterpush] = 1;
  } 

  if ( (get_io_float(ID_MCFI_LEFT_HS) + get_io_float(ID_MCFI_RIGHT_HS) >= 1.0) && ((get_io_float(ID_MCH_ANGLE))>0.05) ) 
  {
    sensor_array[COND_FO_prepush_to_afterpush] = 1;
  }
  
  if (1) //skip afterpush
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
  
    if (get_io_float(ID_MCFI_LEFT_HS) + get_io_float(ID_MCFI_RIGHT_HS) >= 1.0)    // Start transition
  {
    sensor_array[COND_FO_startstance_to_afterpush] = 1;
  }  
  
  if ( (int) get_io_float(ID_FSM_RESET) == 1 )    // Reset to start stance
  {
    sensor_array[COND_FO_startstance] = 1;
  }  

   if (0)
  {
        sensor_array[COND_FO_stop] = 1;
  }


 }

}

