extern float fo_prepush_angle_turned;
extern float discrete_fo_prepush_angle;
extern float fo_prepush_dangle;
extern float fo_afterpush_time;
extern float fo_prepush_angle;
extern int fo_flag_skip_prepush;

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
  COND_FO_stance_to_afterpush,    //Removed, Emergency transition
  COND_FO_prepush_to_afterpush,
  COND_FO_afterpush_to_flipup,
  COND_FO_flipup_to_flipdown,
  COND_FO_flipdown_to_stance,
  COND_FO_stop,
  COND_FO_startstance_to_prepush, // Transition out of starting state at heelstrike. CHANGED from afterpush to prepush 2/21/2011
  COND_FO_startstance_to_flipup, // Transition out of starting state at heelstrike. CHANGED from afterpush to prepush 3/11/2011
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

if (
     ( (get_io_float(ID_E_ANK2ANK_HT) < get_io_float(ID_P_FO_PP_THEIGHT))
            && ( get_io_float(ID_E_FI_ABSANG) > get_io_float(ID_P_F_PP_F_ABSANG) ) 
            && ( get_io_float(ID_MCH_ANGLE) > 0.05 ) )   
     ||
       (  ((int) get_io_float(ID_E_SWING_LEG) == 0)
             && (get_io_float(ID_MCH_ANGLE) >0.05 ) ) 
    )
 {
    sensor_array[COND_FO_stance_to_prepush] = 1;
  }
  

  if(0) //skip this transition, stance to afterpush not allowed
  {
    sensor_array[COND_FO_stance_to_afterpush] = 1;
  } 

  if ( 
        (
        (fo_prepush_angle_turned > get_io_float(ID_P_F_PP_FRAC_ANG)*(fo_prepush_angle+discrete_fo_prepush_angle+fo_prepush_dangle))
                ||
         (fo_flag_skip_prepush == 1)
        )        
     &&
     ((int) get_io_float(ID_E_SWING_LEG) == 0)
     )
  {
    sensor_array[COND_FO_prepush_to_afterpush] = 1;
  }
  
 if (
     ((get_io_float(ID_MCFO_MOTOR_POSITION) - get_io_float(ID_MCFO_RIGHT_ANKLE_ANGLE))
                <= get_io_float(ID_P_F_FU_F_CABL_STRETCH))
                ||
       (fo_afterpush_time >= get_io_float(ID_P_F_AP_TIME))                        
   )
  {
    sensor_array[COND_FO_afterpush_to_flipup] = 1;
  }

  if ((get_io_float(ID_MCH_ANGLE)) < - get_io_float(ID_P_F_FU_H_ANG))
  {
    sensor_array[COND_FO_flipup_to_flipdown] = 1;
  }
  
   if ( (int) get_io_float(ID_E_SWING_LEG) == 1)
   {
    sensor_array[COND_FO_flipdown_to_stance] = 1;
  }  
  

  if ( (int) get_io_float(ID_E_SWING_LEG) == 0)
  {
    //sensor_array[COND_FO_startstance_to_prepush] = 1; 
    sensor_array[COND_FO_startstance_to_flipup] = 1; 
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

