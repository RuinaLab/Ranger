extern float fi_prepush_angle_turned;
extern float discrete_fi_prepush_angle;
extern float fi_prepush_dangle;
extern float fi_afterpush_time;
extern float fi_prepush_angle;
extern int fi_flag_skip_prepush;

enum states 
{
  STATE_FI_stance, 
  STATE_FI_prepush,
  STATE_FI_afterpush, 
  STATE_FI_flipup, 
  STATE_FI_flipdown, 
  STATE_FI_stop,
  STATE_FI_startflipdown  // Starting state
};

// define the inputs
enum sensor 
{
  COND_FI_stance_to_prepush,
  COND_FI_stance_to_afterpush,  //Removed, Emergency transition
  COND_FI_prepush_to_afterpush,
  COND_FI_afterpush_to_flipup,
  COND_FI_flipup_to_flipdown,
  COND_FI_flipdown_to_stance,
  COND_FI_stop,
  COND_FI_startflipdown_to_stance, // Transition out of starting state at heelstrike
  COND_FI_startflipdown  //Reset state machine
};


void get_foot_inner_input(int* sensor_array, int sensor_array_length)
{
    int num_sensors = sensor_array_length;


   
    for (int i=0;i<num_sensors;i++)      // for the inactive sensors the           
      sensor_array[i] =0;             // the values will be zero


 if (0) //For not do not stop the fsm
{}
  
 else
 {
  
if  (
      ( (get_io_float(ID_E_ANK2ANK_HT) < get_io_float(ID_P_FI_PP_THEIGHT))
                && ( get_io_float(ID_E_FO_ABSANG) > get_io_float(ID_P_F_PP_F_ABSANG) )  //Based on absolute angle       
                && (get_io_float(ID_MCH_ANGLE) <-0.05 )     )   
        ||
        ( ((int) get_io_float(ID_E_SWING_LEG) == 1) 
              &&  ( get_io_float(ID_MCH_ANGLE) <-0.05 ) )   
    )
  {
    sensor_array[COND_FI_stance_to_prepush] = 1;
  }  

  if(0) //skip this transition, stance to afterpush not allowed
  {
    sensor_array[COND_FI_stance_to_afterpush] = 1;
  } 

 if ( 
      (
            (fi_prepush_angle_turned > get_io_float(ID_P_F_PP_FRAC_ANG)*(fi_prepush_angle+discrete_fi_prepush_angle+fi_prepush_dangle))   
              ||
            (fi_flag_skip_prepush == 1)   
      )
      &&
     ((int) get_io_float(ID_E_SWING_LEG) == 1)
     )
  {
    sensor_array[COND_FI_prepush_to_afterpush] = 1;
  }
  

  if (
     ((get_io_float(ID_MCFI_MOTOR_POSITION)-get_io_float(ID_MCFI_MID_ANKLE_ANGLE))
            <= get_io_float(ID_P_F_FU_F_CABL_STRETCH))
            ||
     (fi_afterpush_time>=get_io_float(ID_P_F_AP_TIME))
     )        
  {
    sensor_array[COND_FI_afterpush_to_flipup] = 1;
  }

  if ( (get_io_float(ID_MCH_ANGLE) > get_io_float(ID_P_F_FU_H_ANG) )
       )
  {
    sensor_array[COND_FI_flipup_to_flipdown] = 1;
  }
  
  if ( (int) get_io_float(ID_E_SWING_LEG) == 0)
  {
    sensor_array[COND_FI_flipdown_to_stance] = 1;
  }
  
  if ( (int) get_io_float(ID_E_SWING_LEG) == 0)
  {
    sensor_array[COND_FI_startflipdown_to_stance] = 1;
  }
  
   if ( (int) get_io_float(ID_FSM_RESET) == 1 )    // Reset to start stance, called from ui_fsm
  {
    sensor_array[COND_FI_startflipdown] = 1;
  }  
  
   if (0) //For now do not stop fsm
  {
    sensor_array[COND_FI_stop] = 1;
  }
  
 }

}

