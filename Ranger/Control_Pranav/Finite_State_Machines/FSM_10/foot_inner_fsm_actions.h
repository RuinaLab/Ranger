//#include <mb_includes.h>
extern float fi_speed_factor;

static float fi_stance_to_prepush_angle;
static float discrete_fi_prepush_dgain;
static float fi_stopstance_to_startafterpush_angle;

static int fi_start_afterpush_time_counter;


float FI_command_current_limiter(float command_current, float I_lim) //This function is used to limit the current
{

float PDval = get_io_float(ID_MCFI_STIFFNESS)*get_io_float(ID_MCFI_MOTOR_POSITION)
              +get_io_float(ID_MCFI_DAMPNESS)*get_io_float(ID_MCFI_MOTOR_VELOCITY);
float I = command_current - PDval;
                      
 if (I_lim > 0.0) //check for upper bound
    {
    if (I>I_lim) //upper bound reached?
      {command_current = I_lim + PDval;} //change upper bound
    }
 else if (I_lim < 0.0) //check for lower bound
    {
    if (I<I_lim) //lower bound reached?
      {command_current = I_lim + PDval;} //change lower bound
    }  
      
 return command_current;     
      
}


int ACT_FI_stance_entry()
{
  float target_angle;
  
  //target_angle = get_io_float(ID_P_F_ST_F_TANG) + get_io_float(ID_MCFI_MOTOR_POSITION) - get_io_float(ID_E_FI_ABSANG); //IMU used here
  target_angle =  get_io_float(ID_P_F_ST_F_TANG) + get_io_float(ID_MCH_ANGLE) * 0.5; //IMU not used


  set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_ST_F_ANG));
  set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_ST_F_RATE));
  
  float command_current = FI_command_current_limiter(target_angle * get_io_float(ID_MCFI_STIFFNESS), 1.0);
  set_io_float(ID_MCFI_COMMAND_CURRENT, command_current);  
   
 
  return 1;
}

int ACT_FI_stance()
{
  float target_angle;
  
  // target_angle = get_io_float(ID_P_F_ST_F_TANG) + get_io_float(ID_MCFI_MOTOR_POSITION) - get_io_float(ID_E_FI_ABSANG); //IMU used
  target_angle =  get_io_float(ID_P_F_ST_F_TANG) + get_io_float(ID_MCH_ANGLE) * 0.5; //IMU not used
  
  set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_ST_F_ANG));
  set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_ST_F_RATE)); 
  float command_current = FI_command_current_limiter(target_angle * get_io_float(ID_MCFI_STIFFNESS), 1.0);
  set_io_float(ID_MCFI_COMMAND_CURRENT, command_current);
  return 1;
}

int ACT_FI_stopstance_entry()
{
  float target_angle;
  
  //target_angle = get_io_float(ID_P_F_SST_F_TANG) + get_io_float(ID_MCFI_MOTOR_POSITION) - get_io_float(ID_E_FI_ABSANG); //IMU used here
  target_angle =  get_io_float(ID_P_F_SST_F_TANG) + get_io_float(ID_MCH_ANGLE) * 0.5; //IMU not used


  set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_SST_F_ANG));
  set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_SST_F_RATE));
  
  //float command_current = FI_command_current_limiter(target_angle * get_io_float(ID_MCFI_STIFFNESS), 1.0);
  //set_io_float(ID_MCFI_COMMAND_CURRENT, command_current);  
  
  set_io_float(ID_MCFI_COMMAND_CURRENT, target_angle * get_io_float(ID_MCFI_STIFFNESS));  
   
 
  return 1;
}

int ACT_FI_stopstance()
{
  float target_angle;
  
  // target_angle = get_io_float(ID_P_F_SST_F_TANG) + get_io_float(ID_MCFI_MOTOR_POSITION) - get_io_float(ID_E_FI_ABSANG); //IMU used
  target_angle =  get_io_float(ID_P_F_SST_F_TANG) + get_io_float(ID_MCH_ANGLE) * 0.5; //IMU not used
  
  set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_SST_F_ANG));
  set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_SST_F_RATE)); 
  //float command_current = FI_command_current_limiter(target_angle * get_io_float(ID_MCFI_STIFFNESS), 1.0);
  //set_io_float(ID_MCFI_COMMAND_CURRENT, command_current);
  
  set_io_float(ID_MCFI_COMMAND_CURRENT, target_angle * get_io_float(ID_MCFI_STIFFNESS)); //No current limiter, comment out earlier two lines
  
  return 1;
}


int ACT_FI_startafterpush_entry()
{

    //fi_stopstance_to_startafterpush_angle = get_io_float(ID_MCFI_MID_ANKLE_ANGLE);
    //fi_start_afterpush_time_counter = 0;
    
  return 1;
}

int ACT_FI_startafterpush()
{ 
    /*float target_angle;
    target_angle = fi_stopstance_to_startafterpush_angle + get_io_float(ID_P_F_AP_F_TANG);
     
    set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_AP_F_ANG));
    set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_AP_F_RATE));  
    float command_current  = target_angle * get_io_float(ID_MCFI_STIFFNESS);
    set_io_float(ID_MCFI_COMMAND_CURRENT, command_current); 
    
    if (fi_start_afterpush_time_counter <= get_io_float(ID_F_TEST2) )
    {
           set_io_float(ID_MCFI_STIFFNESS, 0);
           set_io_float(ID_MCFI_DAMPNESS, 0);   
           set_io_float(ID_MCFI_COMMAND_CURRENT, get_io_float(ID_F_TEST1) );
           fi_start_afterpush_time_counter = fi_start_afterpush_time_counter + 2;
           
           fi_stopstance_to_startafterpush_angle = get_io_float(ID_MCFI_MID_ANKLE_ANGLE);
    }
    else if ( (fi_start_afterpush_time_counter > get_io_float(ID_F_TEST2) ) 
               &&
              (fi_start_afterpush_time_counter < get_io_float(ID_F_TEST3) ) 
             )
    {
            target_angle = fi_stopstance_to_startafterpush_angle; //get_io_float(ID_P_F_AP_F_TANG);
            set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_AP_F_ANG));
            set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_AP_F_RATE));  
            float command_current  = target_angle * get_io_float(ID_MCFI_STIFFNESS);
            set_io_float(ID_MCFI_COMMAND_CURRENT, command_current);
              
            fi_start_afterpush_time_counter = fi_start_afterpush_time_counter + 2;
    } 
    else
        { fi_start_afterpush_time_counter = 0; } */
             

  return 1; 
}

int ACT_FI_prepush_entry()
{

    fi_stance_to_prepush_angle = get_io_float(ID_MCFI_MID_ANKLE_ANGLE);

  if (((int) get_io_float(ID_D_F_ON)) == 1)
  {
    float robot_velocity_mid_stance  = get_io_float(ID_E_MIDSTANCE_LEGRATE);
    discrete_fi_prepush_dgain = -get_io_float(ID_DC_F_PP_L_ABSRATE)*(robot_velocity_mid_stance-get_io_float(ID_DP_F_PP_L_ABSRATE) );
    if (discrete_fi_prepush_dgain<0.0) {discrete_fi_prepush_dgain = 0.0;}

  }  
  else
  {
       discrete_fi_prepush_dgain = 0.0;
  }
  
    set_io_float(ID_MCFI_STIFFNESS,  get_io_float(ID_C_F_PP_F_ANG)*fi_speed_factor+discrete_fi_prepush_dgain);
    set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_PP_F_RATE)*fi_speed_factor); 
    set_io_float(ID_DA_F_PP_A0, get_io_float(ID_C_F_PP_F_ANG)+discrete_fi_prepush_dgain ); //set the gain as a global variable 
  
  return 1;
}

int ACT_FI_prepush()
{ 
    float target_angle;
    target_angle = fi_stance_to_prepush_angle + get_io_float(ID_P_F_PP_F_TANG)*fi_speed_factor;
     
     /* if (target_angle>=2.6) //Feet safety
        {target_angle=2.6;}
      if (target_angle<=1.0)
         {target_angle = 1.0;} */
  
    set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_PP_F_ANG)*fi_speed_factor);
    set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_PP_F_RATE)*fi_speed_factor);  
    float command_current  = target_angle * get_io_float(ID_MCFI_STIFFNESS);
    set_io_float(ID_MCFI_COMMAND_CURRENT, command_current);

  return 1; 
}

int ACT_FI_afterpush_entry()
{ 
  return 1; 
}

int ACT_FI_afterpush()
{ 
  return 1; 
}

int ACT_FI_flipup_entry()
{ 
  return 1; 
}

int ACT_FI_flipup()
{ 
  float target_angle;
  target_angle =  get_io_float(ID_P_F_FU_F_TANG); //step input
  
  set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_FU_F_ANG)); 
  set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_FU_F_RATE)); 
  float command_current = FI_command_current_limiter(target_angle * get_io_float(ID_MCFI_STIFFNESS), -1.0); //Limit current to -1 A
  set_io_float(ID_MCFI_COMMAND_CURRENT, command_current); 

  return 1;
}

int ACT_FI_flipdown_entry()
{ 
  return 1;
}

int ACT_FI_flipdown()
{ 
  float target_angle;
  
 // target_angle = get_io_float(ID_P_F_FD_F_TANG) + get_io_float(ID_MCFI_MOTOR_POSITION) - get_io_float(ID_E_FI_ABSANG); //IMU used
 target_angle =  get_io_float(ID_P_F_FD_F_TANG) + get_io_float(ID_MCH_ANGLE) * 0.5; //IMU not used
 
  set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_FD_F_ANG));
  set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_FD_F_RATE)); 
  float command_current = FI_command_current_limiter(target_angle * get_io_float(ID_MCFI_STIFFNESS), 1.0); //Limit current to 1 A
  set_io_float(ID_MCFI_COMMAND_CURRENT, command_current);
 
 
  return 1;
}

int ACT_FI_startflipdown_entry()
{ 
  return 1;
}

int ACT_FI_startflipdown()
{ 
  float target_angle;
   
  // target_angle = get_io_float(ID_P_F_FD_F_TANG) + get_io_float(ID_MCFI_MOTOR_POSITION) - get_io_float(ID_E_FI_ABSANG); //IMU used
  target_angle =  get_io_float(ID_P_F_FD_F_TANG) + get_io_float(ID_MCH_ANGLE) * 0.5; //IMU not used

  set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_FD_F_ANG)); 
  set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_FD_F_RATE)); 
  float command_current = FI_command_current_limiter(target_angle * get_io_float(ID_MCFI_STIFFNESS), 0.7); //Limit current to 0.7 A
  set_io_float(ID_MCFI_COMMAND_CURRENT, command_current);

  return 1;
}


int ACT_FI_stop()
{  
  set_io_float(ID_MCFI_COMMAND_CURRENT, 0.0);
  set_io_float(ID_MCFI_STIFFNESS, 0.0);
  set_io_float(ID_MCFI_DAMPNESS, 0.0);
  return 1;
}
