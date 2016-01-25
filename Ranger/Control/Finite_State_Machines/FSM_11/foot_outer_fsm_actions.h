static float fo_stance_to_prepush_angle;
static float discrete_fo_prepush_dgain;

//////////////////////////////////////////////////////////////////////////////////////////////////////

float FO_command_current_limiter(float command_current, float I_lim)
{

float PDval = get_io_float(ID_MCFO_STIFFNESS)*get_io_float(ID_MCFO_MOTOR_POSITION)
              +get_io_float(ID_MCFO_DAMPNESS)*get_io_float(ID_MCFO_MOTOR_VELOCITY);
float I = command_current - PDval;
                      
 if (I_lim > 0) //check for upper bound
    {
    if (I>I_lim) //upper bound reached?
      {command_current = I_lim + PDval;} //change upper bound
    }
 else if (I_lim < 0) //check for lower bound
    {
    if (I<I_lim) //lower bound reached?
      {command_current = I_lim + PDval;} //change lower bound
    }  
      
 return command_current;       
}


int ACT_FO_stance_entry()
{
  float target_angle;

  target_angle = get_io_float(ID_P_F_ST_F_TANG) + get_io_float(ID_MCFO_MOTOR_POSITION) - get_io_float(ID_E_FO_ABSANG); //Use IMU
  //target_angle = get_io_float(ID_P_F_ST_F_TANG) - get_io_float(ID_MCH_ANGLE) * 0.5;;   //Do not use IMU
      
  set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_C_F_ST_F_ANG)); 
  set_io_float(ID_MCFO_DAMPNESS, get_io_float(ID_C_F_ST_F_RATE));   
  float command_current = FO_command_current_limiter(target_angle * get_io_float(ID_MCFO_STIFFNESS), 1.0); //Current limiter
  set_io_float(ID_MCFO_COMMAND_CURRENT, command_current);
  
  return 1;
}

int ACT_FO_stance()
{
  float target_angle;
  
  set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_C_F_ST_F_ANG)); 
  set_io_float(ID_MCFO_DAMPNESS, get_io_float(ID_C_F_ST_F_RATE)); 
  
  target_angle = get_io_float(ID_P_F_ST_F_TANG) + get_io_float(ID_MCFO_MOTOR_POSITION) - get_io_float(ID_E_FO_ABSANG); //Use IMU
  //target_angle =  get_io_float(ID_P_F_ST_F_TANG) - get_io_float(ID_MCH_ANGLE) * 0.5; //Do not use IMU

  float command_current = FO_command_current_limiter(target_angle * get_io_float(ID_MCFO_STIFFNESS), 1.0); //Current limiter 
  set_io_float(ID_MCFO_COMMAND_CURRENT, command_current);
  //set_io_float(ID_MCFO_COMMAND_CURRENT, target_angle * get_io_float(ID_MCFO_STIFFNESS)); //No current limiter, comment out earlier two lines
  
  return 1;
}

int ACT_FO_prepush_entry()
{
  fo_stance_to_prepush_angle = get_io_float(ID_MCFO_RIGHT_ANKLE_ANGLE); //save the stance angle during exit
  
  if (((int) get_io_float(ID_D_F_ON)) == 1)
  {
    float robot_velocity_mid_stance  = get_io_float(ID_E_MIDSTANCE_LEGRATE);
  
      discrete_fo_prepush_dgain = -get_io_float(ID_DC_F_PP_L_ABSRATE)*(robot_velocity_mid_stance-get_io_float(ID_DP_F_PP_L_ABSRATE) );
      if (discrete_fo_prepush_dgain<0.0) {discrete_fo_prepush_dgain = 0.0;}
  }
  else
  {
        discrete_fo_prepush_dgain = 0.0;
  }  

     set_io_float(ID_DA_F_PP_A0, get_io_float(ID_C_F_PP_F_ANG)+discrete_fo_prepush_dgain); //set a global variable for display in labview
     set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_C_F_PP_F_ANG)+discrete_fo_prepush_dgain ); 
     set_io_float(ID_MCFO_DAMPNESS, get_io_float(ID_C_F_PP_F_RATE));
 
  return 1;
}

int ACT_FO_prepush()
{ 
    float target_angle;
    target_angle = fo_stance_to_prepush_angle + get_io_float(ID_P_F_PP_F_TANG); 
    /*if (target_angle>=2.6) //Feet safety
      {target_angle=2.6;}
    if (target_angle<=1.0)
       {target_angle = 1.0;}  */
       
   set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_C_F_PP_F_ANG)); 
   set_io_float(ID_MCFO_DAMPNESS, get_io_float(ID_C_F_PP_F_RATE));
   float command_current = target_angle * get_io_float(ID_MCFO_STIFFNESS);
   set_io_float(ID_MCFO_COMMAND_CURRENT, command_current);
   

  return 1; 
}

int ACT_FO_afterpush_entry()
{ 
  return 1; 
}

int ACT_FO_afterpush()
{ 
  return 1; 
}

int ACT_FO_flipup_entry()
{ 
  return 1;
}

int ACT_FO_flipup()
{ 
  float target_angle;
  
  set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_C_F_FU_F_ANG)); 
  set_io_float(ID_MCFO_DAMPNESS, get_io_float(ID_C_F_FU_F_RATE));
  
  target_angle =  get_io_float(ID_P_F_FU_F_TANG);
  
  float command_current = FO_command_current_limiter(target_angle * get_io_float(ID_MCFO_STIFFNESS), -1.0); //current limiter
  set_io_float(ID_MCFO_COMMAND_CURRENT, command_current);
  //set_io_float(ID_MCFO_COMMAND_CURRENT, target_angle * get_io_float(ID_MCFO_STIFFNESS)); //no current limiter, if using this, comment the earlier line two lines out
  
  return 1;
}

int ACT_FO_flipdown_entry()
{  
  return 1;
}

int ACT_FO_flipdown()
{ 
  float target_angle;
 
  target_angle = get_io_float(ID_P_F_FD_F_TANG) + get_io_float(ID_MCFO_MOTOR_POSITION) - get_io_float(ID_E_FO_ABSANG); //Use IMU
  //target_angle =  get_io_float(ID_P_F_FD_F_TANG) - get_io_float(ID_MCH_ANGLE) * 0.5; //Do not use IMU

  set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_C_F_FD_F_ANG)); 
  set_io_float(ID_MCFO_DAMPNESS, get_io_float(ID_C_F_FD_F_RATE));  
   float command_current = FO_command_current_limiter(target_angle * get_io_float(ID_MCFO_STIFFNESS),1.0); //Current limiter
  set_io_float(ID_MCFO_COMMAND_CURRENT, command_current);
  //set_io_float(ID_MCFO_COMMAND_CURRENT, target_angle * get_io_float(ID_MCFO_STIFFNESS)); //No current limiter
  
  return 1;
}

int ACT_FO_startstance_entry()
{
  return 1;
}

int ACT_FO_startstance()
{
  float target_angle;
  
  set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_C_F_SST_F_ANG));
  set_io_float(ID_MCFO_DAMPNESS, get_io_float(ID_C_F_SST_F_RATE));
  
  target_angle =  get_io_float(ID_P_F_SST_F_TANG) - get_io_float(ID_MCH_ANGLE) * 0.5;
  
  set_io_float(ID_MCFO_COMMAND_CURRENT, target_angle * get_io_float(ID_MCFO_STIFFNESS)); 
  return 1;
}

int ACT_FO_stop()
{  
  set_io_float(ID_MCFO_COMMAND_CURRENT, 0.0);
  set_io_float(ID_MCFO_STIFFNESS, 0.0);
  set_io_float(ID_MCFO_DAMPNESS, 0.0);
  return 1;
}

