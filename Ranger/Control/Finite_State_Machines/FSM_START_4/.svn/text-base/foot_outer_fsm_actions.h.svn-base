static float discrete_fo_prepush_dgain;
static float fo_prepush_dgain;

float discrete_fo_prepush_angle;
float fo_prepush_dangle;
float fo_prepush_angle;
int fo_flag_skip_prepush;

//mb_time = 2; //main brain loop time is set in global_communications 

static float fo_prepush_to_afterpush_angle;
static float fo_stance_to_prepush_angle;

static float fo_prepush_time;
float fo_afterpush_time;

float fo_prepush_angle_turned;

static int fo_HS_occurred;

//////////////////////////////////////////////////////////////////////////////////////////////////////


int ACT_FO_stance_entry()
{
  float target_angle;

  target_angle = get_io_float(ID_P_F_ST_F_TANG) + get_io_float(ID_MCFO_MOTOR_POSITION) - get_io_float(ID_E_FO_ABSANG); //Use IMU
  //target_angle = get_io_float(ID_P_F_ST_F_TANG) - get_io_float(ID_MCH_ANGLE) * 0.5;;   //Do not use IMU
      
  set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_C_F_ST_F_ANG)); 
  set_io_float(ID_MCFO_DAMPNESS, get_io_float(ID_C_F_ST_F_RATE));   
  float  command_current = target_angle * get_io_float(ID_MCFO_STIFFNESS);
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

  float command_current = target_angle * get_io_float(ID_MCFO_STIFFNESS);
  set_io_float(ID_MCFO_COMMAND_CURRENT, command_current);
  
  return 1;
}

int ACT_FO_prepush_entry()
{

  fo_prepush_time = 0.0; 
  fo_prepush_angle = get_io_float(ID_P_F_PP_F_TANG);
  float robot_velocity_mid_stance  = get_io_float(ID_E_MIDSTANCE_LEGRATE);
  
  // Emergency state. Going too fast than skip pre-push //
  /*if (robot_velocity_mid_stance>get_io_float(ID_P_F_PP_F_HRATE))
    {
      fo_flag_skip_prepush = 1;
      fo_prepush_angle = 0.0;
     }*/
  // Emergency conditions end //
  
  fo_stance_to_prepush_angle =  get_io_float(ID_E_FO_ABSANG); 
  fo_prepush_angle_turned = 0; 
  if ( (int)get_io_float(ID_E_SWING_LEG) == 0 ) //if heel-strike occurred before pre-push starts
      set_io_float(ID_E_F_PP_MISSED,0); //outer missed
  else
      set_io_float(ID_E_F_PP_MISSED,2); //not missed 
      
    fo_HS_occurred = 0; //HS not occurred
    // Set emergency after-push (missed pre-push) to zero
    fo_prepush_dangle = 0.0;
    fo_prepush_dgain = 0.0;
  
 
 discrete_fo_prepush_dgain = 0.0;
 discrete_fo_prepush_angle = 0.0; 
/*  if (((int) get_io_float(ID_D_F_ON)) == 1)
  { 
  if (robot_velocity_mid_stance<get_io_float(ID_DP_F_PP_L_ABSRATE)) 
     {
      discrete_fo_prepush_dgain = -get_io_float(ID_DC_F_PP_L_ABSRATE)*(robot_velocity_mid_stance-get_io_float(ID_DP_F_PP_L_ABSRATE) );
      discrete_fo_prepush_angle = -get_io_float(ID_DC_F_PP_L_ABSRATE2)*(robot_velocity_mid_stance-get_io_float(ID_DP_F_PP_L_ABSRATE) );
      }    
  } */
  
  if (((int) get_io_float(ID_D_F_ON)) == 2) 
  { 
        discrete_fo_prepush_dgain = -get_io_float(ID_DC_F_PP_L_ABSRATE)*(robot_velocity_mid_stance-get_io_float(ID_DP_H_AM_L_ABSRATE) );
        discrete_fo_prepush_angle = -get_io_float(ID_DC_F_PP_L_ABSRATE2)*(robot_velocity_mid_stance-get_io_float(ID_DP_H_AM_L_ABSRATE) );       
  }     
  
  if (((int) get_io_float(ID_D_F_ON)) == 1 ) 
  { 
    if ( robot_velocity_mid_stance < get_io_float(ID_DP_F_PP_L_ABSRATE) )
        {
          discrete_fo_prepush_dgain = -get_io_float(ID_DC_F_PP_L_ABSRATE)*(robot_velocity_mid_stance-get_io_float(ID_DP_H_AM_L_ABSRATE) );
          discrete_fo_prepush_angle = -get_io_float(ID_DC_F_PP_L_ABSRATE2)*(robot_velocity_mid_stance-get_io_float(ID_DP_H_AM_L_ABSRATE) );       
        }
  }  
     
  set_io_float(ID_DA_F_PP_L_ABSRATE,  discrete_fo_prepush_dgain);
  set_io_float(ID_DA_F_PP_L_ABSRATE2,  discrete_fo_prepush_angle); 
    
  return 1;
}

int ACT_FO_prepush()
{ 
  
  float robot_velocity_mid_stance  = get_io_float(ID_E_MIDSTANCE_LEGRATE);
  if (fo_HS_occurred == 0 && ((int)get_io_float(ID_E_SWING_LEG) == 0) ) //detect heel-strike in pre-push
    {
      fo_HS_occurred = 1;
      set_io_float(ID_E_F_PP2HS_TIME,fo_prepush_time); //time between pre-push and heel-strike
      //set sensor value to give time from pre-push to heel-strike.
      
      //Emergency for missed pre-pushoff
      /*if (fo_prepush_time<=get_io_float(ID_P_F_PP_TIME) 
           && (robot_velocity_mid_stance < get_io_float(ID_P_F_PP_F_LRATE)) )
      {
        fo_prepush_dangle = get_io_float(ID_P_F_PP_F_TANG2); //put controls here
        fo_prepush_dgain = get_io_float(ID_C_F_PP_F_ANG2)-get_io_float(ID_C_F_PP_F_ANG);   //put controls here
      }*/
      //Emergency ends 
    } 

    
  fo_prepush_time =  fo_prepush_time + mb_time ; //the function is called once per 2 ms.

  
  fo_prepush_angle_turned = get_io_float(ID_E_FO_ABSANG) - fo_stance_to_prepush_angle;
      
  float target_angle;
  
  set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_C_F_PP_F_ANG)+discrete_fo_prepush_dgain+fo_prepush_dgain); 
  set_io_float(ID_MCFO_DAMPNESS, get_io_float(ID_C_F_PP_F_RATE)); 
  
  target_angle = get_io_float(ID_P_F_ST_F_TANG)+fo_prepush_angle + get_io_float(ID_MCFO_MOTOR_POSITION) 
                  - get_io_float(ID_E_FO_ABSANG) + discrete_fo_prepush_angle + fo_prepush_dangle; //Use IMU
// target_angle = fo_stance_to_prepush_angle +get_io_float(ID_P_F_PP_F_TANG) + get_io_float(ID_MCFO_MOTOR_POSITION) 
//                  - get_io_float(ID_E_FO_ABSANG) + discrete_fo_prepush_angle; //Use IMU
 
 
  float command_current = target_angle * get_io_float(ID_MCFO_STIFFNESS);
  set_io_float(ID_MCFO_COMMAND_CURRENT, command_current); 
   

  return 1; 
}

int ACT_FO_prepush_exit()
{
    set_io_float(ID_E_F_PP_TIME,fo_prepush_time);
    return 1;
}

int ACT_FO_afterpush_entry()
{ 
  fo_flag_skip_prepush = 0;
  set_io_float(ID_E_F_PP_TIME,fo_prepush_time);
  fo_prepush_to_afterpush_angle = get_io_float(ID_MCFO_MOTOR_POSITION);
  fo_afterpush_time = 0.0; 

  return 1; 
}

int ACT_FO_afterpush()
{ 

  fo_afterpush_time = fo_afterpush_time + mb_time ;
  
//     set_io_float(ID_E_TEST4, (get_io_float(ID_MCFO_MOTOR_POSITION) - get_io_float(ID_MCFO_RIGHT_ANKLE_ANGLE)) ) ;
  
   
   // PD CONTROL
//    float target_angle = fo_prepush_to_afterpush_angle;        
//   set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_C_F_AP_F_ANG)); 
//   set_io_float(ID_MCFO_DAMPNESS, get_io_float(ID_C_F_AP_F_RATE)); 
//   float command_current = target_angle * get_io_float(ID_MCFO_STIFFNESS);
//   set_io_float(ID_MCFO_COMMAND_CURRENT, command_current);
//   
   
   //Passive
   set_io_float(ID_MCFO_STIFFNESS, 0.0); 
   set_io_float(ID_MCFO_DAMPNESS, 0.0); 
   float command_current = 0.0;
   set_io_float(ID_MCFO_COMMAND_CURRENT, command_current);

  return 1; 
}

int ACT_FO_afterpush_exit()
{
     set_io_float(ID_E_F_AP_TIME,fo_afterpush_time);
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
  float  command_current = target_angle * get_io_float(ID_MCFO_STIFFNESS);
  set_io_float(ID_MCFO_COMMAND_CURRENT, command_current);
  
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
  float command_current = target_angle * get_io_float(ID_MCFO_STIFFNESS); 
  set_io_float(ID_MCFO_COMMAND_CURRENT, command_current);
  
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
  
  target_angle =  get_io_float(ID_P_F_SST_F_TANG) - get_io_float(ID_MCH_ANGLE) * 0.5; //Do not use IMU when robot starts. 
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

