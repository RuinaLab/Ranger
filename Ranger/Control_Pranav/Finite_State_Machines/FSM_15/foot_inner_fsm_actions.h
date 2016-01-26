static float fi_stance_to_prepush_angle;
static float fi_prepush_to_afterpush_angle;

static float fi_prepush_time;
float fi_afterpush_time;
int fi_flag_skip_prepush;

//mb_time = 2; //main brain loop time is set in global_communications 

float discrete_fi_prepush_angle;
float fi_prepush_dangle;
float fi_prepush_angle;

static float discrete_fi_prepush_dgain;
static float fi_prepush_dgain;

float fi_prepush_angle_turned;

static int fi_HS_occurred;



int ACT_FI_stance_entry()
{
  float target_angle;
  
  target_angle = get_io_float(ID_P_F_ST_F_TANG) + get_io_float(ID_MCFI_MOTOR_POSITION) - get_io_float(ID_E_FI_ABSANG); //IMU used here
  //target_angle =  get_io_float(ID_P_F_ST_F_TANG) + get_io_float(ID_MCH_ANGLE) * 0.5; //IMU not used


  set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_ST_F_ANG));
  set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_ST_F_RATE));
  
  float command_current = target_angle * get_io_float(ID_MCFI_STIFFNESS);
  set_io_float(ID_MCFI_COMMAND_CURRENT, command_current);  
   
 
  return 1;
}

int ACT_FI_stance()
{
  float target_angle;
  
  target_angle = get_io_float(ID_P_F_ST_F_TANG) + get_io_float(ID_MCFI_MOTOR_POSITION) - get_io_float(ID_E_FI_ABSANG); //IMU used
  //target_angle =  get_io_float(ID_P_F_ST_F_TANG) + get_io_float(ID_MCH_ANGLE) * 0.5; //IMU not used
  
  set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_ST_F_ANG));
  set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_ST_F_RATE)); 
  float command_current = target_angle * get_io_float(ID_MCFI_STIFFNESS);
  set_io_float(ID_MCFI_COMMAND_CURRENT, command_current);
  return 1;
}

int ACT_FI_prepush_entry()
{
    fi_prepush_time = 0.0; 
    fi_prepush_angle = get_io_float(ID_P_F_PP_F_TANG);
    float robot_velocity_mid_stance  = get_io_float(ID_E_MIDSTANCE_LEGRATE);
    
    // Emergency state. Going too fast than skip pre-push //
    /*if (robot_velocity_mid_stance > get_io_float(ID_P_F_PP_F_HRATE))
        {
        fi_flag_skip_prepush = 1;
        fi_prepush_angle = 0.0;
        }*/
    // Emergency conditions end //
    
    fi_HS_occurred = 0; //HS not occurred
    // Set emergency after-push (missed pre-push) to zero
    fi_prepush_dangle = 0.0;
    fi_prepush_dgain = 0.0;
    
   if ( (int)get_io_float(ID_E_SWING_LEG) == 1 ) //if heel-strike occurred before pre-push starts
      set_io_float(ID_E_F_PP_MISSED,1); //inner missed
    else
      set_io_float(ID_E_F_PP_MISSED,2); //not missed 

    fi_stance_to_prepush_angle = get_io_float(ID_E_FI_ABSANG); 
    fi_prepush_angle_turned = 0; 

  discrete_fi_prepush_dgain = 0.0;
  discrete_fi_prepush_angle = 0.0;
  
 /* if (((int) get_io_float(ID_D_F_ON)) == 1)
  { 
    if (robot_velocity_mid_stance < get_io_float(ID_DP_F_PP_L_ABSRATE))
      {
        discrete_fi_prepush_dgain = -get_io_float(ID_DC_F_PP_L_ABSRATE)*(robot_velocity_mid_stance-get_io_float(ID_DP_F_PP_L_ABSRATE) );
        discrete_fi_prepush_angle = -get_io_float(ID_DC_F_PP_L_ABSRATE2)*(robot_velocity_mid_stance-get_io_float(ID_DP_F_PP_L_ABSRATE) );       
      }      
     
  } */ 
  
   if (((int) get_io_float(ID_D_F_ON)) == 2  )
  { 
        discrete_fi_prepush_dgain = -get_io_float(ID_DC_F_PP_L_ABSRATE)*(robot_velocity_mid_stance-get_io_float(ID_DP_H_AM_L_ABSRATE) );
        discrete_fi_prepush_angle = -get_io_float(ID_DC_F_PP_L_ABSRATE2)*(robot_velocity_mid_stance-get_io_float(ID_DP_H_AM_L_ABSRATE) );    
  }     
  
    if (((int) get_io_float(ID_D_F_ON)) == 1 )
  { 
    if (robot_velocity_mid_stance < get_io_float(ID_DP_F_PP_L_ABSRATE) )
    {
        discrete_fi_prepush_dgain = -get_io_float(ID_DC_F_PP_L_ABSRATE)*(robot_velocity_mid_stance-get_io_float(ID_DP_H_AM_L_ABSRATE) );
        discrete_fi_prepush_angle = -get_io_float(ID_DC_F_PP_L_ABSRATE2)*(robot_velocity_mid_stance-get_io_float(ID_DP_H_AM_L_ABSRATE) );  
    }     
  }   
    
  set_io_float(ID_DA_F_PP_L_ABSRATE,  discrete_fi_prepush_dgain);
  set_io_float(ID_DA_F_PP_L_ABSRATE2,  discrete_fi_prepush_angle); 
    
  return 1;
}

int ACT_FI_prepush()
{ 
    
    float robot_velocity_mid_stance  = get_io_float(ID_E_MIDSTANCE_LEGRATE);
    if (fi_HS_occurred == 0 && ((int)get_io_float(ID_E_SWING_LEG) == 1) ) //detect heel-strike in pre-push
    {
      fi_HS_occurred = 1;
      set_io_float(ID_E_F_PP2HS_TIME,fi_prepush_time); //time between pre-push and heel-strike
      //set sensor value to give time from pre-push to heel-strike.
      
      //Emergency for missed pre-pushoff
      /*if (fi_prepush_time<=get_io_float(ID_P_F_PP_TIME) 
            && (robot_velocity_mid_stance < get_io_float(ID_P_F_PP_F_LRATE)))
      {
        fi_prepush_dangle = get_io_float(ID_P_F_PP_F_TANG2); //put controls here
        fi_prepush_dgain = get_io_float(ID_C_F_PP_F_ANG2)-get_io_float(ID_C_F_PP_F_ANG);   //put controls here
      }*/
      //Emergency ends   
      
         //Added 4/25/2011
         if (((int) get_io_float(ID_D_F_ON)) == 2  )
          { 
                fi_prepush_dgain = discrete_fi_prepush_dgain;
                fi_prepush_dangle = discrete_fi_prepush_angle;    
          }     
          
          if (((int) get_io_float(ID_D_F_ON)) == 1 )
          { 
              if (robot_velocity_mid_stance < get_io_float(ID_DP_F_PP_L_ABSRATE) )
              {
                  fi_prepush_dgain = discrete_fi_prepush_dgain;
                  fi_prepush_dangle = discrete_fi_prepush_angle;   
              }     
          }
          //Added 4/25/2011   
    }

  
    fi_prepush_time =  fi_prepush_time + mb_time ; //the function is called once per 2 ms.  


    fi_prepush_angle_turned = get_io_float(ID_E_FI_ABSANG) - fi_stance_to_prepush_angle;
    
    float target_angle;
    target_angle = get_io_float(ID_P_F_ST_F_TANG)+fi_prepush_angle + get_io_float(ID_MCFI_MOTOR_POSITION)  //Added 4/25/2011 
                  - get_io_float(ID_E_FI_ABSANG)+fi_prepush_dangle; //IMU used               
//    target_angle = get_io_float(ID_P_F_ST_F_TANG)+fi_prepush_angle + get_io_float(ID_MCFI_MOTOR_POSITION) 
//                  - get_io_float(ID_E_FI_ABSANG)+discrete_fi_prepush_angle+fi_prepush_dangle; //IMU used
//   target_angle = fi_stance_to_prepush_angle + get_io_float(ID_P_F_PP_F_TANG) + get_io_float(ID_MCFI_MOTOR_POSITION) 
//                  - get_io_float(ID_E_FI_ABSANG)+discrete_fi_prepush_angle; //IMU used
                                                
    set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_PP_F_ANG)+fi_prepush_dgain);    //Added 4/25/2011             
    //set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_PP_F_ANG)+discrete_fi_prepush_dgain+fi_prepush_dgain);
    set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_PP_F_RATE));
    float command_current = target_angle * get_io_float(ID_MCFI_STIFFNESS);
    set_io_float(ID_MCFI_COMMAND_CURRENT, command_current);  

  return 1; 
}

int ACT_FI_prepush_exit()
{
    set_io_float(ID_E_F_PP_TIME,fi_prepush_time);
    return 1;
}

int ACT_FI_afterpush_entry()
{ 
  
  fi_flag_skip_prepush = 0; //reset skip prepush to zero
   
  fi_afterpush_time = 0.0;
  fi_prepush_to_afterpush_angle = get_io_float(ID_MCFI_MOTOR_POSITION);

  return 1; 
}

int ACT_FI_afterpush()
{ 
    
   float command_current; 
   fi_afterpush_time =  fi_afterpush_time + mb_time ; 
   
//    set_io_float(ID_E_TEST4, (get_io_float(ID_MCFI_MOTOR_POSITION)-get_io_float(ID_MCFI_MID_ANKLE_ANGLE) )) ;
  
   
      // PD CONTROL 
//   float target_angle = fi_prepush_to_afterpush_angle; 
//   set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_AP_F_ANG )); 
//   set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_AP_F_RATE)); 
//   command_current = target_angle * get_io_float(ID_MCFI_STIFFNESS); 
//   set_io_float(ID_MCFI_COMMAND_CURRENT, command_current);
   

  
      // Passive
   set_io_float(ID_MCFI_STIFFNESS, 0.0); 
   set_io_float(ID_MCFI_DAMPNESS, 0.0); 
   command_current = 0.0; 
   set_io_float(ID_MCFI_COMMAND_CURRENT, command_current);
   
  return 1; 
}

int ACT_FI_afterpush_exit()
{
     set_io_float(ID_E_F_AP_TIME,fi_afterpush_time);
    return 1;
}

int ACT_FI_flipup_entry()
{ 
  return 1; 
}

int ACT_FI_flipup()
{ 
  float target_angle;
  target_angle =  get_io_float(ID_P_F_FU_F_TANG); 
  
  set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_FU_F_ANG)); 
  set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_FU_F_RATE)); 
  float command_current = target_angle * get_io_float(ID_MCFI_STIFFNESS);
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
  
  target_angle = get_io_float(ID_P_F_FD_F_TANG) + get_io_float(ID_MCFI_MOTOR_POSITION) - get_io_float(ID_E_FI_ABSANG); //IMU used
 //target_angle =  get_io_float(ID_P_F_FD_F_TANG) + get_io_float(ID_MCH_ANGLE) * 0.5; //IMU not used
 
  set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_FD_F_ANG));
  set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_FD_F_RATE)); 
  float command_current = target_angle * get_io_float(ID_MCFI_STIFFNESS);
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
  target_angle =  get_io_float(ID_P_F_FD_F_TANG) + get_io_float(ID_MCH_ANGLE) * 0.5; //IMU not used during robot starting

  set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_FD_F_ANG)); 
  set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_FD_F_RATE)); 
  float command_current = target_angle * get_io_float(ID_MCFI_STIFFNESS);
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
