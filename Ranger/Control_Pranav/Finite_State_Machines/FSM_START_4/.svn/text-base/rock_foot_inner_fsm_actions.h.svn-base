//Include the function for flashing an LED (it is defined fully in UI_fsm_actions.h
void set_UI_LED(int led_number, char color);

static float fi_counter_level, fi_counter_push, fi_counter_clear;

//static float fi_stance_to_prepush_angle;
//static float fi_prepush_to_afterpush_angle;
//
//static float fi_prepush_time;
//float fi_afterpush_time;
//
//float discrete_fi_prepush_angle;
//float fi_prepush_dangle;
//float fi_prepush_angle;
//
//static float discrete_fi_prepush_dgain;
//static float fi_prepush_dgain;
//
//float fi_prepush_angle_turned;



int ACT_rock_FI_level_entry()
{
  //Initialize the counter that keeps track of the time (ms) spent in this state
  fi_counter_level=0;
  
  ////increment counter to update hip position for things later
  //set_io_float(ID_P_R_ROCK_COUNT, get_io_float(ID_P_R_ROCK_COUNT)+1);
  
  return 1;
}


int ACT_rock_FI_level()
{ 
  fi_counter_level=fi_counter_level+mb_time;
  
  set_UI_LED(2, 'g');
  set_UI_LED(3, 'g');
  set_io_float(ID_T_TEST_01, fi_counter_level);
  
  float target_angle;
  target_angle =  get_io_float(ID_P_F_FD_F_TANG) + get_io_float(ID_MCH_ANGLE) * 0.5; //IMU not used during robot starting

  //set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_FD_F_ANG)); 
  set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_P_R_FI_KP)); 
  set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_FD_F_RATE)); 
  float command_current = target_angle * get_io_float(ID_MCFI_STIFFNESS);
  set_io_float(ID_MCFI_COMMAND_CURRENT, command_current);
  
  set_io_float(ID_T_TEST_08, command_current);
   
  return 1;
}
int ACT_rock_FI_push_entry()
{
  //Initialize the counter that keeps track of the time (ms) spent in this state
  fi_counter_push=0;
  
//      fi_prepush_time = 0.0; 
//      fi_prepush_angle = get_io_float(ID_P_F_PP_F_TANG);
//      float robot_velocity_mid_stance  = get_io_float(ID_E_MIDSTANCE_LEGRATE);
//
//        
//      fi_HS_occurred = 0; //HS not occurred
//      // Set emergency after-push (missed pre-push) to zero
//      fi_prepush_dangle = 0.0;
//      fi_prepush_dgain = 0.0;
//  
//      fi_stance_to_prepush_angle = get_io_float(ID_E_FI_ABSANG); 
//      fi_prepush_angle_turned = 0; 
//    
//      discrete_fi_prepush_dgain = 0.0;
//      discrete_fi_prepush_angle = 0.0 
      
  return 1;
}


int ACT_rock_FI_push()
{ 
  fi_counter_push=fi_counter_push+mb_time;
  
  set_UI_LED(2, 'y');
  set_UI_LED(3, 'y');
  set_io_float(ID_T_TEST_02, fi_counter_push);
  
  
  float target_angle;
  target_angle =  get_io_float(ID_P_F_FD_F_TANG) + get_io_float(ID_MCH_ANGLE) * 0.5; //IMU not used during robot starting
  target_angle=target_angle + get_io_float(ID_P_R_FI_PUSH_OFFSET);
  
  //set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_FD_F_ANG)); 
  set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_P_R_FI_KP)); 
  set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_FD_F_RATE)); 
  float command_current = target_angle * get_io_float(ID_MCFI_STIFFNESS);
  set_io_float(ID_MCFI_COMMAND_CURRENT, command_current);
  
  
  
//  
//      float robot_velocity_mid_stance  = get_io_float(ID_E_MIDSTANCE_LEGRATE);
//        if (fi_HS_occurred == 0 && ((int)get_io_float(ID_E_SWING_LEG) == 1) ) //detect heel-strike in pre-push
//        {
//          fi_HS_occurred = 1;
//          set_io_float(ID_E_F_PP2HS_TIME,fi_prepush_time); //time between pre-push and heel-strike
//          //set sensor value to give time from pre-push to heel-strike.
// 
//        }
//    
//      
//        fi_prepush_time =  fi_prepush_time + mb_time ; //the function is called once per 2 ms.  
//    
//    
//        fi_prepush_angle_turned = get_io_float(ID_E_FI_ABSANG) - fi_stance_to_prepush_angle;
//        
//        float target_angle;
//        target_angle = get_io_float(ID_P_F_ST_F_TANG)+fi_prepush_angle + get_io_float(ID_MCFI_MOTOR_POSITION) 
//                      - get_io_float(ID_E_FI_ABSANG)+discrete_fi_prepush_angle+fi_prepush_dangle; //IMU used
//    //   target_angle = fi_stance_to_prepush_angle + get_io_float(ID_P_F_PP_F_TANG) + get_io_float(ID_MCFI_MOTOR_POSITION) 
//    //                  - get_io_float(ID_E_FI_ABSANG)+discrete_fi_prepush_angle; //IMU used
//                                                    
//                       
//        set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_PP_F_ANG)+discrete_fi_prepush_dgain+fi_prepush_dgain);
//        set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_PP_F_RATE));
//        float command_current = target_angle * get_io_float(ID_MCFI_STIFFNESS);
//        set_io_float(ID_MCFI_COMMAND_CURRENT, command_current);  
//  
//  
//  
  
  
  return 1;
}

int ACT_rock_FI_clear_entry()
{
  //Initialize the counter that keeps track of the time (ms) spent in this state
  fi_counter_clear=0;
  
  return 1;
}


int ACT_rock_FI_clear()
{ 
  fi_counter_clear=fi_counter_clear+mb_time;

  set_UI_LED(2, 'r');
  set_UI_LED(3, 'r');
  set_io_float(ID_T_TEST_03, fi_counter_clear);
  
  
    float target_angle;
  target_angle =  get_io_float(ID_P_F_FD_F_TANG) + get_io_float(ID_MCH_ANGLE) * 0.5; //IMU not used during robot starting
  target_angle=target_angle - get_io_float(ID_P_R_FI_CLEAR_OFFSET);
  
  //set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_FD_F_ANG)); 
  set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_P_R_FI_KP)); 
  set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_FD_F_RATE)); 
  float command_current = target_angle * get_io_float(ID_MCFI_STIFFNESS);
  set_io_float(ID_MCFI_COMMAND_CURRENT, command_current);
  
//  //Control the relative angle between the leg and foot to be minimum allowed (~0.24) with some tolerance
//  //float angle_error=0.3-get_io_float(ID_E_LI_ABSANG)+get_io_float(ID_E_FI_ABSANG);
//  float angle_error=1.2-get_io_float(ID_E_FI_ABSANG);
//  
//  //Do all proportional control on top level
//  set_io_float(ID_MCFI_STIFFNESS, 0);
//  float command_current = angle_error * get_io_float(ID_C_F_ST_F_ANG);
//  //Use low level derivitive control with default rate
//  set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_ST_F_RATE));
//  set_io_float(ID_MCFI_COMMAND_CURRENT, command_current);
//  
//  set_io_float(ID_T_TEST_10, command_current);
   
  return 1;
}
