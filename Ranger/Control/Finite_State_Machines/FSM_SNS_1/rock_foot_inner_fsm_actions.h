//Include the function for flashing an LED (it is defined fully in UI_fsm_actions.h
void set_UI_LED(int led_number, char color);

static float fi_counter_level, fi_counter_push, fi_counter_clear;

int ACT_rock_FI_level_entry()
{
  //Initialize the counter that keeps track of the time (ms) spent in this state
  fi_counter_level=0;
  
  return 1;
}


int ACT_rock_FI_level()
{ 
  fi_counter_level=fi_counter_level+mb_time;
  
  float target_angle;
  target_angle =  get_io_float(ID_P_R_FI_TANG) + get_io_float(ID_MCH_ANGLE) * 0.5; //IMU not used during robot starting

  //set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_FD_F_ANG)); 
  set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_P_R_FI_KP)); 
  //set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_FD_F_RATE)); 
  float command_current = target_angle * get_io_float(ID_MCFI_STIFFNESS);
  set_io_float(ID_MCFI_COMMAND_CURRENT, command_current);
  
     
  return 1;
}
int ACT_rock_FI_push_entry()
{
  //Initialize the counter that keeps track of the time (ms) spent in this state
  fi_counter_push=0;
      
  return 1;
}


int ACT_rock_FI_push()
{ 
  fi_counter_push=fi_counter_push+mb_time;
  
  
  float target_angle;
  target_angle =  get_io_float(ID_P_R_FI_TANG) + get_io_float(ID_MCH_ANGLE) * 0.5; //IMU not used during robot starting
  target_angle=target_angle + 0.85; //get_io_float(ID_P_R_FI_PUSH_OFFSET);
  // FI_PUSH_OFFSET == 0.85
  // For some reason, doesn't work as well when variable is used.
  
  //set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_FD_F_ANG)); 
  set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_P_R_FI_KP)); // 20
  set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_FD_F_RATE)); 
  float command_current = target_angle * get_io_float(ID_MCFI_STIFFNESS); 
  set_io_float(ID_MCFI_COMMAND_CURRENT, command_current);

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
  set_io_float(ID_T_TEST_02, 1);
  
    float target_angle;
  target_angle =  get_io_float(ID_P_R_FI_TANG) + get_io_float(ID_MCH_ANGLE) * 0.5; //IMU not used during robot starting
  target_angle=target_angle - get_io_float(ID_P_R_FI_CLEAR_OFFSET); //1.5
  
  //set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_FD_F_ANG)); 
  set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_P_R_FI_KP)); 
  //set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_FD_F_RATE)); 
  float command_current = target_angle * get_io_float(ID_MCFI_STIFFNESS);
  set_io_float(ID_MCFI_COMMAND_CURRENT, command_current);

   
  return 1;
}
