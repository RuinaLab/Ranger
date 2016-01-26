//Include the function for flashing an LED (it is defined fully in UI_fsm_actions.h
void set_UI_LED(int led_number, char color);

static float fo_counter_level, fo_counter_push;


int ACT_rock_FO_level_entry()
{
fo_counter_level=0;

return 1;
}


int ACT_rock_FO_level()
{ 
    fo_counter_level=fo_counter_level+mb_time;
//    set_io_float(ID_T_TEST_04, fo_counter_level);
  
  float target_angle;
  target_angle =  get_io_float(ID_P_R_FO_TANG) - get_io_float(ID_MCH_ANGLE) * 0.5; //Do not use IMU when robot starts. 
  
  //set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_C_F_SST_F_ANG));
  set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_P_R_FO_KP));
//  set_io_float(ID_MCFO_DAMPNESS, get_io_float(ID_C_F_SST_F_RATE));
  set_io_float(ID_MCFO_COMMAND_CURRENT, target_angle * get_io_float(ID_MCFO_STIFFNESS)); 
  
      
  return 1;
}


int ACT_rock_FO_push_entry()
{
fo_counter_push=0;

return 1;
}


int ACT_rock_FO_push()
{ 
    fo_counter_push=fo_counter_push+mb_time;
//    set_io_float(ID_T_TEST_01, 2);

  float target_angle;
  target_angle =  get_io_float(ID_P_R_FO_TANG) - get_io_float(ID_MCH_ANGLE) * 0.5; //Do not use IMU when robot starts. 
  target_angle = target_angle + get_io_float(ID_P_R_FO_PUSH_OFFSET);
  
  //set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_C_F_SST_F_ANG));
  set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_P_R_FO_KP));
  //set_io_float(ID_MCFO_DAMPNESS, get_io_float(ID_C_F_SST_F_RATE));
  set_io_float(ID_MCFO_COMMAND_CURRENT, target_angle * get_io_float(ID_MCFO_STIFFNESS)); 
    
  return 1;
}
