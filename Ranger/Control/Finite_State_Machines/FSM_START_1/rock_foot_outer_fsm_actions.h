void set_UI_LED(int led_number, char color);

int ACT_rock_FO_flipup()
{ 
//  set_UI_LED(4, '-');
//  set_UI_LED(5, 'y');
  ////do this action while in flip up////
   
   ////This was copied from Pranav's FSM code to flip up the inner foot
     float target_angle;
  target_angle =  0;
  
  set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_C_F_FU_F_ANG)); 
  set_io_float(ID_MCFO_DAMPNESS, get_io_float(ID_C_F_FU_F_RATE)); 
  float command_current = target_angle * get_io_float(ID_MCFO_STIFFNESS);
  set_io_float(ID_MCFO_COMMAND_CURRENT, command_current); 
   
 
  return 1;
}

int ACT_rock_FO_flipdown()
{ 
//      set_UI_LED(4, 'y');
//  set_UI_LED(5, '-');
//  ////do this action while in flip down////
//  
//  ////This was copied from Pranav's FSM code to flip down the inner foot
//  float target_angle;
//  
     float target_angle;
  target_angle =  get_io_float(ID_P_F_FU_F_TANG); 
  
  set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_C_F_FU_F_ANG)); 
  set_io_float(ID_MCFO_DAMPNESS, get_io_float(ID_C_F_FU_F_RATE)); 
  float command_current = target_angle * get_io_float(ID_MCFO_STIFFNESS);
  set_io_float(ID_MCFO_COMMAND_CURRENT, command_current); 
  
  return 1;
}
