//Include the function for flashing an LED (it is defined fully in UI_fsm_actions.h
void set_UI_LED(int led_number, char color);

static float fo_down_swing_count, fo_up_swing_count;

int ACT_rock_FO_flipup_entry()
{
fo_up_swing_count=0;
set_UI_LED(4, '-');
set_UI_LED(5, '-');
return 1;
}

int ACT_rock_FO_flipup()
{ 
    fo_up_swing_count=fo_up_swing_count+mb_time;
  set_UI_LED(4, 'r');
  set_UI_LED(5, 'r');
  set_io_float(ID_T_TEST5, fo_up_swing_count);
    
//  ////do this action while in flip up////
   
   ////This was copied from Pranav's FSM code to flip up the inner foot
//     float target_angle;
//  target_angle =  get_io_float(ID_P_F_FU_F_TANG); 
//  
//  set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_FU_F_ANG)); 
//  set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_FU_F_RATE)); 
//  float command_current = target_angle * get_io_float(ID_MCFI_STIFFNESS);
//  set_io_float(ID_MCFI_COMMAND_CURRENT, command_current); 
   
 
  return 1;
}

int ACT_rock_FO_flipdown_entry()
{
fo_down_swing_count=0;
  set_UI_LED(4, '-');
  set_UI_LED(5, '-');
return 1;
}

int ACT_rock_FO_flipdown()
{ 
  set_UI_LED(4, 'y');
  set_UI_LED(5, 'y');
  set_io_float(ID_T_TEST5, fo_down_swing_count);
  
  ////do this action while in flip down////
  
  fo_down_swing_count=fo_down_swing_count+mb_time;
  
//     float target_angle;
//  target_angle =  0; 
//  
//  set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_FU_F_ANG)); 
//  set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_FU_F_RATE)); 
//  float command_current = target_angle * get_io_float(ID_MCFI_STIFFNESS);
//  set_io_float(ID_MCFI_COMMAND_CURRENT, command_current); 
  
  return 1;
}
