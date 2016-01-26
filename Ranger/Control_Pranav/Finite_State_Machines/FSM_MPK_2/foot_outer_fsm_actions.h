int ACT_FO_flipup()
{ 
    
  ////do this action while in flip up////
   
   ////This was copied from Pranav's FSM code to flip up the inner foot
     float target_angle;
  target_angle =  get_io_float(ID_P_F_FU_F_TANG); 
  
  set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_C_F_FU_F_ANG)); 
  set_io_float(ID_MCFO_DAMPNESS, get_io_float(ID_C_F_FU_F_RATE)); 
  float command_current = target_angle * get_io_float(ID_MCFO_STIFFNESS);
  set_io_float(ID_MCFO_COMMAND_CURRENT, command_current); 
   
 
  return 1;
}

int ACT_FO_flipdown()
{ 
    
  ////do this action while in flip down////
  
  ////This was copied from Pranav's FSM code to flip down the inner foot
  float target_angle;
  
  target_angle = get_io_float(ID_P_F_FD_F_TANG) + get_io_float(ID_MCFI_MOTOR_POSITION) - get_io_float(ID_E_FI_ABSANG); //IMU used
 //target_angle =  get_io_float(ID_P_F_FD_F_TANG) + get_io_float(ID_MCH_ANGLE) * 0.5; //IMU not used
 
  set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_C_F_FD_F_ANG));
  set_io_float(ID_MCFO_DAMPNESS, get_io_float(ID_C_F_FD_F_RATE)); 
  float command_current = target_angle * get_io_float(ID_MCFO_STIFFNESS);
  set_io_float(ID_MCFO_COMMAND_CURRENT, command_current);
  
  return 1;
}
