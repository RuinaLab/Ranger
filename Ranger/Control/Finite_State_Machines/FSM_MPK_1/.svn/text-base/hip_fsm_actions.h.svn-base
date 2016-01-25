int ACT_HI_swing(void)
{

  set_io_float(ID_MCH_STIFFNESS, 0.25*get_io_float(ID_C_HI_SH_H_ANG));
  set_io_float(ID_MCH_DAMPNESS, 0.25*get_io_float(ID_C_HI_SH_H_RATE));
  set_io_float(ID_MCH_COMMAND_CURRENT, 0.25*get_io_float(ID_P_HI_SH_H_TANG) * get_io_float(ID_C_HI_SH_H_ANG));  
    
  return 1;
}



int ACT_HO_swing(void)
{
  set_io_float(ID_MCH_STIFFNESS, -0.25*get_io_float(ID_C_HI_SH_H_ANG));
  set_io_float(ID_MCH_DAMPNESS, -0.25*get_io_float(ID_C_HI_SH_H_RATE));
  set_io_float(ID_MCH_COMMAND_CURRENT, -0.25*get_io_float(ID_P_HI_SH_H_TANG) * get_io_float(ID_C_HI_SH_H_ANG));    
    
  return 1;
}

