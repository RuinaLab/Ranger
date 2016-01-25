//Include the function for flashing an LED (it is defined fully in UI_fsm_actions.h
void set_UI_LED(int led_number, char color);

int ACT_HI_swing(void)
{

//Flash a LED
set_UI_LED(1, '-');
set_UI_LED(2, 'y');

// Use an unstable controller to create a "frictionless" pendulum
set_io_float(ID_MCH_DAMPNESS, -0.22);



//  set_io_float(ID_MCH_STIFFNESS, 0.25*get_io_float(ID_C_HI_SH_H_ANG));
//  set_io_float(ID_MCH_DAMPNESS, 0.25*get_io_float(ID_C_HI_SH_H_RATE));
//  set_io_float(ID_MCH_COMMAND_CURRENT, 0.25*get_io_float(ID_P_HI_SH_H_TANG) * get_io_float(ID_C_HI_SH_H_ANG));  
    
  return 1;
}



int ACT_HO_swing(void)
{

//Flash a LED
set_UI_LED(1, 'o');
set_UI_LED(2, '-');

// Use an unstable controller to create a "frictionless" pendulum
set_io_float(ID_MCH_DAMPNESS, -0.22);


//  set_io_float(ID_MCH_STIFFNESS, -0.25*get_io_float(ID_C_HI_SH_H_ANG));
//  set_io_float(ID_MCH_DAMPNESS, -0.25*get_io_float(ID_C_HI_SH_H_RATE));
//  set_io_float(ID_MCH_COMMAND_CURRENT, -0.25*get_io_float(ID_P_HI_SH_H_TANG) * get_io_float(ID_C_HI_SH_H_ANG));    
    
  return 1;
}

