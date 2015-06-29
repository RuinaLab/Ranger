//Include the function for flashing an LED (it is defined fully in UI_fsm_actions.h
void set_UI_LED(int led_number, char color);


static float hi_swing_count, ho_swing_count;

int ACT_HI_swing_entry(void)
{
hi_swing_count = 0;
set_UI_LED(0, '-');
set_UI_LED(1, '-');

return 1;
}


int ACT_HI_swing(void)
{

hi_swing_count = hi_swing_count + mb_time;
//Flash a LED
set_UI_LED(0, 'y');
set_UI_LED(1, 'y');


//decriment the angular position every step.
float hip_angle_nominal = get_io_float(ID_F_TEST1)-get_io_float(ID_P_R_ROCK_COUNT)*get_io_float(ID_F_TEST2);
if (hip_angle_nominal<0)
  {
  hip_angle_nominal=0;  //don't allow negative angles for robot safety reasons.
  }
set_io_float(ID_T_TEST7, hip_angle_nominal);
//This motor control code was copied from Pranav's HIP_START_HOLD code, with slight modification
  set_io_float(ID_MCH_STIFFNESS, get_io_float(ID_C_HI_SH_H_ANG));
  set_io_float(ID_MCH_DAMPNESS, get_io_float(ID_C_HI_SH_H_RATE));
  set_io_float(ID_MCH_COMMAND_CURRENT, hip_angle_nominal * get_io_float(ID_C_HI_SH_H_ANG)); 


////Use an unstable controller to create a "frictionless" pendulum
//set_io_float(ID_MCH_DAMPNESS, -0.20);



//  set_io_float(ID_MCH_STIFFNESS, 0.25*get_io_float(ID_C_HI_SH_H_ANG));
//  set_io_float(ID_MCH_DAMPNESS, 0.25*get_io_float(ID_C_HI_SH_H_RATE));
//  set_io_float(ID_MCH_COMMAND_CURRENT, 0.25*get_io_float(ID_P_HI_SH_H_TANG) * get_io_float(ID_C_HI_SH_H_ANG));  
//    
  return 1;
}

int ACT_HI_swing_exit(void)
{
//increment counter
set_io_float(ID_P_R_ROCK_COUNT, get_io_float(ID_P_R_ROCK_COUNT)+1);

return 1;
}

int ACT_HO_swing_entry(void)
{
ho_swing_count = 0;
set_UI_LED(0, '-');
set_UI_LED(1, '-');

return 1;
}


int ACT_HO_swing(void)
{
ho_swing_count = ho_swing_count + mb_time;

//Flash a LED
set_UI_LED(0, 'r');
set_UI_LED(1, 'r');


 //Use an unstable controller to create a "frictionless" pendulum
set_io_float(ID_MCH_DAMPNESS, -0.10);


//  set_io_float(ID_MCH_STIFFNESS, -0.25*get_io_float(ID_C_HI_SH_H_ANG));
//  set_io_float(ID_MCH_DAMPNESS, -0.25*get_io_float(ID_C_HI_SH_H_RATE));
//  set_io_float(ID_MCH_COMMAND_CURRENT, -0.25*get_io_float(ID_P_HI_SH_H_TANG) * get_io_float(ID_C_HI_SH_H_ANG));    
    
  return 1;
}

