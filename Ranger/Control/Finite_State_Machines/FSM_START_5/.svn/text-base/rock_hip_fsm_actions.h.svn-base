//Include the function for flashing an LED (it is defined fully in UI_fsm_actions.h
void set_UI_LED(int led_number, char color);
static float hip_counter_hold, hip_counter_set, hip_counter_free;
static float hip_hold_angle;


int ACT_rock_HIP_hold_entry(void)
{
  hip_counter_hold = 0;
  
  hip_hold_angle=get_io_float(ID_MCH_ANGLE);
  
  return 1;
}


int ACT_rock_HIP_hold(void)
{
  hip_counter_hold = hip_counter_hold + mb_time;
  //Flash a LED
  set_UI_LED(0, 'g');
  set_UI_LED(1, 'g');
  
  set_io_float(ID_T_TEST_06, hip_counter_hold);
  set_io_float(ID_T_TEST_09, hip_hold_angle);
  
  //Hold the hip angle at the position it was in when the entry function was called
  set_io_float(ID_MCH_STIFFNESS, get_io_float(ID_C_HI_SH_H_ANG));
  set_io_float(ID_MCH_DAMPNESS, get_io_float(ID_C_HI_SH_H_RATE));
  set_io_float(ID_MCH_COMMAND_CURRENT, hip_hold_angle* get_io_float(ID_C_HI_SH_H_ANG)); 
  
  
  return 1;
}



int ACT_rock_HIP_set_entry(void)
{
  hip_counter_set = 0;
  
  return 1;
}


int ACT_rock_HIP_set(void)
{
  hip_counter_set = hip_counter_set + mb_time;
  //Flash a LED
  set_UI_LED(0, 'y');
  set_UI_LED(1, 'y');
  
  set_io_float(ID_T_TEST_07, hip_counter_hold);
  
  //Set hip angle to a the hip target angle
  set_io_float(ID_MCH_STIFFNESS, get_io_float(ID_C_HI_SH_H_ANG));
  set_io_float(ID_MCH_DAMPNESS, get_io_float(ID_C_HI_SH_H_RATE));
  set_io_float(ID_MCH_COMMAND_CURRENT, get_io_float(ID_P_HI_SH_H_TANG) * get_io_float(ID_C_HI_SH_H_ANG)); 
  
  return 1;
}



int ACT_rock_HIP_free_entry(void)
{
  hip_counter_free = 0;
   
  return 1;
}


int ACT_rock_HIP_free(void)
{
  hip_counter_free = hip_counter_free + mb_time;
  //Flash a LED
  set_UI_LED(0, 'r');
  set_UI_LED(1, 'r');
  
   //Turn off the motors!
  set_io_float(ID_MCH_STIFFNESS, 0);
  set_io_float(ID_MCH_DAMPNESS, 0);
  set_io_float(ID_MCH_COMMAND_CURRENT, 0);
  
  return 1;
}
