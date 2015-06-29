//Include the function for flashing an LED (it is defined fully in UI_fsm_actions.h
void set_UI_LED(int led_number, char color);
static float hip_counter_hold, hip_counter_set, hip_counter_free;


int ACT_rock_HIP_hold_entry(void)
{
  hip_counter_hold = 0;
  
  return 1;
}


int ACT_rock_HIP_hold(void)
{
  hip_counter_hold = hip_counter_hold + mb_time;
  //Flash a LED
  set_UI_LED(0, 'g');
  set_UI_LED(1, 'g');
  
  set_io_float(ID_T_TEST_06, hip_counter_hold);
  
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
  
  set_io_float(ID_T_TEST_08, hip_counter_hold);
  
  return 1;
}
