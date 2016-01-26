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
  
  set_UI_LED(4, 'r');
  set_UI_LED(5, 'r');

  set_io_float(ID_T_TEST_04, fo_counter_level);
      
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
  
  set_UI_LED(4, 'g');
  set_UI_LED(5, 'g');
    
  set_io_float(ID_T_TEST_05, fo_counter_push);
    
  return 1;
}
