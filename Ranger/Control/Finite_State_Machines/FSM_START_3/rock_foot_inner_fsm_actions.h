//Include the function for flashing an LED (it is defined fully in UI_fsm_actions.h
void set_UI_LED(int led_number, char color);

static float fi_counter_level, fi_counter_push, fi_counter_clear;

int ACT_rock_FI_level_entry()
{
  //Initialize the counter that keeps track of the time (ms) spent in this state
  fi_counter_level=0;
  
  ////increment counter to update hip position for things later
  //set_io_float(ID_P_R_ROCK_COUNT, get_io_float(ID_P_R_ROCK_COUNT)+1);
  
  return 1;
}


int ACT_rock_FI_level()
{ 
  fi_counter_level=fi_counter_level+mb_time;
  
  set_UI_LED(2, 'g');
  set_UI_LED(3, 'g');
  set_io_float(ID_T_TEST_01, fi_counter_level);
  
  return 1;
}
int ACT_rock_FI_push_entry()
{
  //Initialize the counter that keeps track of the time (ms) spent in this state
  fi_counter_push=0;
  
  return 1;
}


int ACT_rock_FI_push()
{ 
  fi_counter_push=fi_counter_push+mb_time;
  
  set_UI_LED(2, 'y');
  set_UI_LED(3, 'y');
  set_io_float(ID_T_TEST_02, fi_counter_push);
  
  return 1;
}

int ACT_rock_FI_clear_entry()
{
  //Initialize the counter that keeps track of the time (ms) spent in this state
  fi_counter_clear=0;
  
  return 1;
}


int ACT_rock_FI_clear()
{ 
  fi_counter_clear=fi_counter_clear+mb_time;

  set_UI_LED(2, 'r');
  set_UI_LED(3, 'r');
  set_io_float(ID_T_TEST_03, fi_counter_clear);
  
  return 1;
}
