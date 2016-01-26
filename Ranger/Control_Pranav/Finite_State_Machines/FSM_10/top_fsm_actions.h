static int step_number_when_stop_command_received;

int ACTION_top_walk_stop_entry(void)
{ 
   
    step_number_when_stop_command_received = (int) get_io_float(ID_E_STEP_NO); 
     
    return 1;
}


int ACTION_top_walk_stop(void)
{ 
    float rc_command = get_io_float(ID_UI_RC_1)/52000 - 1.1; //A number between 0 and 1
    set_io_float(ID_A_T_RC1_NORM,rc_command);  
    
    int step_no = (int) get_io_float(ID_E_STEP_NO);
    
    if (step_no == step_number_when_stop_command_received + 1.0)//remove push-off first
          {
          set_io_float(ID_T_TEST1,get_io_float(ID_H_TEST1) );
          set_io_float(ID_T_TEST2,1.0 ); //does not matter. prepush skipped
          set_io_float(ID_T_TEST3,0.0); //don't stop
           set_io_float(ID_T_TEST4,1.0); //don't prepush
           set_io_float(ID_T_TEST5,0.0); //flag to indicate start. 0 means don't start.
          }
    else if (step_no == step_number_when_stop_command_received + 2.0)//not stop by going to ehold on hip and stop stance on feet
          {
          set_io_float(ID_T_TEST1,1.0 ); //does not matter, stophold skips the normal fsm.
          set_io_float(ID_T_TEST2,1.0 ); //does not matter, prepush skipped
          set_io_float(ID_T_TEST3,1.0); //stop, got to stop hold and don't flip up trailing foot. 
          set_io_float(ID_T_TEST4,1.0); //don't prepush
          set_io_float(ID_T_TEST5,0.0); //flag to indicate start. 0 means don't start.
          }
                    
    return 1;
}

int ACTION_top_walk_start_entry(void)
{  
// Set normal walking parameters.
    set_io_float(ID_T_TEST1,0.9); //hip swing modulation 0.6 - 1.2 (prop to velocity controller)
    set_io_float(ID_T_TEST2,0.9); //push-off modulation 0.6 - 1.2
    set_io_float(ID_T_TEST3,0.0); //flag to indicate stop. 0 means don't stop.
                                  //hip go to stophold and feet go to stopstance
    set_io_float(ID_T_TEST4,0.0); //setting this to 1 removes push-off completely. no use of T_TEST2 in that case
    set_io_float(ID_T_TEST5,1.0); //flag to indicate start. 0 means don't start.
  	return 1;
 }
 
 int ACTION_top_walk_start(void)
{
   
    float rc_command = get_io_float(ID_UI_RC_1)/52000 - 1.1; //A number between 0 and 1
    set_io_float(ID_A_T_RC1_NORM,rc_command);
    
    return 1;    
}


int ACTION_top_walk_normal_entry(void)
{  
    set_io_float(ID_T_TEST1,0.9); //hip swing modulation 0.6 - 1.2 (prop to velocity controller)
    set_io_float(ID_T_TEST2,0.9); //push-off modulation 0.6 - 1.2
    set_io_float(ID_T_TEST3,0.0); //flag to indicate stop. 0 means don't stop.
                                  //hip go to stophold and feet go to stopstance
    set_io_float(ID_T_TEST4,0.0); //setting this to 1 removes push-off completely. no use of T_TEST2 in that case
    set_io_float(ID_T_TEST5,0.0); //flag to indicate start. 0 means don't start.
    
  	return 1;
 }
 
 int ACTION_top_walk_normal(void)
{
   
    float rc_command = get_io_float(ID_UI_RC_1)/52000 - 1.1; //A number between 0 and 1
    set_io_float(ID_A_T_RC1_NORM,rc_command);
    
    return 1;    
}
 
 int ACTION_top_walk_fast_entry(void)
{ 
    set_io_float(ID_T_TEST1,1.1); //hip swing modulation 0.6 - 1.2 (prop to velocity controller)
    set_io_float(ID_T_TEST2,1.1); //push-off modulation 0.6 - 1.2
    set_io_float(ID_T_TEST3,0.0); //flag to indicate stop. 0 means don't stop.
                                  //hip go to stophold and feet go to stopstance
    set_io_float(ID_T_TEST4,0.0); //setting this to 1 removes push-off completely. no use of T_TEST2 in that case
    set_io_float(ID_T_TEST5,0.0); //flag to indicate start. 0 means don't start.
  	return 1;
 }
 
  int ACTION_top_walk_fast(void)
{
    float rc_command = get_io_float(ID_UI_RC_1)/52000 - 1.1; //A number between 0 and 1
    set_io_float(ID_A_T_RC1_NORM,rc_command);
  	return 1;
 }

 int ACTION_top_walk_slow_entry(void)
{
    set_io_float(ID_T_TEST1,0.7); //hip swing modulation 0.6 - 1.2 (prop to velocity controller)
    set_io_float(ID_T_TEST2,0.7); //push-off modulation 0.6 - 1.2
    set_io_float(ID_T_TEST3,0.0); //flag to indicate stop. 0 means don't stop.
                                  //hip go to stophold and feet go to stopstance
    set_io_float(ID_T_TEST4,0.0); //setting this to 1 removes push-off completely. no use of T_TEST2 in that case
    set_io_float(ID_T_TEST5,0.0); //flag to indicate start. 0 means don't start.
  	return 1;
 }

  int ACTION_top_walk_slow(void)
{
    float rc_command = get_io_float(ID_UI_RC_1)/52000 - 1.1; //A number between 0 and 1
    set_io_float(ID_A_T_RC1_NORM,rc_command);
  	return 1;
 }


 int ACTION_top_temp(void)
{
   
    float rc_command = get_io_float(ID_UI_RC_1)/52000 - 1.1; //A number between 0 and 1
    set_io_float(ID_A_T_RC1_NORM,rc_command);
    
    return 1;    
}



int ACTION_top_stop(void)
{

  return 1;
}
