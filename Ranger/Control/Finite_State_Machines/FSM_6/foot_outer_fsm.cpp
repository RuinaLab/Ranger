#include <mb_includes.h>
//#include <iostream>
#include "fsm.h"
#include "foot_outer_fsm.h"
#include "foot_outer_fsm_conditions.h"
#include "global_params.h"
#include "global_sensors.h"
#include "global_communications.h"


float fo_counts_flipup, fo_counts_flipdown, fo_counts_stance;

/*float FO_command_current_limiter(float command_current, float I_lim)
{

float PDval = get_io_float(ID_MCFO_STIFFNESS)*get_io_float(ID_MCFO_MOTOR_POSITION)
              +get_io_float(ID_MCFO_DAMPNESS)*get_io_float(ID_MCFO_MOTOR_VELOCITY);
float I = command_current - PDval;
                      
 if (I_lim > 0) //check for upper bound
    {
    if (I>I_lim) //upper bound reached?
      {command_current = I_lim + PDval;} //change upper bound
    }
 else if (I_lim < 0) //check for lower bound
    {
    if (I<I_lim) //lower bound reached?
      {command_current = I_lim + PDval;} //change lower bound
    }  
      
 return command_current;     
      
}*/

int ACT_FO_stance_entry()
{
 fo_counts_stance = 0;
 //set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_C_F_ST_F_ANG)); 
 //set_io_float(ID_MCFO_DAMPNESS, get_io_float(ID_C_F_ST_F_RATE)); 
 
  set_io_float(ID_MCFO_STIFFNESS, 0.0); 
  set_io_float(ID_MCFO_DAMPNESS, 0.0); 
  set_io_float(ID_MCFO_COMMAND_CURRENT, 0.0);
 
  return 1;
}

int ACT_FO_stance()
{
  float target_angle;
  
  fo_counts_stance = fo_counts_stance+2;
  
  set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_C_F_ST_F_ANG)); 
  set_io_float(ID_MCFO_DAMPNESS, get_io_float(ID_C_F_ST_F_RATE)); 
  
  target_angle =  get_io_float(ID_P_F_ST_F_TANG); //Temp 5/3/2010 for feet asymmetry testing

  //float command_current = FO_command_current_limiter(target_angle * get_io_float(ID_MCFO_STIFFNESS), 1.0);
  //set_io_float(ID_MCFO_COMMAND_CURRENT, command_current);
  set_io_float(ID_MCFO_COMMAND_CURRENT, target_angle * get_io_float(ID_MCFO_STIFFNESS)); 
  
  //set_io_float(ID_MCFO_STIFFNESS, 0.0); 
  //set_io_float(ID_MCFO_DAMPNESS, 0.0); 
  //set_io_float(ID_MCFO_COMMAND_CURRENT, 0.0);
  
  

  return 1;
}


int ACT_FO_flipup_entry()
{ 
   fo_counts_flipup = 0;
   set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_C_F_FU_F_ANG)); 
   set_io_float(ID_MCFO_DAMPNESS, get_io_float(ID_C_F_FU_F_RATE)); 

  return 1;
}

int ACT_FO_flipup()
{ 
   float target_angle;
  
   fo_counts_flipup = fo_counts_flipup+ 2;
  
  // ******* PD on position starts *******//
   target_angle =  get_io_float(ID_P_F_FU_F_TANG);

   //float current_limit =  -1.0; //get_io_float(ID_F_TEST1)*(-get_io_float(ID_F_TEST3)-get_io_float(ID_MCFO_MOTOR_VELOCITY));
   //float command_current = FO_command_current_limiter(target_angle * get_io_float(ID_MCFO_STIFFNESS), current_limit);
   float command_current = target_angle * get_io_float(ID_MCFO_STIFFNESS);
   set_io_float(ID_MCFO_COMMAND_CURRENT, command_current-get_io_float(ID_F_TEST1) );
   // ******* PD on position ends *********


  
  
  return 1;
}

int ACT_FO_flipdown_entry()
{ 
fo_counts_flipdown = 0;

  set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_C_F_FD_F_ANG)); 
  set_io_float(ID_MCFO_DAMPNESS, get_io_float(ID_C_F_FD_F_RATE));  
  return 1;
}

int ACT_FO_flipdown()
{ 
  float target_angle;
  
  fo_counts_flipdown = fo_counts_flipdown+2;

    target_angle =  get_io_float(ID_P_F_FD_F_TANG); 
    
    //float current_limit = 1.0; //get_io_float(ID_F_TEST2)*(get_io_float(ID_F_TEST3)-get_io_float(ID_MCFO_MOTOR_VELOCITY));
    //float command_current = FO_command_current_limiter(target_angle * get_io_float(ID_MCFO_STIFFNESS), current_limit);
    float command_current = target_angle * get_io_float(ID_MCFO_STIFFNESS);
    set_io_float(ID_MCFO_COMMAND_CURRENT, command_current+get_io_float(ID_F_TEST2));
    //set_io_float(ID_MCFO_COMMAND_CURRENT, target_angle * get_io_float(ID_MCFO_STIFFNESS));
  // ******* PD on position end *******//

  
  return 1;
}

int ACT_FO_stop()
{  
  set_io_float(ID_MCFO_COMMAND_CURRENT, 0.0);
  set_io_float(ID_MCFO_STIFFNESS, 0.0);
  set_io_float(ID_MCFO_DAMPNESS, 0.0);
  return 1;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////



void def_foot_outer_fsm(fsm* fo_fsm)
{

//cout << "defining ankle fsm.. " << endl;

// numStates is the number of states defined in structure.h
int numStates = 4;
// numInpts is the number of inputs defined in structure.h
int numInputs = 4;

// construct the fsm
fsm fsm1(numStates, STATE_FO_flipup, numInputs);

char* name = (char*) "foot_outer_fsm";
fsm1.set_name(name);

// associate the states with actions

fsm1.state_def(STATE_FO_stance, ACT_FO_stance_entry, ACT_FO_stance, ACT_FO_stance);
fsm1.state_def(STATE_FO_flipup, ACT_FO_flipup_entry, ACT_FO_flipup, ACT_FO_flipup);
fsm1.state_def(STATE_FO_flipdown, ACT_FO_flipdown_entry, ACT_FO_flipdown, ACT_FO_flipdown);
fsm1.state_def(STATE_FO_stop, ACT_FO_stop, ACT_FO_stop, ACT_FO_stop);



fsm1.print_state_transition_matrix();

// define the transitions
// a transition needs to be defined for every state for every input
// for the sensors which dont make a difference it should stay in the same state

fsm1.trans_def(STATE_FO_stance, COND_FO_stance_to_flipup, STATE_FO_flipup);
fsm1.trans_def(STATE_FO_flipup, COND_FO_flipup_to_flipdown, STATE_FO_flipdown);
fsm1.trans_def(STATE_FO_flipdown, COND_FO_flipdown_to_stance, STATE_FO_stance);
fsm1.trans_def(STATE_FO_stance, COND_FO_stop, STATE_FO_stop);
fsm1.trans_def(STATE_FO_flipup, COND_FO_stop, STATE_FO_stop);
fsm1.trans_def(STATE_FO_flipdown, COND_FO_stop, STATE_FO_stop);

fsm1.print_state_transition_matrix();


// define exit state of fsm
fsm1.exit_state_def(STATE_FO_stop);

fsm1.set_sensor_input_function(get_foot_outer_input);

fsm1.set_state_communication_variable(&g_foot_outer_fsm_state);

//set_io_float(ID_MB_FOOT_OUTER_FSM_STATE, (float)g_foot_outer_fsm_state);

copy(fo_fsm, &fsm1);

}

