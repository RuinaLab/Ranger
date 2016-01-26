
//#include <iostream>
#include <mb_includes.h>
#include "fsm.h"
#include "foot_inner_fsm.h"
#include "foot_inner_fsm_conditions.h"
#include "global_params.h"
#include "global_sensors.h"
#include "global_communications.h"


float fi_counts_flipup, fi_counts_flipdown, fi_counts_stance;

/*float FI_command_current_limiter(float command_current, float I_lim)
{

float PDval = get_io_float(ID_MCFI_STIFFNESS)*get_io_float(ID_MCFI_MOTOR_POSITION)
              +get_io_float(ID_MCFI_DAMPNESS)*get_io_float(ID_MCFI_MOTOR_VELOCITY);
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

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Define action functions

int ACT_FI_stance_entry()
{
  fi_counts_stance = 0; //Initialize counter
  
//  set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_ST_F_ANG)); 
//  set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_ST_F_RATE)); 
  set_io_float(ID_MCFI_STIFFNESS, 0.0); 
  set_io_float(ID_MCFI_DAMPNESS, 0.0); 
  
  return 1;
}

int ACT_FI_stance()
{
  float target_angle;
  
  fi_counts_stance = fi_counts_stance+2; //Increment by 2 as this function is called once every 2 ms.
  
  // Already set in ACT_FI_stance_entry
  set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_ST_F_ANG)); 
  set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_ST_F_RATE)); 
  
   target_angle =  get_io_float(ID_P_F_ST_F_TANG); 
  
  
  //float command_current = FI_command_current_limiter(target_angle * get_io_float(ID_MCFI_STIFFNESS), 1.0); //Current limit of 1 A
  //set_io_float(ID_MCFI_COMMAND_CURRENT, command_current);
  set_io_float(ID_MCFI_COMMAND_CURRENT, target_angle * get_io_float(ID_MCFI_STIFFNESS)); //If don't want current limiter
  
  //set_io_float(ID_MCFI_STIFFNESS, 0.0); 
  //set_io_float(ID_MCFI_DAMPNESS, 0.0); 
  //set_io_float(ID_MCFI_COMMAND_CURRENT, 0.0);
  
  
  return 1;
}


int ACT_FI_flipup_entry()
{ 
 fi_counts_flipup = 0;
 
 set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_FU_F_ANG)); 
 set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_FU_F_RATE)); 

  return 1; 
}

int ACT_FI_flipup()
{ 
  float target_angle;
  
   fi_counts_flipup = fi_counts_flipup+2;
  

   // ******* PD on position starts *******//
   set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_FU_F_ANG)); 
   set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_FU_F_RATE)); 
  
   target_angle =  get_io_float(ID_P_F_FU_F_TANG); 

   // float current_limit =   -1.0; // get_io_float(ID_F_TEST1)*(-get_io_float(ID_F_TEST3)-get_io_float(ID_MCFI_MOTOR_VELOCITY));
   //float command_current = FI_command_current_limiter(target_angle * get_io_float(ID_MCFI_STIFFNESS), current_limit);
   float command_current = target_angle * get_io_float(ID_MCFI_STIFFNESS);
   set_io_float(ID_MCFI_COMMAND_CURRENT, command_current-get_io_float(ID_F_TEST3));
  // ******* PD on position ends *******//

  
  return 1;
}

int ACT_FI_flipdown_entry()
{ 

 fi_counts_flipdown = 0;

 set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_FD_F_ANG)); 
 set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_FD_F_RATE)); 
 
  return 1;
}

int ACT_FI_flipdown()
{ 
  float target_angle;
  
  fi_counts_flipdown = fi_counts_flipdown+2;
  

    // ******* PD on position starts *******//
   set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_FD_F_ANG)); 
   set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_FD_F_RATE)); 
   
   target_angle =  get_io_float(ID_P_F_FD_F_TANG); //Control on relative angle
      
    // float current_limit =  1.0; //get_io_float(ID_F_TEST2)*(get_io_float(ID_F_TEST3)-get_io_float(ID_MCFI_MOTOR_VELOCITY));  
    //float command_current = FI_command_current_limiter(target_angle * get_io_float(ID_MCFI_STIFFNESS), current_limit);
    
    float command_current = target_angle * get_io_float(ID_MCFI_STIFFNESS);
    set_io_float(ID_MCFI_COMMAND_CURRENT, command_current+get_io_float(ID_F_TEST4));
    // ******* PD on position end *******// 

  return 1;
}

int ACT_FI_stop()
{  
  set_io_float(ID_MCFI_COMMAND_CURRENT, 0.0);
  set_io_float(ID_MCFI_STIFFNESS, 0.0);
  set_io_float(ID_MCFI_DAMPNESS, 0.0);
  return 1;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////


void def_foot_inner_fsm(fsm* fi_fsm)
{

//cout << "defining ankle fsm.. " << endl;

// numStates is the number of states defined in structure.h
int numStates = 4;
// numInpts is the number of inputs defined in structure.h
int numInputs = 4;

// construct the fsm
fsm fsm1(numStates, STATE_FI_flipup, numInputs);

char* name = (char*) "foot_inner_fsm";
fsm1.set_name(name);

// associate the states with actions

fsm1.state_def(STATE_FI_stance, ACT_FI_stance_entry, ACT_FI_stance, ACT_FI_stance);
fsm1.state_def(STATE_FI_flipup, ACT_FI_flipup_entry, ACT_FI_flipup, ACT_FI_flipup);
fsm1.state_def(STATE_FI_flipdown, ACT_FI_flipdown_entry, ACT_FI_flipdown, ACT_FI_flipdown);
fsm1.state_def(STATE_FI_stop, ACT_FI_stop, ACT_FI_stop, ACT_FI_stop);


fsm1.print_state_transition_matrix();

// define the transitions
// a transition needs to be defined for every state for every input
// for the sensors which dont make a difference it should stay in the same state

fsm1.trans_def(STATE_FI_stance, COND_FI_stance_to_flipup, STATE_FI_flipup);
fsm1.trans_def(STATE_FI_flipup, COND_FI_flipup_to_flipdown, STATE_FI_flipdown);
fsm1.trans_def(STATE_FI_flipdown, COND_FI_flipdown_to_stance, STATE_FI_stance);
fsm1.trans_def(STATE_FI_stance, COND_FI_stop, STATE_FI_stop);
fsm1.trans_def(STATE_FI_flipup, COND_FI_stop, STATE_FI_stop);
fsm1.trans_def(STATE_FI_flipdown, COND_FI_stop, STATE_FI_stop);

fsm1.print_state_transition_matrix();


// define exit state of fsm
fsm1.exit_state_def(STATE_FI_stop);

fsm1.set_sensor_input_function(get_foot_inner_input);

fsm1.set_state_communication_variable(&g_foot_inner_fsm_state);

//set_io_float(ID_MB_FOOT_INNER_FSM_STATE, (float)g_foot_inner_fsm_state);

copy(fi_fsm, &fsm1);

}

