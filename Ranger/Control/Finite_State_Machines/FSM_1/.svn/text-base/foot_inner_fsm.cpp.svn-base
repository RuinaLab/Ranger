
//#include <iostream>
#include <mb_includes.h>
#include "fsm.h"
#include "foot_inner_fsm.h"
#include "foot_inner_fsm_conditions.h"
#include "global_params.h"
#include "global_sensors.h"
#include "global_communications.h"

//using namespace std;

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Define action functions

int ACTION_FOOT_INNER_entry_stance()
{
  set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_MB_FOOT_INNER_STANCE_KP));
  set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_MB_FOOT_INNER_STANCE_KD));
  return 1;
}

int ACTION_FOOT_INNER_stance()
{
  float target_angle;
  
  target_angle =  get_io_float(ID_MB_FOOT_INNER_STANCE_ANG) + get_io_float(ID_MCH_ANGLE) * 0.5;

  if ((get_io_float(ID_MCFI_MID_ANKLE_ANGLE) - target_angle) > 0.1)
  {
     target_angle = get_io_float(ID_MCFI_MID_ANKLE_ANGLE) - 0.1;
  }
  
  set_io_float(ID_MCFI_COMMAND_CURRENT, target_angle * get_io_float(ID_MCFI_STIFFNESS)); 
  return 1;
}

int ACTION_FOOT_INNER_pushoff()
{ 
  float target_angle;
  
  target_angle =  get_io_float(ID_MB_FOOT_INNER_STANCE_ANG) + get_io_float(ID_MCH_ANGLE) * 0.5;

  if ((get_io_float(ID_MCFI_MID_ANKLE_ANGLE) - target_angle) > 0.1)
  {
     target_angle = get_io_float(ID_MCFI_MID_ANKLE_ANGLE) - 0.1;
  }
  
  set_io_float(ID_MCFI_COMMAND_CURRENT, target_angle * get_io_float(ID_MCFI_STIFFNESS));
  return 1; 
}

int ACTION_FOOT_INNER_entry_flipup()
{ 
  set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_MB_FOOT_INNER_FLIPUP_KP));
  set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_MB_FOOT_INNER_FLIPUP_KD));
  return 1;
}

int ACTION_FOOT_INNER_flipup()
{ 
  float target_angle;
  
  target_angle =  get_io_float(ID_MB_FOOT_INNER_FLIPUP_ANG);

//  if ((get_io_float(ID_MCFI_MID_ANKLE_ANGLE) - target_angle) > 0.1)
//  {
//     target_angle = get_io_float(ID_MCFI_MID_ANKLE_ANGLE) - 0.1;
//  }
  
  set_io_float(ID_MCFI_COMMAND_CURRENT, target_angle * get_io_float(ID_MCFI_STIFFNESS));
  return 1;
}

int ACTION_FOOT_INNER_entry_flipdown()
{ 
  set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_MB_FOOT_INNER_FLIPDOWN_KP));
  set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_MB_FOOT_INNER_FLIPDOWN_KD));  
  return 1;
}

int ACTION_FOOT_INNER_flipdown()
{ 
  float target_angle;
  
  target_angle =  get_io_float(ID_MB_FOOT_INNER_FLIPDOWN_ANG) + get_io_float(ID_MCH_ANGLE) * 0.5;

  if ((get_io_float(ID_MCFI_MID_ANKLE_ANGLE) - target_angle) > 0.1)
  {
     target_angle = get_io_float(ID_MCFI_MID_ANKLE_ANGLE) - 0.1;
  }
  
  set_io_float(ID_MCFI_COMMAND_CURRENT, target_angle * get_io_float(ID_MCFI_STIFFNESS));
  return 1;
}

int ACTION_FOOT_INNER_stop()
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
int numStates = 5;
// numInpts is the number of inputs defined in structure.h
int numInputs = 5;

// construct the fsm
fsm fsm1(numStates, STATE_FOOT_INNER_flipdown, numInputs);

char* name = (char*) "foot_inner_fsm";
fsm1.set_name(name);

// associate the states with actions

fsm1.state_def(STATE_FOOT_INNER_stance, ACTION_FOOT_INNER_entry_stance, ACTION_FOOT_INNER_stance, ACTION_FOOT_INNER_stance);
fsm1.state_def(STATE_FOOT_INNER_pushoff, ACTION_FOOT_INNER_pushoff, ACTION_FOOT_INNER_pushoff, ACTION_FOOT_INNER_pushoff);
fsm1.state_def(STATE_FOOT_INNER_flipup, ACTION_FOOT_INNER_entry_flipup, ACTION_FOOT_INNER_flipup, ACTION_FOOT_INNER_flipup);
fsm1.state_def(STATE_FOOT_INNER_flipdown, ACTION_FOOT_INNER_entry_flipdown, ACTION_FOOT_INNER_flipdown, ACTION_FOOT_INNER_flipdown);
fsm1.state_def(STATE_FOOT_INNER_stop, ACTION_FOOT_INNER_stop, ACTION_FOOT_INNER_stop, ACTION_FOOT_INNER_stop);


fsm1.print_state_transition_matrix();

// define the transitions
// a transition needs to be defined for every state for every input
// for the sensors which dont make a difference it should stay in the same state

fsm1.trans_def(STATE_FOOT_INNER_stance, COND_FOOT_INNER_stance_to_pushoff, STATE_FOOT_INNER_pushoff);
fsm1.trans_def(STATE_FOOT_INNER_pushoff, COND_FOOT_INNER_pushoff_to_flipup, STATE_FOOT_INNER_flipup);
fsm1.trans_def(STATE_FOOT_INNER_flipup, COND_FOOT_INNER_flipup_to_flipdown, STATE_FOOT_INNER_flipdown);
fsm1.trans_def(STATE_FOOT_INNER_flipdown, COND_FOOT_INNER_flipdown_to_stance, STATE_FOOT_INNER_stance);
fsm1.trans_def(STATE_FOOT_INNER_stance, COND_FOOT_INNER_stop, STATE_FOOT_INNER_stop);
fsm1.trans_def(STATE_FOOT_INNER_pushoff, COND_FOOT_INNER_stop, STATE_FOOT_INNER_stop);
fsm1.trans_def(STATE_FOOT_INNER_flipup, COND_FOOT_INNER_stop, STATE_FOOT_INNER_stop);
fsm1.trans_def(STATE_FOOT_INNER_flipdown, COND_FOOT_INNER_stop, STATE_FOOT_INNER_stop);

fsm1.print_state_transition_matrix();


// define exit state of fsm
fsm1.exit_state_def(STATE_FOOT_INNER_stop);

fsm1.set_sensor_input_function(get_foot_inner_input);

fsm1.set_state_communication_variable(&g_foot_inner_fsm_state);

//set_io_float(ID_MB_FOOT_INNER_FSM_STATE, (float)g_foot_inner_fsm_state);

copy(fi_fsm, &fsm1);

}
