
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
int counter_FI;

int ACT_FI_free()
{  
  set_io_float(ID_MCFI_COMMAND_CURRENT, 0.0);
  set_io_float(ID_MCFI_STIFFNESS, 0.0);
  set_io_float(ID_MCFI_DAMPNESS, 0.0);
  return 1;
}


int ACT_FI_stance_entry()
{
  counter_FI = 0;
  
  return 1;
}


int ACT_FI_stance()
{
  float target_angle;
  
  counter_FI = counter_FI + 1;
  
  set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_ST_F_ANG)); //TEMP 4/6/2010
  set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_ST_F_RATE)); //TEMP 4/6/2010
  
  target_angle =  get_io_float(ID_P_F_ST_F_TANG) + get_io_float(ID_MCH_ANGLE) * 0.5;

//  if ((get_io_float(ID_MCFI_MID_ANKLE_ANGLE) - target_angle) > 0.1) //Ankle cable slackness safety
//  {
//     target_angle = get_io_float(ID_MCFI_MID_ANKLE_ANGLE) - 0.1;
//  }
  
  set_io_float(ID_MCFI_COMMAND_CURRENT, target_angle * get_io_float(ID_MCFI_STIFFNESS)); 
  return 1;
}


int ACT_FI_flipup_entry()
{
  counter_FI = 0;
  
  return 1;
}

int ACT_FI_flipup()
{ 
  float target_angle;
  
  counter_FI = counter_FI + 1;
  
   set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_FU_F_ANG)); //TEMP , 4/6/2010
   set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_FU_F_RATE)); //TEMP , 4/6/2010
  
  target_angle =  get_io_float(ID_P_F_FU_F_TANG);
  set_io_float(ID_MCFI_COMMAND_CURRENT, target_angle * get_io_float(ID_MCFI_STIFFNESS));
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
int numInputs = 5;

// construct the fsm
fsm fsm1(numStates, STATE_FI_free, numInputs);

char* name = (char*) "foot_inner_fsm";
fsm1.set_name(name);

// associate the states with actions

fsm1.state_def(STATE_FI_free, ACT_FI_free, ACT_FI_free, ACT_FI_free);
fsm1.state_def(STATE_FI_stance, ACT_FI_stance_entry, ACT_FI_stance, ACT_FI_stance);
fsm1.state_def(STATE_FI_flipup, ACT_FI_flipup_entry, ACT_FI_flipup, ACT_FI_flipup);
fsm1.state_def(STATE_FI_stop, ACT_FI_stop, ACT_FI_stop, ACT_FI_stop);


fsm1.print_state_transition_matrix();

// define the transitions
// a transition needs to be defined for every state for every input
// for the sensors which dont make a difference it should stay in the same state

fsm1.trans_def(STATE_FI_free, COND_FI_free_to_stance, STATE_FI_stance);
fsm1.trans_def(STATE_FI_free, COND_FI_free_to_flipup, STATE_FI_flipup);   
fsm1.trans_def(STATE_FI_flipup, COND_FI_flipup_to_free, STATE_FI_free);  
fsm1.trans_def(STATE_FI_stance, COND_FI_stance_to_free, STATE_FI_free);  

fsm1.print_state_transition_matrix();


// define exit state of fsm
fsm1.exit_state_def(STATE_FI_stop);

fsm1.set_sensor_input_function(get_foot_inner_input);

fsm1.set_state_communication_variable(&g_foot_inner_fsm_state);

//set_io_float(ID_MB_FOOT_INNER_FSM_STATE, (float)g_foot_inner_fsm_state);

copy(fi_fsm, &fsm1);

}

