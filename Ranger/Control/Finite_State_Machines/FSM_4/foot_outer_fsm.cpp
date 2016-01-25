#include <mb_includes.h>
//#include <iostream>
#include "fsm.h"
#include "foot_outer_fsm.h"
#include "foot_outer_fsm_conditions.h"
#include "global_params.h"
#include "global_sensors.h"
#include "global_communications.h"

//using namespace std;

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Define action functions
int counter_FO;

int ACT_FO_free()
{  
  set_io_float(ID_MCFO_COMMAND_CURRENT, 0.0);
  set_io_float(ID_MCFO_STIFFNESS, 0.0);
  set_io_float(ID_MCFO_DAMPNESS, 0.0);
  return 1;
}

int ACT_FO_stance_entry()
{
  counter_FO = 0;
  
  return 1;
}


int ACT_FO_stance()
{
  float target_angle;
  
  counter_FO = counter_FO + 1;
  
  set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_C_F_ST_F_ANG)); //TEMP , 4/6/2010
  set_io_float(ID_MCFO_DAMPNESS, get_io_float(ID_C_F_ST_F_RATE)); //TEMP , 4/6/2010
  
  
  target_angle =  get_io_float(ID_P_F_ST_F_TANG) + get_io_float(ID_MCH_ANGLE) * 0.5;

//  if ((get_io_float(ID_MCFO_MID_ANKLE_ANGLE) - target_angle) > 0.1) //Ankle cable slackness safety
//  {
//     target_angle = get_io_float(ID_MCFO_MID_ANKLE_ANGLE) - 0.1;
//  }
  
  set_io_float(ID_MCFO_COMMAND_CURRENT, target_angle * get_io_float(ID_MCFO_STIFFNESS)); 
  return 1;
}

int ACT_FO_flipup_entry()
{
  counter_FO = 0;
  
  return 1;
}


int ACT_FO_flipup()
{ 
  float target_angle;
  
  counter_FO = counter_FO + 1;
  
  set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_C_F_FU_F_ANG)); //TEMP , 4/6/2010
  set_io_float(ID_MCFO_DAMPNESS, get_io_float(ID_C_F_FU_F_RATE)); //TEMP , 4/6/2010
  
  target_angle =  get_io_float(ID_P_F_FU_F_TANG);
  set_io_float(ID_MCFO_COMMAND_CURRENT, target_angle * get_io_float(ID_MCFO_STIFFNESS));
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
int numInputs = 5;

// construct the fsm
fsm fsm1(numStates, STATE_FO_free, numInputs);

char* name = (char*) "foot_outer_fsm";
fsm1.set_name(name);

// associate the states with actions

fsm1.state_def(STATE_FO_free, ACT_FO_free, ACT_FO_free, ACT_FO_free);
fsm1.state_def(STATE_FO_stance, ACT_FO_stance_entry, ACT_FO_stance, ACT_FO_stance);
fsm1.state_def(STATE_FO_flipup, ACT_FO_flipup_entry, ACT_FO_flipup, ACT_FO_flipup);
fsm1.state_def(STATE_FO_stop, ACT_FO_stop, ACT_FO_stop, ACT_FO_stop);


fsm1.print_state_transition_matrix();

// define the transitions
// a transition needs to be defined for every state for every input
// for the sensors which dont make a difference it should stay in the same state

fsm1.trans_def(STATE_FO_free, COND_FO_free_to_stance, STATE_FO_stance);
fsm1.trans_def(STATE_FO_free, COND_FO_free_to_flipup, STATE_FO_flipup);  
fsm1.trans_def(STATE_FO_flipup, COND_FO_flipup_to_free, STATE_FO_free);  
fsm1.trans_def(STATE_FO_stance, COND_FO_stance_to_free, STATE_FO_free);

fsm1.print_state_transition_matrix();


// define exit state of fsm
fsm1.exit_state_def(STATE_FO_stop);

fsm1.set_sensor_input_function(get_foot_outer_input);

fsm1.set_state_communication_variable(&g_foot_outer_fsm_state);

//set_io_float(ID_MB_FOOT_OUTER_FSM_STATE, (float)g_foot_outer_fsm_state);

copy(fo_fsm, &fsm1);

}

