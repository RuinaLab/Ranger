
//#include <iostream>
#include <mb_includes.h>
#include "fsm.h"
#include "system_init_fsm_conditions.h"
#include "system_init_fsm.h"
#include "global_sensors.h"
#include "global_params.h"
#include "global_communications.h"

///////////////////////////////////////////////////////////////////////////////////////////////////
//Action functions
int ACTION_SI_system_start(void)
{
  set_io_float(ID_MB_STATUS, 1);
  return 1;
}

int ACTION_SI_system_run(void)
{
  set_io_float(ID_MB_STATUS, 2);
  return 1;
}

int ACTION_SI_stop(void)
{
  return 1;
}
///////////////////////////////////////////////////////////////////////////////////////////////////

void def_system_init_fsm(fsm* h_fsm)
{

//cout << endl;
//cout << "defining hip_fsm..  " << endl;

// numStates is the number of states defined in structure.h
int numStates = 3;
// numInpts is the number of inputs defined in structure.h
int numInputs = 2;

// construct the fsm
fsm fsm1(numStates, STATE_SI_system_start, numInputs);

char* name = (char*) "system_init_fsm";
fsm1.set_name(name);

//debug
fsm1.print_state_transition_matrix();

///////////////////////////////////////////////////////////////////////////////////////////////////
// associate the states with actions

fsm1.state_def(STATE_SI_system_start, ACTION_SI_system_start, ACTION_SI_system_start, ACTION_SI_system_start);
fsm1.state_def(STATE_SI_system_run, ACTION_SI_system_run, ACTION_SI_system_run, ACTION_SI_system_run);
fsm1.state_def(STATE_SI_stop, ACTION_SI_stop, ACTION_SI_stop, ACTION_SI_stop);

///////////////////////////////////////////////////////////////////////////////////////////////////

fsm1.print_state_transition_matrix();

///////////////////////////////////////////////////////////////////////////////////////////////////
// define the transitions
// a transition needs to be defined for every state for every input
// for the sensors which dont make a difference it should stay in the same state

fsm1.trans_def(STATE_SI_system_start, COND_SI_all_ready, STATE_SI_system_run);
fsm1.trans_def(STATE_SI_system_run, COND_SI_stop, STATE_SI_stop);

///////////////////////////////////////////////////////////////////////////////////////////////////

fsm1.print_state_transition_matrix();

fsm1.exit_state_def(STATE_SI_stop);

fsm1.set_sensor_input_function(get_system_init_conditions_input);

fsm1.set_state_communication_variable(&g_system_init_fsm_state);
//set_io_float(ID_MB_SYSTEM_INIT_FSM_STATE, (float)g_system_init_fsm_state);

copy(h_fsm, &fsm1);

}
