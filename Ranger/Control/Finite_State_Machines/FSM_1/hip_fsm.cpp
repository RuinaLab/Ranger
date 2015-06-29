//#include <iostream>
#include <mb_includes.h>
#include "fsm.h"
#include "hip_fsm_conditions.h"
#include "hip_fsm.h"
#include "global_sensors.h"
#include "global_params.h"
#include "global_communications.h"



int ACTION_HIP_entry_inner_free(void)
{
  set_io_float(ID_MCH_COMMAND_CURRENT, 0.0);
  return 1;
}

int ACTION_HIP_inner_free(void)
{
 // set_io_float(ID_MCH_COMMAND_CURRENT, 0.0);
  return 1;
}

int ACTION_HIP_exit_inner_free(void)
{
  set_io_float(ID_MCH_COMMAND_CURRENT, 0.0);
  return 1;
}

int ACTION_HIP_entry_outer_swing(void)
{
  set_io_float(ID_MCH_COMMAND_CURRENT, get_io_float(ID_MB_HIP_FSM_OUTER_SWING_I));
  return 1;
}

int ACTION_HIP_outer_swing(void)
{
 // set_io_float(ID_MCH_COMMAND_CURRENT, get_io_float(ID_MB_HIP_FSM_OUTER_SWING_I));
  return 1;
}

int ACTION_HIP_exit_outer_swing(void)
{
  set_io_float(ID_MCH_COMMAND_CURRENT, 0.0);
  return 1;
}

int ACTION_HIP_entry_outer_free(void)
{
  set_io_float(ID_MCH_COMMAND_CURRENT, 0.0);
  return 1;
}

int ACTION_HIP_outer_free(void)
{
//  set_io_float(ID_MCH_COMMAND_CURRENT, 0.0);
  return 1;
}

int ACTION_HIP_exit_outer_free(void)
{
  set_io_float(ID_MCH_COMMAND_CURRENT, 0.0);
  return 1;
}

int ACTION_HIP_entry_inner_swing(void)
{
  set_io_float(ID_MCH_COMMAND_CURRENT, get_io_float(ID_MB_HIP_FSM_INNER_SWING_I));
  return 1;
}

int ACTION_HIP_inner_swing(void)
{
//  set_io_float(ID_MCH_COMMAND_CURRENT, get_io_float(ID_MB_HIP_FSM_INNER_SWING_I));
  return 1;
}

int ACTION_HIP_exit_inner_swing(void)
{
  set_io_float(ID_MCH_COMMAND_CURRENT, 0.0);
  return 1;
}

int ACTION_HIP_stop(void)
{
  set_io_float(ID_MCH_MOTOR_TARGET_CURRENT, 0.0);
  return 1;
}


void def_hip_fsm(fsm* h_fsm)
{

//cout << endl;
//cout << "defining hip_fsm..  " << endl;

// numStates is the number of states defined in structure.h
int numStates = 5;
// numInpts is the number of inputs defined in structure.h
int numInputs = 5;

// construct the fsm

// define start state
fsm fsm1(numStates, STATE_HIP_inner_free, numInputs);

char* name = (char*) "hip_fsm";
fsm1.set_name(name);

//debug
fsm1.print_state_transition_matrix();

// associate the states with actions

fsm1.state_def(STATE_HIP_inner_free, ACTION_HIP_entry_inner_free, ACTION_HIP_inner_free, ACTION_HIP_exit_inner_free);
fsm1.state_def(STATE_HIP_inner_swing, ACTION_HIP_entry_inner_swing, ACTION_HIP_inner_swing, ACTION_HIP_exit_inner_swing);
fsm1.state_def(STATE_HIP_outer_free, ACTION_HIP_entry_outer_free, ACTION_HIP_outer_free, ACTION_HIP_exit_outer_free);
fsm1.state_def(STATE_HIP_outer_swing, ACTION_HIP_entry_outer_swing, ACTION_HIP_outer_swing, ACTION_HIP_exit_outer_swing);
fsm1.state_def(STATE_HIP_stop, ACTION_HIP_stop, ACTION_HIP_stop, ACTION_HIP_stop);


fsm1.print_state_transition_matrix();

// define the transitions
// a transition needs to be defined for every state for every input
// for the sensors which dont make a difference it should stay in the same state

fsm1.trans_def(STATE_HIP_inner_free, COND_HIP_inner_free_to_outer_swing, STATE_HIP_outer_swing);
fsm1.trans_def(STATE_HIP_outer_swing, COND_HIP_outer_swing_to_outer_free, STATE_HIP_outer_free);
fsm1.trans_def(STATE_HIP_outer_free, COND_HIP_outer_free_to_inner_swing, STATE_HIP_inner_swing);
fsm1.trans_def(STATE_HIP_inner_swing, COND_HIP_inner_swing_to_inner_free, STATE_HIP_inner_free);
fsm1.trans_def(STATE_HIP_inner_free, COND_HIP_stop, STATE_HIP_stop);
fsm1.trans_def(STATE_HIP_outer_swing, COND_HIP_stop, STATE_HIP_stop);
fsm1.trans_def(STATE_HIP_outer_free, COND_HIP_stop, STATE_HIP_stop);
fsm1.trans_def(STATE_HIP_inner_swing, COND_HIP_stop, STATE_HIP_stop);

fsm1.print_state_transition_matrix();


// define exit state of fsm
fsm1.exit_state_def(STATE_HIP_stop);

fsm1.set_sensor_input_function(get_hip_sensor_input);

fsm1.set_state_communication_variable(&g_hip_fsm_state);

//set_io_float(ID_MB_HIP_FSM_STATE, (float)g_hip_fsm_state);

copy(h_fsm, &fsm1);

}
