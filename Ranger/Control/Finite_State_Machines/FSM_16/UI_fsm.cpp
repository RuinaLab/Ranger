//extern "C"{
#include <mb_includes.h>
//}
#include "fsm.h"
#include "global_communications.h"

#include "ui_fsm_conditions.h"
#include "ui_fsm_actions.h"

///////////////////////////////////////////////////////////////////////////////////////////////////

void def_UI_fsm(fsm* h_fsm)
{

// numStates is the number of states in state machine
int numStates = 6;
// numInpts is the number of condition in state machine
int numInputs = 8;

// construct the fsm
fsm fsm1(numStates, STATE_UI_yawen, numInputs);

char* name = (char*) "ui_fsm";
fsm1.set_name(name);

//debug
fsm1.print_state_transition_matrix();

///////////////////////////////////////////////////////////////////////////////////////////////////
// associate the states with actions

/***********YW: new state**************/
fsm1.state_def(STATE_UI_yawen, ACTION_UI_yawen_entry, ACTION_UI_yawen, ACTION_UI_yawen);

fsm1.state_def(STATE_UI_calibrate, ACTION_UI_calibrate_entry, ACTION_UI_calibrate, ACTION_UI_calibrate);
fsm1.state_def(STATE_UI_standby, ACTION_UI_standby_entry, ACTION_UI_standby, ACTION_UI_standby_exit);
fsm1.state_def(STATE_UI_walk, ACTION_UI_walk_entry, ACTION_UI_walk, ACTION_UI_walk_exit);
fsm1.state_def(STATE_UI_stop, ACTION_UI_stop, ACTION_UI_stop, ACTION_UI_stop);
fsm1.state_def(STATE_UI_nogo, ACTION_UI_nogo_entry, ACTION_UI_nogo, ACTION_UI_nogo_exit);

///////////////////////////////////////////////////////////////////////////////////////////////////

fsm1.print_state_transition_matrix();

///////////////////////////////////////////////////////////////////////////////////////////////////
// define the transitions
// a transition needs to be defined for every state for every input
// for the sensors which dont make a difference it should stay in the same state
fsm1.trans_def(STATE_UI_calibrate, COND_UI_calibrate_to_walk, STATE_UI_walk);
fsm1.trans_def(STATE_UI_standby, COND_UI_standby_to_calibrate, STATE_UI_calibrate);

fsm1.trans_def(STATE_UI_walk, COND_UI_walk_to_standby, STATE_UI_standby);
fsm1.trans_def(STATE_UI_standby, COND_UI_standby_to_walk, STATE_UI_walk);

fsm1.trans_def(STATE_UI_walk, COND_UI_walk_to_nogo, STATE_UI_nogo);
fsm1.trans_def(STATE_UI_nogo, COND_UI_nogo_to_walk, STATE_UI_walk);
fsm1.trans_def(STATE_UI_nogo, COND_UI_nogo_to_standby, STATE_UI_standby);

fsm1.trans_def(STATE_UI_calibrate, COND_UI_stop, STATE_UI_stop);
fsm1.trans_def(STATE_UI_standby, COND_UI_stop, STATE_UI_stop);
fsm1.trans_def(STATE_UI_walk, COND_UI_stop, STATE_UI_stop);
fsm1.trans_def(STATE_UI_nogo, COND_UI_stop, STATE_UI_stop);
///////////////////////////////////////////////////////////////////////////////////////////////////

fsm1.print_state_transition_matrix();

fsm1.exit_state_def(STATE_UI_stop);

fsm1.set_sensor_input_function(get_UI_conditions_input);

fsm1.set_state_communication_variable(&g_ui_fsm_state);

copy(h_fsm, &fsm1);

}
