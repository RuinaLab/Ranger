#include <mb_includes.h>
#include "fsm.h"
#include "global_communications.h"

#include "top_fsm_conditions.h"
#include "top_fsm_actions.h"

///////////////////////////////////////////////////////////////////////////////////////////////////

void def_top_fsm(fsm* h_fsm)
{


// numStates is the number of states in state machine
int numStates = 7;
// numInpts is the number of condition in state machine
int numInputs = 11;

// construct the fsm
fsm fsm1(numStates, STATE_top_walk_normal, numInputs);

char* name = (char*) "top_fsm";
fsm1.set_name(name);

//debug
fsm1.print_state_transition_matrix();

///////////////////////////////////////////////////////////////////////////////////////////////////
// associate the states with actions

fsm1.state_def(STATE_top_walk_normal, ACTION_top_walk_normal_entry, ACTION_top_walk_normal, ACTION_top_walk_normal);
fsm1.state_def(STATE_top_walk_slow, ACTION_top_walk_slow_entry, ACTION_top_walk_slow, ACTION_top_walk_slow);
fsm1.state_def(STATE_top_walk_fast, ACTION_top_walk_fast_entry, ACTION_top_walk_fast, ACTION_top_walk_fast);
fsm1.state_def(STATE_top_walk_stop, ACTION_top_walk_stop_entry, ACTION_top_walk_stop, ACTION_top_walk_stop);
fsm1.state_def(STATE_top_walk_start, ACTION_top_walk_start_entry, ACTION_top_walk_start, ACTION_top_walk_start);
fsm1.state_def(STATE_top_stop, ACTION_top_stop, ACTION_top_stop, ACTION_top_stop);
fsm1.state_def(STATE_top_temp, ACTION_top_temp, ACTION_top_temp, ACTION_top_temp);


///////////////////////////////////////////////////////////////////////////////////////////////////

fsm1.print_state_transition_matrix();

///////////////////////////////////////////////////////////////////////////////////////////////////
// define the transitions
// a transition needs to be defined for every state for every input
// for the sensors which dont make a difference it should stay in the same state
//fsm1.trans_def(STATE_top_walk_normal, COND_top_walk_normal_to_walk_stop, STATE_top_walk_stop);
fsm1.trans_def(STATE_top_walk_normal, COND_top_walk_normal_to_walk_fast, STATE_top_walk_fast);
fsm1.trans_def(STATE_top_walk_normal, COND_top_walk_normal_to_temp, STATE_top_temp);
//fsm1.trans_def(STATE_top_walk_normal, COND_top_walk_normal_to_walk_slow, STATE_top_walk_slow);

//fsm1.trans_def(STATE_top_walk_stop, COND_top_walk_stop_to_walk_start, STATE_top_walk_start); //on rc command

fsm1.trans_def(STATE_top_walk_stop, COND_top_walk_stop_to_walk_normal, STATE_top_walk_normal); //on pressing fsm_reset
//fsm1.trans_def(STATE_top_walk_start, COND_top_walk_start_to_walk_normal, STATE_top_walk_normal); //on pressing fsm_reset


fsm1.trans_def(STATE_top_walk_fast, COND_top_walk_normal, STATE_top_walk_normal);
fsm1.trans_def(STATE_top_walk_slow, COND_top_walk_normal, STATE_top_walk_normal);

fsm1.trans_def(STATE_top_temp, COND_top_temp_to_walk_slow, STATE_top_walk_slow);
fsm1.trans_def(STATE_top_temp, COND_top_temp_to_walk_stop, STATE_top_walk_stop);

///////////////////////////////////////////////////////////////////////////////////////////////////

fsm1.print_state_transition_matrix();

fsm1.exit_state_def(STATE_top_stop);

fsm1.set_sensor_input_function(get_top_conditions_input);

fsm1.set_state_communication_variable(&g_top_fsm_state);

copy(h_fsm, &fsm1);

}
