#include <mb_includes.h>
#include "global_communications.h"
#include "fsm.h"

#include "rock_foot_inner_fsm_conditions.h"
#include "rock_foot_inner_fsm_actions.h"

void def_rock_foot_inner_fsm(fsm* rfi_fsm)
{
// numStates is the number of states in the feet innner state machine
int numStates = 3;
// numInpts is the number of conditions
int numInputs = 4;

// starting state
fsm fsm1(numStates, STATE_rock_FI_push, numInputs);

// Give it a name
char* name = (char*) "rock_foot_inner_fsm";
fsm1.set_name(name);

// associate the states with actions
fsm1.state_def(STATE_rock_FI_level, ACT_rock_FI_level_entry, ACT_rock_FI_level, ACT_rock_FI_level);
fsm1.state_def(STATE_rock_FI_push, ACT_rock_FI_push_entry, ACT_rock_FI_push, ACT_rock_FI_push);
fsm1.state_def(STATE_rock_FI_clear, ACT_rock_FI_clear_entry, ACT_rock_FI_clear, ACT_rock_FI_clear);


fsm1.print_state_transition_matrix();

// define the transitions
// a transition needs to be defined for every state for every input
// for the sensors which dont make a difference it should stay in the same state

fsm1.trans_def(STATE_rock_FI_level, COND_rock_FI_level_to_push, STATE_rock_FI_push);
fsm1.trans_def(STATE_rock_FI_push, COND_rock_FI_push_to_level, STATE_rock_FI_level);
fsm1.trans_def(STATE_rock_FI_push, COND_rock_FI_push_to_clear, STATE_rock_FI_clear);
fsm1.trans_def(STATE_rock_FI_clear, COND_rock_FI_clear_to_level, STATE_rock_FI_level);

fsm1.print_state_transition_matrix();

// define exit state of fsm
//fsm fsm1(numStates, STATE_FI_flipdown, numInputs);

fsm1.set_sensor_input_function(get_rock_foot_inner_input);

fsm1.set_state_communication_variable(&g_rock_foot_inner_fsm_state);

copy(rfi_fsm, &fsm1);

}

