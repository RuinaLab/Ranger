#include <mb_includes.h>
#include "global_communications.h"
#include "fsm.h"

#include "rock_foot_outer_fsm_conditions.h"
#include "rock_foot_outer_fsm_actions.h"

void def_rock_foot_outer_fsm(fsm* rfo_fsm)
{
// numStates is the number of states in the feet innner state machine
int numStates = 2;
// numInpts is the number of conditions
int numInputs = 2;

// starting state
fsm fsm1(numStates, STATE_rock_FO_level, numInputs);

// Give it a name
char* name = (char*) "rock_foot_outer_fsm";
fsm1.set_name(name);

// associate the states with actions
fsm1.state_def(STATE_rock_FO_level, ACT_rock_FO_level_entry, ACT_rock_FO_level, ACT_rock_FO_level);
fsm1.state_def(STATE_rock_FO_push, ACT_rock_FO_push_entry,ACT_rock_FO_push, ACT_rock_FO_push);


fsm1.print_state_transition_matrix();

// define the transitions
// a transition needs to be defined for every state for every input
// for the sensors which dont make a difference it should stay in the same state
fsm1.trans_def(STATE_rock_FO_level, COND_rock_FO_level_to_push, STATE_rock_FO_push);
fsm1.trans_def(STATE_rock_FO_push, COND_rock_FO_push_to_level, STATE_rock_FO_level);

fsm1.print_state_transition_matrix();

// define exit state of fsm
//fsm fsm1(numStates, STATE_FI_flipdown, numInputs);

fsm1.set_sensor_input_function(get_rock_foot_outer_input);

fsm1.set_state_communication_variable(&g_rock_foot_outer_fsm_state);

copy(rfo_fsm, &fsm1);

}
