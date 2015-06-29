#include <mb_includes.h>
#include "global_communications.h"
#include "fsm.h"

#include "foot_outer_fsm_conditions.h"
#include "foot_outer_fsm_actions.h"

void def_foot_outer_fsm(fsm* fi_fsm)
{
// numStates is the number of states in the feet innner state machine
int numStates = 2;
// numInpts is the number of conditions
int numInputs = 2;

// starting state
fsm fsm1(numStates, STATE_FO_flipdown, numInputs);

// Give it a name
char* name = (char*) "foot_outer_fsm";
fsm1.set_name(name);

// associate the states with actions
fsm1.state_def(STATE_FO_flipup, ACT_FO_flipup);
fsm1.state_def(STATE_FO_flipdown, ACT_FO_flipdown);


fsm1.print_state_transition_matrix();

// define the transitions
// a transition needs to be defined for every state for every input
// for the sensors which dont make a difference it should stay in the same state
fsm1.trans_def(STATE_FO_flipup, COND_FO_flipup_to_flipdown, STATE_FO_flipdown);
fsm1.trans_def(STATE_FO_flipdown, COND_FO_flipdown_to_push, STATE_FO_flipup);

fsm1.print_state_transition_matrix();

// define exit state of fsm
//fsm fsm1(numStates, STATE_FI_flipdown, numInputs);

fsm1.set_sensor_input_function(get_foot_outer_input);

fsm1.set_state_communication_variable(&g_foot_outer_fsm_state);

copy(fi_fsm, &fsm1);

}
