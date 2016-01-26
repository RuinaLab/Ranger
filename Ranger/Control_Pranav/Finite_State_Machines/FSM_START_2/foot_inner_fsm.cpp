#include <mb_includes.h>
#include "global_communications.h"
#include "fsm.h"

#include "foot_inner_fsm_conditions.h"
#include "foot_inner_fsm_actions.h"

void def_foot_inner_fsm(fsm* fi_fsm)
{
// numStates is the number of states in the feet innner state machine
int numStates = 7;
// numInpts is the number of conditions
int numInputs = 9;

// starting state
fsm fsm1(numStates, STATE_FI_startflipdown, numInputs);

// Give it a name
char* name = (char*) "foot_inner_fsm";
fsm1.set_name(name);

// associate the states with actions
fsm1.state_def(STATE_FI_stance, ACT_FI_stance_entry, ACT_FI_stance, ACT_FI_stance);
fsm1.state_def(STATE_FI_prepush, ACT_FI_prepush_entry, ACT_FI_prepush, ACT_FI_prepush_exit); 
fsm1.state_def(STATE_FI_afterpush, ACT_FI_afterpush_entry, ACT_FI_afterpush, ACT_FI_afterpush_exit);
fsm1.state_def(STATE_FI_flipup, ACT_FI_flipup_entry, ACT_FI_flipup, ACT_FI_flipup);
fsm1.state_def(STATE_FI_flipdown, ACT_FI_flipdown_entry, ACT_FI_flipdown, ACT_FI_flipdown);
fsm1.state_def(STATE_FI_stop, ACT_FI_stop, ACT_FI_stop, ACT_FI_stop);
fsm1.state_def(STATE_FI_startflipdown, ACT_FI_startflipdown_entry, ACT_FI_startflipdown, ACT_FI_startflipdown); //Start state

fsm1.print_state_transition_matrix();

// define the transitions
// a transition needs to be defined for every state for every input
// for the sensors which dont make a difference it should stay in the same state
fsm1.trans_def(STATE_FI_stance, COND_FI_stance_to_prepush, STATE_FI_prepush);
fsm1.trans_def(STATE_FI_prepush, COND_FI_prepush_to_afterpush, STATE_FI_afterpush);
fsm1.trans_def(STATE_FI_stance, COND_FI_stance_to_afterpush, STATE_FI_afterpush);   //skipped, Emergency transition
fsm1.trans_def(STATE_FI_afterpush, COND_FI_afterpush_to_flipup, STATE_FI_flipup);
fsm1.trans_def(STATE_FI_flipup, COND_FI_flipup_to_flipdown, STATE_FI_flipdown);
fsm1.trans_def(STATE_FI_flipdown, COND_FI_flipdown_to_stance, STATE_FI_stance);
fsm1.trans_def(STATE_FI_startflipdown, COND_FI_startflipdown_to_stance, STATE_FI_stance);   // Transition from start flipdown

fsm1.trans_def(STATE_FI_stance, COND_FI_startflipdown, STATE_FI_startflipdown);
fsm1.trans_def(STATE_FI_prepush, COND_FI_startflipdown, STATE_FI_startflipdown);
fsm1.trans_def(STATE_FI_afterpush, COND_FI_startflipdown, STATE_FI_startflipdown);
fsm1.trans_def(STATE_FI_flipup, COND_FI_startflipdown, STATE_FI_startflipdown);
fsm1.trans_def(STATE_FI_flipdown, COND_FI_startflipdown, STATE_FI_startflipdown);

fsm1.print_state_transition_matrix();

// define exit state of fsm
fsm1.exit_state_def(STATE_FI_stop);

fsm1.set_sensor_input_function(get_foot_inner_input);

fsm1.set_state_communication_variable(&g_foot_inner_fsm_state);

copy(fi_fsm, &fsm1);

}

