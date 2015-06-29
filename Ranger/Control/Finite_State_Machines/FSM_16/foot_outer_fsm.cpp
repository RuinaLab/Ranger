#include <mb_includes.h>
#include "fsm.h"
#include "global_communications.h"

#include "foot_outer_fsm_conditions.h"
#include "foot_outer_fsm_actions.h"


void def_foot_outer_fsm(fsm* fo_fsm)
{
// numStates is the number of states in the state machine
int numStates = 7;
// numInpts is the number of conditions in the state machine
int numInputs = 10;

// start state 
fsm fsm1(numStates, STATE_FO_startstance, numInputs);

char* name = (char*) "foot_outer_fsm";
fsm1.set_name(name);

// associate the states with actions
fsm1.state_def(STATE_FO_stance, ACT_FO_stance_entry, ACT_FO_stance, ACT_FO_stance);
fsm1.state_def(STATE_FO_prepush, ACT_FO_prepush_entry, ACT_FO_prepush, ACT_FO_prepush_exit);
fsm1.state_def(STATE_FO_afterpush, ACT_FO_afterpush_entry, ACT_FO_afterpush, ACT_FO_afterpush_exit);
fsm1.state_def(STATE_FO_flipup, ACT_FO_flipup_entry, ACT_FO_flipup, ACT_FO_flipup);
fsm1.state_def(STATE_FO_flipdown, ACT_FO_flipdown_entry, ACT_FO_flipdown, ACT_FO_flipdown);
fsm1.state_def(STATE_FO_stop, ACT_FO_stop, ACT_FO_stop, ACT_FO_stop);
fsm1.state_def(STATE_FO_startstance, ACT_FO_startstance_entry, ACT_FO_startstance, ACT_FO_startstance);


fsm1.print_state_transition_matrix();

// define the transitions
// a transition needs to be defined for every state for every input
// for the sensors which dont make a difference it should stay in the same state
//THese are the arrows in the state machine diagram
fsm1.trans_def(STATE_FO_stance, COND_FO_stance_to_prepush, STATE_FO_prepush);
fsm1.trans_def(STATE_FO_prepush, COND_FO_prepush_to_afterpush, STATE_FO_afterpush);
fsm1.trans_def(STATE_FO_stance, COND_FO_stance_to_afterpush, STATE_FO_afterpush);   //skipped, Emergency transition
fsm1.trans_def(STATE_FO_afterpush, COND_FO_afterpush_to_flipup, STATE_FO_flipup);
fsm1.trans_def(STATE_FO_flipup, COND_FO_flipup_to_flipdown, STATE_FO_flipdown);
fsm1.trans_def(STATE_FO_flipdown, COND_FO_flipdown_to_stance, STATE_FO_stance);
fsm1.trans_def(STATE_FO_startstance, COND_FO_startstance_to_prepush, STATE_FO_prepush);   // Transition from start stance
                                                                             //CHANGED: afterpush to prepush 2/21/2011
fsm1.trans_def(STATE_FO_startstance, COND_FO_startstance_to_flipup, STATE_FO_flipup);   // Transition from start stance
                                                                             //CHANGED: afterpush to prepush 3/11/2011


fsm1.trans_def(STATE_FO_stance, COND_FO_startstance, STATE_FO_startstance);
fsm1.trans_def(STATE_FO_prepush, COND_FO_startstance, STATE_FO_startstance);
fsm1.trans_def(STATE_FO_afterpush, COND_FO_startstance, STATE_FO_startstance);
fsm1.trans_def(STATE_FO_flipup, COND_FO_startstance, STATE_FO_startstance);
fsm1.trans_def(STATE_FO_flipdown, COND_FO_startstance, STATE_FO_startstance);

fsm1.print_state_transition_matrix();

// define exit state of fsm
fsm1.exit_state_def(STATE_FO_stop);

fsm1.set_sensor_input_function(get_foot_outer_input);

fsm1.set_state_communication_variable(&g_foot_outer_fsm_state);

copy(fo_fsm, &fsm1);

}

