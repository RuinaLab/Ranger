#include <mb_includes.h>
#include "fsm.h"
#include "global_communications.h"

#include "steering_fsm_conditions.h"
#include "steering_fsm_actions.h"



void def_steering_fsm(fsm* fi_fsm)
{

// numStates is the number of states in state machine
int numStates = 6;
// numInpts is the number of condition in state machine
int numInputs = 7;

// start the fsm in this state
//fsm fsm1(numStates, STATE_S_innerlegfront, numInputs);
fsm fsm1(numStates, STATE_S_innerswingaftermid, numInputs);

char* name = (char*) "steering_fsm";
fsm1.set_name(name);

// associate the states with actions
fsm1.state_def(STATE_S_innerlegfront, ACT_S_innerlegfront_entry, ACT_S_innerlegfront, ACT_S_innerlegfront);
fsm1.state_def(STATE_S_innerlegstance, ACT_S_innerlegstance_entry, ACT_S_innerlegstance, ACT_S_innerlegstance);
fsm1.state_def(STATE_S_innerlegback, ACT_S_innerlegback_entry, ACT_S_innerlegback, ACT_S_innerlegback);
fsm1.state_def(STATE_S_innerswingpremid, ACT_S_innerswingpremid_entry, ACT_S_innerswingpremid, ACT_S_innerswingpremid);
fsm1.state_def(STATE_S_innerswingaftermid, ACT_S_innerswingaftermid_entry, ACT_S_innerswingaftermid, ACT_S_innerswingaftermid);
fsm1.state_def(STATE_S_stop, ACT_S_stop, ACT_S_stop, ACT_S_stop);


fsm1.print_state_transition_matrix();

// define the transitions
// a transition needs to be defined for every state for every input
// for the sensors which dont make a difference it should stay in the same state
fsm1.trans_def(STATE_S_innerlegfront, COND_S_inlegfront_to_inlegstance, STATE_S_innerlegstance);
fsm1.trans_def(STATE_S_innerlegstance, COND_S_inlegstance_to_inlegback, STATE_S_innerlegback);  
fsm1.trans_def(STATE_S_innerlegback, COND_S_inlegback_to_inswingpremid, STATE_S_innerswingpremid);
fsm1.trans_def(STATE_S_innerswingpremid, COND_S_inswingpremid_to_inswingaftermid, STATE_S_innerswingaftermid);
fsm1.trans_def(STATE_S_innerswingaftermid, COND_S_inswingaftermid_to_inlegfront, STATE_S_innerlegfront);


// go to the start state, when the fsm is to be reset
fsm1.trans_def(STATE_S_innerlegfront, COND_S_reset, STATE_S_innerswingaftermid);
fsm1.trans_def(STATE_S_innerlegstance, COND_S_reset, STATE_S_innerswingaftermid);
fsm1.trans_def(STATE_S_innerlegback, COND_S_reset, STATE_S_innerswingaftermid);
fsm1.trans_def(STATE_S_innerswingpremid, COND_S_reset, STATE_S_innerswingaftermid);


//Stop from any state based on COND
fsm1.trans_def(STATE_S_innerlegfront, COND_S_stop, STATE_S_stop);
fsm1.trans_def(STATE_S_innerlegstance, COND_S_stop, STATE_S_stop);
fsm1.trans_def(STATE_S_innerlegback, COND_S_stop, STATE_S_stop);
fsm1.trans_def(STATE_S_innerswingpremid, COND_S_stop, STATE_S_stop);
fsm1.trans_def(STATE_S_innerswingaftermid, COND_S_stop, STATE_S_stop);

fsm1.print_state_transition_matrix();

// define exit state of fsm
fsm1.exit_state_def(STATE_S_stop);

fsm1.set_sensor_input_function(get_steering_input);

fsm1.set_state_communication_variable(&g_steering_fsm_state);

//set_io_float(ID_MB_FOOT_INNER_FSM_STATE, (float)g_foot_inner_fsm_state);

copy(fi_fsm, &fsm1);

}




/*
void def_steering_fsm(fsm* fi_fsm)
{

// numStates is the number of states in state machine
int numStates = 5;
// numInpts is the number of condition in state machine
int numInputs = 8;

// start the fsm in this state
fsm fsm1(numStates, STATE_S_innerlegfront, numInputs);

char* name = (char*) "steering_fsm";
fsm1.set_name(name);

// associate the states with actions
fsm1.state_def(STATE_S_innerlegfront, ACT_S_innerlegfront_entry, ACT_S_innerlegfront, ACT_S_innerlegfront);
fsm1.state_def(STATE_S_innerlegstance, ACT_S_innerlegstance_entry, ACT_S_innerlegstance, ACT_S_innerlegstance);
fsm1.state_def(STATE_S_innerlegback, ACT_S_innerlegback_entry, ACT_S_innerlegback, ACT_S_innerlegback);
fsm1.state_def(STATE_S_innerlegswing, ACT_S_innerlegswing_entry, ACT_S_innerlegswing, ACT_S_innerlegswing);
fsm1.state_def(STATE_S_stop, ACT_S_stop, ACT_S_stop, ACT_S_stop);


fsm1.print_state_transition_matrix();

// define the transitions
// a transition needs to be defined for every state for every input
// for the sensors which dont make a difference it should stay in the same state
fsm1.trans_def(STATE_S_innerlegfront, COND_S_inlegfront_to_inlegstance, STATE_S_innerlegstance);
fsm1.trans_def(STATE_S_innerlegstance, COND_S_inlegstance_to_inlegback, STATE_S_innerlegback);  
fsm1.trans_def(STATE_S_innerlegback, COND_S_inlegback_to_inlegswing, STATE_S_innerlegswing);
fsm1.trans_def(STATE_S_innerlegswing, COND_S_inlegswing_to_inlegfront, STATE_S_innerlegfront);

//Stop from any state based on COND
fsm1.trans_def(STATE_S_innerlegfront, COND_S_inlegfront_to_stop, STATE_S_stop);
fsm1.trans_def(STATE_S_innerlegstance, COND_S_inlegstance_to_stop, STATE_S_stop);
fsm1.trans_def(STATE_S_innerlegback, COND_S_inlegback_to_stop, STATE_S_stop);
fsm1.trans_def(STATE_S_innerlegswing, COND_S_inlegswing_to_stop, STATE_S_stop);

fsm1.print_state_transition_matrix();

// define exit state of fsm
fsm1.exit_state_def(STATE_S_stop);

fsm1.set_sensor_input_function(get_steering_input);

fsm1.set_state_communication_variable(&g_steering_fsm_state);

//set_io_float(ID_MB_FOOT_INNER_FSM_STATE, (float)g_foot_inner_fsm_state);

copy(fi_fsm, &fsm1);

}
*/
