#include <mb_includes.h>
#include "fsm.h"
#include "global_communications.h"


#include "rock_hip_fsm_conditions.h"
#include "rock_hip_fsm_actions.h"


void def_rock_hip_fsm(fsm* rh_fsm)
{

// numStates is the number of states in the state machine
int numStates = 2;
// numInpts is the number of conditions defined in hip_fsm_conditions.h
int numInputs = 2;

// construct the fsm

// define start state
fsm fsm1(numStates, STATE_HI_swing, numInputs);

char* name = (char*) "rock_hip_fsm";
fsm1.set_name(name);

//debug 
fsm1.print_state_transition_matrix();

// associate the states with actions
fsm1.state_def(STATE_HI_swing, ACT_HI_swing_entry,ACT_HI_swing,ACT_HI_swing);
fsm1.state_def(STATE_HO_swing, ACT_HO_swing_entry,ACT_HO_swing,ACT_HO_swing);

fsm1.print_state_transition_matrix();

// define the transitions
// a transition needs to be defined for every state for every input
// for the sensors which dont make a difference it should stay in the same state
//These are the arrows in the state machine diagram
fsm1.trans_def(STATE_HI_swing, COND_HI_swing_to_HO_swing, STATE_HO_swing);
fsm1.trans_def(STATE_HO_swing, COND_HO_swing_to_HI_swing, STATE_HI_swing);


fsm1.print_state_transition_matrix();

//define exit state of fsm
//fsm1.exit_state_def(STATE_H_stop);

fsm1.set_sensor_input_function(get_rock_hip_sensor_input);

fsm1.set_state_communication_variable(&g_rock_hip_fsm_state);

//This is set in ui_fsm currently
//set_io_float(ID_MB_HIP_FSM_STATE, (float)g_hip_fsm_state);

copy(rh_fsm, &fsm1);

}

