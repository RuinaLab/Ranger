#include <mb_includes.h>
#include "fsm.h"
#include "global_communications.h"



#include "hip_fsm_actions.h"
#include "hip_fsm_conditions.h"

void def_hip_fsm(fsm* h_fsm)
{

// numStates is the number of states in the state machine
int numStates = 12;
// numInpts is the number of conditions defined in hip_fsm_conditions.h
int numInputs = 17;

// construct the fsm

// define start state
fsm fsm1(numStates, STATE_HI_starthold, numInputs);

char* name = (char*) "hip_fsm";
fsm1.set_name(name);

//debug 
fsm1.print_state_transition_matrix();

// associate the states with actions
fsm1.state_def(STATE_HI_starthold, ACT_HI_starthold_entry, ACT_HI_starthold, ACT_HI_starthold); //Beginning start hold state
fsm1.state_def(STATE_HI_preswing, ACT_HI_preswing, ACT_HI_preswing, ACT_HI_preswing);
fsm1.state_def(STATE_HI_premid, ACT_HI_premid_entry, ACT_HI_premid, ACT_HI_premid_exit);
fsm1.state_def(STATE_HI_aftermid, ACT_HI_aftermid_entry, ACT_HI_aftermid, ACT_HI_aftermid);
fsm1.state_def(STATE_HO_preswing, ACT_HO_preswing, ACT_HO_preswing, ACT_HO_preswing);
fsm1.state_def(STATE_HO_premid, ACT_HO_premid_entry, ACT_HO_premid, ACT_HO_premid_exit);
fsm1.state_def(STATE_HO_aftermid, ACT_HO_aftermid_entry, ACT_HO_aftermid, ACT_HO_aftermid);
fsm1.state_def(STATE_H_stop, ACT_H_stop, ACT_H_stop, ACT_H_stop); //Stop state
fsm1.state_def(STATE_HI_ehold, ACT_HI_ehold_entry, ACT_HI_ehold, ACT_HI_ehold);  //Emergency inner hold state
fsm1.state_def(STATE_HO_ehold, ACT_HO_ehold_entry, ACT_HO_ehold, ACT_HO_ehold);  //Emergency outer hold state
fsm1.state_def(STATE_HI_stophold, ACT_HI_stophold_entry, ACT_HI_stophold, ACT_HI_stophold);  //inner hold state for stopping
fsm1.state_def(STATE_HO_stophold, ACT_HO_stophold_entry, ACT_HO_stophold, ACT_HO_stophold);  //outer hold state for stopping

fsm1.print_state_transition_matrix();

// define the transitions
// a transition needs to be defined for every state for every input
// for the sensors which dont make a difference it should stay in the same state
//These are the arrows in the state machine diagram
fsm1.trans_def(STATE_HI_starthold, COND_HI_starthold_to_HO_preswing, STATE_HO_preswing);
fsm1.trans_def(STATE_HO_preswing, COND_HO_preswing_to_HO_premid, STATE_HO_premid);
fsm1.trans_def(STATE_HO_premid, COND_HO_premid_to_HO_aftermid, STATE_HO_aftermid);

fsm1.trans_def(STATE_HO_aftermid, COND_HO_aftermid_to_HI_preswing, STATE_HI_preswing); 
fsm1.trans_def(STATE_HO_ehold, COND_HO_ehold_to_HI_preswing, STATE_HI_preswing);
fsm1.trans_def(STATE_HI_preswing, COND_HI_preswing_to_HI_premid, STATE_HI_premid);
fsm1.trans_def(STATE_HI_premid, COND_HI_premid_to_HI_aftermid, STATE_HI_aftermid);

fsm1.trans_def(STATE_HI_aftermid, COND_HI_aftermid_to_HO_preswing, STATE_HO_preswing);
fsm1.trans_def(STATE_HI_ehold, COND_HI_ehold_to_HO_preswing, STATE_HO_preswing);

fsm1.trans_def(STATE_HI_premid, COND_HI_premid_to_HI_ehold, STATE_HI_ehold);
fsm1.trans_def(STATE_HO_premid, COND_HO_premid_to_HO_ehold, STATE_HO_ehold);
fsm1.trans_def(STATE_HI_aftermid, COND_HI_aftermid_to_HI_ehold, STATE_HI_ehold);
fsm1.trans_def(STATE_HO_aftermid, COND_HO_aftermid_to_HO_ehold, STATE_HO_ehold);

fsm1.trans_def(STATE_HI_preswing, COND_HI_preswing_to_HI_stophold, STATE_HI_stophold); //stopping
fsm1.trans_def(STATE_HO_preswing, COND_HO_preswing_to_HO_stophold, STATE_HO_stophold);

fsm1.trans_def(STATE_HI_preswing, COND_HI_starthold, STATE_HI_starthold);
fsm1.trans_def(STATE_HI_premid, COND_HI_starthold, STATE_HI_starthold);
fsm1.trans_def(STATE_HI_aftermid, COND_HI_starthold, STATE_HI_starthold);
fsm1.trans_def(STATE_HI_ehold, COND_HI_starthold, STATE_HI_starthold);
fsm1.trans_def(STATE_HI_stophold, COND_HI_starthold, STATE_HI_starthold);

fsm1.trans_def(STATE_HO_preswing, COND_HI_starthold, STATE_HI_starthold);
fsm1.trans_def(STATE_HO_premid, COND_HI_starthold, STATE_HI_starthold);
fsm1.trans_def(STATE_HO_aftermid, COND_HI_starthold, STATE_HI_starthold);
fsm1.trans_def(STATE_HO_ehold, COND_HI_starthold, STATE_HI_starthold);
fsm1.trans_def(STATE_HO_stophold, COND_HI_starthold, STATE_HI_starthold);

fsm1.print_state_transition_matrix();

//define exit state of fsm
fsm1.exit_state_def(STATE_H_stop);

fsm1.set_sensor_input_function(get_hip_sensor_input);

fsm1.set_state_communication_variable(&g_hip_fsm_state);

//This is set in ui_fsm currently
//set_io_float(ID_MB_HIP_FSM_STATE, (float)g_hip_fsm_state);

copy(h_fsm, &fsm1);

}

