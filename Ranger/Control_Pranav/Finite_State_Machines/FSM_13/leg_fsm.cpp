#include <mb_includes.h>
#include "fsm.h"
#include "global_communications.h"


#include "leg_fsm_conditions.h"
#include "leg_fsm_actions.h"


void def_leg_fsm(fsm* l_fsm)
{

// numStates is the number of states in the state machine
int numStates = 7;
// numInpts is the number of conditions defined in hip_fsm_conditions.h
int numInputs = 8;

// construct the fsm

// define start state
fsm fsm1(numStates, STATE_LO_aftermid, numInputs);

char* name = (char*) "leg_fsm";
fsm1.set_name(name);

//debug 
fsm1.print_state_transition_matrix();

// associate the states with actions
fsm1.state_def(STATE_LI_preswing, ACT_LI_preswing_entry, ACT_LI_preswing, ACT_LI_preswing);
fsm1.state_def(STATE_LI_premid, ACT_LI_premid_entry, ACT_LI_premid, ACT_LI_premid);
fsm1.state_def(STATE_LI_aftermid, ACT_LI_aftermid_entry, ACT_LI_aftermid, ACT_LI_aftermid);
fsm1.state_def(STATE_LO_preswing, ACT_LO_preswing_entry, ACT_LO_preswing, ACT_LO_preswing);
fsm1.state_def(STATE_LO_premid, ACT_LO_premid_entry, ACT_LO_premid, ACT_LO_premid);
fsm1.state_def(STATE_LO_aftermid, ACT_LO_aftermid_entry, ACT_LO_aftermid, ACT_LO_aftermid);
fsm1.state_def(STATE_L_stop, ACT_L_stop, ACT_L_stop, ACT_L_stop); //Stop state

fsm1.print_state_transition_matrix();

// define the transitions
// a transition needs to be defined for every state for every input
// for the sensors which dont make a difference it should stay in the same state
//These are the arrows in the state machine diagram

fsm1.trans_def(STATE_LO_preswing, COND_LO_preswing_to_LO_premid, STATE_LO_premid);
fsm1.trans_def(STATE_LO_premid, COND_LO_premid_to_LO_aftermid, STATE_LO_aftermid);
fsm1.trans_def(STATE_LO_aftermid, COND_LO_aftermid_to_LI_preswing, STATE_LI_preswing); 

fsm1.trans_def(STATE_LI_preswing, COND_LI_preswing_to_LI_premid, STATE_LI_premid);
fsm1.trans_def(STATE_LI_premid, COND_LI_premid_to_LI_aftermid, STATE_LI_aftermid);
fsm1.trans_def(STATE_LI_aftermid, COND_LI_aftermid_to_LO_preswing, STATE_LO_preswing);


fsm1.trans_def(STATE_LI_preswing, COND_LO_startaftermid, STATE_LO_aftermid);
fsm1.trans_def(STATE_LI_premid, COND_LO_startaftermid, STATE_LO_aftermid);
fsm1.trans_def(STATE_LI_aftermid, COND_LO_startaftermid, STATE_LO_aftermid);
fsm1.trans_def(STATE_LO_preswing, COND_LO_startaftermid, STATE_LO_aftermid);
fsm1.trans_def(STATE_LO_premid, COND_LO_startaftermid, STATE_LO_aftermid);

fsm1.print_state_transition_matrix();

//define exit state of fsm
fsm1.exit_state_def(STATE_L_stop);

fsm1.set_sensor_input_function(get_leg_sensor_input);

fsm1.set_state_communication_variable(&g_leg_fsm_state);

//This is set in ui_fsm currently
//set_io_float(ID_MB_HIP_FSM_STATE, (float)g_hip_fsm_state);

copy(l_fsm, &fsm1);

}

