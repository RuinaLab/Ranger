
//#include <iostream>
#include "fsm.h"
#include "ankle_fsm.h"
#include "ankle_fsm_sensors.h"
#include "global_params.h"
#include "global_sensors.h"
#include "global_communications.h"

//using namespace std;

int do_push_off()
{ //cout << "  doing push_off" << endl; 

  return 1;
}

int do_lifting()
{ //cout << "  doing lifting" << endl;
  return 1; 
}

int do_staying_up()
{ //cout << "  doing staying up" << endl; 
  return 1;
}

int do_ready_to_hit()
{ //cout << "  doing ready to hit" << endl; 
  return 1;
}

int do_on_ground()
{ //cout << "  doing on ground" << endl; 
  return 1;
}

int do_ready_to_exit()
{ //cout << "  doing ready to exit" << endl; 
  return 1;
}



void def_ankle_fsm(fsm* a_fsm)
{

//cout << "defining ankle fsm.. " << endl;

// numStates is the number of states defined in structure.h
int numStates = 6;
// numInpts is the number of inputs defined in structure.h
int numInputs = 8;

// construct the fsm
fsm fsm1(numStates, on_ground, numInputs);

char* name = (char*) "ankle_fsm";
fsm1.set_name(name);

// associate the states with actions
fsm1.state_def(push_off, do_push_off);
fsm1.state_def(lifting, do_lifting);
fsm1.state_def(up, do_staying_up);
fsm1.state_def(ready_to_hit, do_ready_to_hit);
fsm1.state_def(on_ground, do_on_ground);
fsm1.state_def(ready_to_exit, do_ready_to_exit, 1);

// define the transitions
// a transition needs to be defined for every state for every input
// for the sensors which dont make a difference it should stay in the same state
fsm1.trans_def(push_off, _ground, lifting);
fsm1.trans_def(lifting, above_threshold, up);
fsm1.trans_def(up, hip_threshold, ready_to_hit);
fsm1.trans_def(ready_to_hit, ground, on_ground);
fsm1.trans_def(on_ground, _stop_command, push_off);

fsm1.trans_def(push_off, stop_command, lifting);
fsm1.trans_def(lifting, stop_command, up);
fsm1.trans_def(up, stop_command, ready_to_hit);
fsm1.trans_def(ready_to_hit, stop_command, on_ground);
fsm1.trans_def(on_ground, stop_command, ready_to_exit);

//fsm1.exit_state_def(ready_to_exit);

fsm1.set_sensor_input_function(get_ankle_sensor_input);

fsm1.set_state_communication_variable(&g_ankle_fsm_state);

copy(a_fsm, &fsm1);

}
