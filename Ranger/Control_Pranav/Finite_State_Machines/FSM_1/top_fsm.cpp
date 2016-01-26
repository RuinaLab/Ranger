
//#include <iostream>
#include "top_fsm.h"
#include "top_fsm_sensors.h"
#include "global_params.h"
#include "global_sensors.h"
#include "global_communications.h"

//using namespace std;

extern fsm* hip_fsm;
extern fsm* ankle_fsm;

int do_walking()
{ 
  //cout << "  doing walking" << endl;	

  if (g_top_fsm_stop_command == 1){

     //cout << "  received stop command" << endl;
         
         // assert stop commands to the lower level FSMs
         g_hip_fsm_stop_command = 1;
         g_ankle_fsm_stop_command = 1;
   
     //cout << "  stopping lower level fsms" << endl; 

        // run the lower level FSMs until they are ready to exit 
         while(!hip_fsm->ready_to_exit() || !ankle_fsm->ready_to_exit()){
               hip_fsm->run();
              ankle_fsm->run();
          }

         // once they are ready to exit they can be stopped
           hip_fsm->stop();
           ankle_fsm->stop();

         // clear the stop command sensors again so they can 
         // be used the next time
         g_hip_fsm_stop_command = 0;
         g_ankle_fsm_stop_command = 0;

         g_top_fsm_stop_command = 0;
         	
       }

   else {	

      //cout << "  running lower level fsms" << endl;
      // run the lower level FSMs
        hip_fsm->run();
        ankle_fsm->run();

   }
   
   return 1;
}


int do_stopping()
{ 

//cout << "  doing stopping" << endl; 
//exit(1);

return 1;

}

int do_top_ready_to_exit()
{

//cout << "  doing ready to exit" << endl;

  return 1;
}

void def_top_fsm(fsm* t_fsm){

//cout << endl;
//cout << "defining top_fsm..  " << endl;

// numStates is the number of states defined in structure.h
int numStates = 3;
// numInpts is the number of inputs defined in structure.h
int numInputs = 2;

// construct the fsm
fsm fsm1(numStates, 0, numInputs);

char* name = (char*) "top_fsm";

fsm1.set_name(name);

// associate the states with actions
fsm1.state_def(walking, do_walking, do_walking, do_walking);
fsm1.state_def(stopping, do_stopping, do_stopping, do_stopping);
fsm1.state_def(ready_to_exit, do_top_ready_to_exit, do_top_ready_to_exit, do_top_ready_to_exit);

// define the transitions
// a transition needs to be defined for every state for every input
// for the sensors which dont make a difference it should stay in the same state
fsm1.trans_def(walking, walk_command, walking);
fsm1.trans_def(stopping, walk_command, walking);
fsm1.trans_def(walking, stop_command, stopping);
fsm1.trans_def(stopping, stop_command, stopping);

fsm1.exit_state_def(ready_to_exit);

fsm1.set_sensor_input_function(get_top_sensor_input);

fsm1.set_state_communication_variable(&g_top_fsm_state);

copy(t_fsm, &fsm1);

}
