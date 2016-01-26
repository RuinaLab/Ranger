
//#include <iostream>
#include <mb_includes.h>
#include "fsm.h"
#include "steering_fsm.h"
#include "steering_fsm_conditions.h"
#include "global_params.h"
#include "global_sensors.h"
#include "global_communications.h"

static float desired_angle;
//static float scaled_rc_value;
//using namespace std;

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Define action functions
//TO DO:
//ID_D_S_NULL_S_DANG to point to desired steer angle commanded by rc

int ACT_S_innerlegfront_entry()
{
  desired_angle = -desired_angle; //NEW
  
  //set_io_float(ID_MCSI_STIFFNESS, get_io_float(ID_C_S_NULL_S_ANG)); 
  //set_io_float(ID_MCSI_DAMPNESS, get_io_float(ID_C_S_NULL_S_RATE)); 
  return 1;
}

int ACT_S_innerlegfront()
{
  // Test code to find what ID_UI_RC0 is doing 6/2/2010
  //float rc_command = get_io_float(ID_UI_RC_0)/25000 - 3.4;
  //set_io_float(ID_E_TEST1,rc_command);
  // End Test
  
  //set_io_float(ID_MCSI_STIFFNESS, get_io_float(ID_C_S_NULL_S_ANG)); 
  //set_io_float(ID_MCSI_DAMPNESS, get_io_float(ID_C_S_NULL_S_RATE)); 
 
  //set_io_float(ID_MCSI_COMMAND_CURRENT, desired_angle * get_io_float(ID_MCSI_STIFFNESS));

  return 1;
}

int ACT_S_innerlegstance_entry()
{
  //desired_angle = get_io_float(ID_P_S_TEMP_S_TANG); //OLD //Set desired angle 
  
  //set_io_float(ID_MCSI_STIFFNESS, get_io_float(ID_C_S_NULL_S_ANG)); 
  //set_io_float(ID_MCSI_DAMPNESS, get_io_float(ID_C_S_NULL_S_RATE)); 

  

  return 1;
}

int ACT_S_innerlegstance()
{ 
  
  //set_io_float(ID_MCSI_STIFFNESS, get_io_float(ID_C_S_NULL_S_ANG)); 
  //set_io_float(ID_MCSI_DAMPNESS, get_io_float(ID_C_S_NULL_S_RATE)); 
  
  
  set_io_float(ID_MCSI_COMMAND_CURRENT, desired_angle * get_io_float(ID_MCSI_STIFFNESS));
  return 1; 
}

int ACT_S_innerlegback_entry()
{ 

  
  float rc_command = get_io_float(ID_UI_RC_0)/25000 - 3.4; //A number between -1 and 1
  desired_angle = get_io_float(ID_P_S_ILST_S_MAXANG)*rc_command; 
  
  if (rc_command<-1.0)
      { rc_command = -1.0;}
  else if (rc_command>1.0)
      { rc_command=1.0; }
  
  //set_io_float(ID_E_TEST1,rc_command);
  //set_io_float(ID_D_S_NULL_S_DANG,desired_angle); Needs to be a data channel in can_id.h 2010/5/22
  
  if (rc_command<0.15 && rc_command >-0.15) //don't steer
     { //Put MCSI sleep here
      set_io_float(ID_MCSI_STIFFNESS, get_io_float(ID_C_S_NULL_S_ANG)); 
      set_io_float(ID_MCSI_DAMPNESS, get_io_float(ID_C_S_NULL_S_RATE));
      } 
  else //steer
    {
      set_io_float(ID_MCSI_STIFFNESS, get_io_float(ID_C_S_NULL_S_ANG)); 
      set_io_float(ID_MCSI_DAMPNESS, get_io_float(ID_C_S_NULL_S_RATE)); 
    }
  return 1; 
}

int ACT_S_innerlegback()
{ 
   //set_io_float(ID_MCSI_STIFFNESS, get_io_float(ID_C_S_NULL_S_ANG)); 
   //set_io_float(ID_MCSI_DAMPNESS, get_io_float(ID_C_S_NULL_S_RATE)); 
 
  //set_io_float(ID_MCSI_COMMAND_CURRENT, desired_angle * get_io_float(ID_MCSI_STIFFNESS));
  return 1; 
}

int ACT_S_innerlegswing_entry()
{ 
  //set_io_float(ID_MCSI_STIFFNESS, get_io_float(ID_C_S_NULL_S_ANG)); 
  //set_io_float(ID_MCSI_DAMPNESS, get_io_float(ID_C_S_NULL_S_RATE)); 

  //desired_angle = -get_io_float(ID_P_S_TEMP_S_TANG); //OLD
  //ID_D_S_NULL_S_DANG = set_io_float(ID_D_S_NULL_S_DANG,desired_angle);

  return 1; 
}

int ACT_S_innerlegswing()
{ 

  //set_io_float(ID_MCSI_STIFFNESS, get_io_float(ID_C_S_NULL_S_ANG)); 
  //set_io_float(ID_MCSI_DAMPNESS, get_io_float(ID_C_S_NULL_S_RATE)); 
 
  //desired_angle = -get_io_float(ID_P_S_TEMP_S_TANG); //OLD
  set_io_float(ID_MCSI_COMMAND_CURRENT, desired_angle * get_io_float(ID_MCSI_STIFFNESS));
  //set_io_float(ID_MCSI_COMMAND_CURRENT, 0.1);

  return 1;
}


int ACT_S_stop()
{  
  set_io_float(ID_MCSI_COMMAND_CURRENT, 0.0);
  set_io_float(ID_MCSI_STIFFNESS, 0.0);
  set_io_float(ID_MCSI_DAMPNESS, 0.0);
  return 1;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////


void def_steering_fsm(fsm* fi_fsm)
{

//cout << "defining ankle fsm.. " << endl;

// numStates is the number of states defined in structure.h
int numStates = 5;
// numInpts is the number of inputs defined in structure.h
int numInputs = 8;

// construct the fsm
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

