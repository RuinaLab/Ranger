#include <mb_includes.h>
//#include <iostream>
#include "fsm.h"
#include "foot_outer_fsm.h"
#include "foot_outer_fsm_conditions.h"
#include "global_params.h"
#include "global_sensors.h"
#include "global_communications.h"

//using namespace std;
static float stance_to_prepush_angle;
//////////////////////////////////////////////////////////////////////////////////////////////////////
// Define action functions

int ACT_FO_stance_entry()
{
//  set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_C_F_ST_F_ANG)); //TEMP comment, 4/6/2010
//  set_io_float(ID_MCFO_DAMPNESS, get_io_float(ID_C_F_ST_F_RATE)); //TEMP comment, 4/6/2010
  return 1;
}

int ACT_FO_stance()
{
  float target_angle;
  
  set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_C_F_ST_F_ANG)); //TEMP , 4/6/2010
  set_io_float(ID_MCFO_DAMPNESS, get_io_float(ID_C_F_ST_F_RATE)); //TEMP , 4/6/2010
  
  
  target_angle =  get_io_float(ID_P_F_ST_F_TANG) - get_io_float(ID_MCH_ANGLE) * 0.5;
  //  target_angle =  get_io_float(ID_P_F_ST_F_TANG); //Temp 5/3/2010 for feet asymmetry testing

//  if ((get_io_float(ID_MCFO_MID_ANKLE_ANGLE) - target_angle) > 0.1) //Ankle cable slackness safety
//  {
//     target_angle = get_io_float(ID_MCFO_MID_ANKLE_ANGLE) - 0.1;
//  }
  
  set_io_float(ID_MCFO_COMMAND_CURRENT, target_angle * get_io_float(ID_MCFO_STIFFNESS)); 
  return 1;
}

int ACT_FO_prepush_entry()

{
 stance_to_prepush_angle = get_io_float(ID_MCFO_RIGHT_ANKLE_ANGLE); //save the stance angle during exit
//  set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_C_F_PP_F_ANG)); //TEMP comment, 4/6/2010
//  set_io_float(ID_MCFO_DAMPNESS, get_io_float(ID_C_F_PP_F_RATE)); //TEMP comment, 4/6/2010
  return 1;
}

int ACT_FO_prepush()
{ 
  float target_angle;
  
  set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_C_F_PP_F_ANG)); //TEMP , 4/6/2010
  set_io_float(ID_MCFO_DAMPNESS, get_io_float(ID_C_F_PP_F_RATE)); //TEMP , 4/6/2010
  
  target_angle = stance_to_prepush_angle + get_io_float(ID_P_F_PP_F_TANG); 
//  target_angle =  get_io_float(ID_P_F_PP_F_TANG) + get_io_float(ID_MCH_ANGLE) * 0.5;
//  target_angle =  get_io_float(ID_P_F_PP_F_TANG); //Based on relative angle, 4/27/2010
  
//  if ((get_io_float(ID_MCFO_MID_ANKLE_ANGLE) - target_angle) > 0.1)   //Ankle cable slackness safety
//  {
//     target_angle = get_io_float(ID_MCFO_MID_ANKLE_ANGLE) - 0.1;
//  }
  
  set_io_float(ID_MCFO_COMMAND_CURRENT, target_angle * get_io_float(ID_MCFO_STIFFNESS));
  return 1; 
}

int ACT_FO_afterpush_entry()
{ 
  return 1; 
}

int ACT_FO_afterpush()
{ 
  return 1; 
}

int ACT_FO_flipup_entry()
{ 
//  set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_C_F_FU_F_ANG)); //TEMP comment, 4/6/2010
//  set_io_float(ID_MCFO_DAMPNESS, get_io_float(ID_C_F_FU_F_RATE)); //TEMP comment, 4/6/2010
  return 1;
}

int ACT_FO_flipup()
{ 
  float target_angle;
  
  set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_C_F_FU_F_ANG)); //TEMP , 4/6/2010
  set_io_float(ID_MCFO_DAMPNESS, get_io_float(ID_C_F_FU_F_RATE)); //TEMP , 4/6/2010
  
  target_angle =  get_io_float(ID_P_F_FU_F_TANG);
  set_io_float(ID_MCFO_COMMAND_CURRENT, target_angle * get_io_float(ID_MCFO_STIFFNESS));
  return 1;
}

int ACT_FO_flipdown_entry()
{ 
//  set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_C_F_FD_F_ANG)); //TEMP comment, 4/6/2010
//  set_io_float(ID_MCFO_DAMPNESS, get_io_float(ID_C_F_FD_F_RATE));  //TEMP comment, 4/6/2010
  return 1;
}

int ACT_FO_flipdown()
{ 
  float target_angle;
  
  set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_C_F_FD_F_ANG)); //TEMP , 4/6/2010
  set_io_float(ID_MCFO_DAMPNESS, get_io_float(ID_C_F_FD_F_RATE));  //TEMP , 4/6/2010
  
  
  target_angle =  get_io_float(ID_P_F_FD_F_TANG) - get_io_float(ID_MCH_ANGLE) * 0.5;
 // target_angle =  get_io_float(ID_P_F_FD_F_TANG) ; //TEMP 4/6/2010
  set_io_float(ID_MCFO_COMMAND_CURRENT, target_angle * get_io_float(ID_MCFO_STIFFNESS));
  return 1;
}

int ACT_FO_startstance_entry()
{
//  set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_C_F_SST_F_ANG)); //TEMP comment, 4/6/2010
//  set_io_float(ID_MCFO_DAMPNESS, get_io_float(ID_C_F_SST_F_RATE)); //TEMP comment, 4/6/2010
  return 1;
}

int ACT_FO_startstance()
{
  float target_angle;
  
  set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_C_F_SST_F_ANG)); //TEMP , 4/6/2010
  set_io_float(ID_MCFO_DAMPNESS, get_io_float(ID_C_F_SST_F_RATE)); //TEMP , 4/6/2010
  
  target_angle =  get_io_float(ID_P_F_SST_F_TANG) - get_io_float(ID_MCH_ANGLE) * 0.5;
 

//  if ((get_io_float(ID_MCFO_MID_ANKLE_ANGLE) - target_angle) > 0.1) //Ankle cable slackness safety
//  {
//     target_angle = get_io_float(ID_MCFO_MID_ANKLE_ANGLE) - 0.1;
//  }
  
  set_io_float(ID_MCFO_COMMAND_CURRENT, target_angle * get_io_float(ID_MCFO_STIFFNESS)); 
  return 1;
}

int ACT_FO_stop()
{  
  set_io_float(ID_MCFO_COMMAND_CURRENT, 0.0);
  set_io_float(ID_MCFO_STIFFNESS, 0.0);
  set_io_float(ID_MCFO_DAMPNESS, 0.0);
  return 1;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////



void def_foot_outer_fsm(fsm* fo_fsm)
{

//cout << "defining ankle fsm.. " << endl;

// numStates is the number of states defined in structure.h
int numStates = 7;
// numInpts is the number of inputs defined in structure.h
int numInputs = 8;

// construct the fsm
fsm fsm1(numStates, STATE_FO_startstance, numInputs);

char* name = (char*) "foot_outer_fsm";
fsm1.set_name(name);

// associate the states with actions

fsm1.state_def(STATE_FO_stance, ACT_FO_stance_entry, ACT_FO_stance, ACT_FO_stance);
fsm1.state_def(STATE_FO_prepush, ACT_FO_prepush_entry, ACT_FO_prepush, ACT_FO_prepush);
fsm1.state_def(STATE_FO_afterpush, ACT_FO_afterpush_entry, ACT_FO_afterpush, ACT_FO_afterpush);
fsm1.state_def(STATE_FO_flipup, ACT_FO_flipup_entry, ACT_FO_flipup, ACT_FO_flipup);
fsm1.state_def(STATE_FO_flipdown, ACT_FO_flipdown_entry, ACT_FO_flipdown, ACT_FO_flipdown);
fsm1.state_def(STATE_FO_stop, ACT_FO_stop, ACT_FO_stop, ACT_FO_stop);
fsm1.state_def(STATE_FO_startstance, ACT_FO_startstance_entry, ACT_FO_startstance, ACT_FO_startstance);


fsm1.print_state_transition_matrix();

// define the transitions
// a transition needs to be defined for every state for every input
// for the sensors which dont make a difference it should stay in the same state

fsm1.trans_def(STATE_FO_stance, COND_FO_stance_to_prepush, STATE_FO_prepush);
fsm1.trans_def(STATE_FO_stance, COND_FO_stance_to_afterpush, STATE_FO_afterpush);   // Emergency transition
fsm1.trans_def(STATE_FO_prepush, COND_FO_prepush_to_afterpush, STATE_FO_afterpush);
fsm1.trans_def(STATE_FO_afterpush, COND_FO_afterpush_to_flipup, STATE_FO_flipup);
fsm1.trans_def(STATE_FO_flipup, COND_FO_flipup_to_flipdown, STATE_FO_flipdown);
fsm1.trans_def(STATE_FO_flipdown, COND_FO_flipdown_to_stance, STATE_FO_stance);
fsm1.trans_def(STATE_FO_startstance, COND_FO_startstance_to_afterpush, STATE_FO_afterpush);   // Transition from start stance
fsm1.trans_def(STATE_FO_stance, COND_FO_stop, STATE_FO_stop);
fsm1.trans_def(STATE_FO_afterpush, COND_FO_stop, STATE_FO_stop);
fsm1.trans_def(STATE_FO_flipup, COND_FO_stop, STATE_FO_stop);
fsm1.trans_def(STATE_FO_flipdown, COND_FO_stop, STATE_FO_stop);

fsm1.print_state_transition_matrix();


// define exit state of fsm
fsm1.exit_state_def(STATE_FO_stop);

fsm1.set_sensor_input_function(get_foot_outer_input);

fsm1.set_state_communication_variable(&g_foot_outer_fsm_state);

//set_io_float(ID_MB_FOOT_OUTER_FSM_STATE, (float)g_foot_outer_fsm_state);

copy(fo_fsm, &fsm1);

}

