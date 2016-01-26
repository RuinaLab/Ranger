
//#include <iostream>
#include <mb_includes.h>
#include "fsm.h"
#include "foot_inner_fsm.h"
#include "foot_inner_fsm_conditions.h"
#include "global_params.h"
#include "global_sensors.h"
#include "global_communications.h"

//using namespace std;
static float stance_to_prepush_angle;
//////////////////////////////////////////////////////////////////////////////////////////////////////
// Define action functions

int ACT_FI_stance_entry()
{
//  set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_ST_F_ANG)); //TEMP comment, 4/6/2010
//  set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_ST_F_RATE)); //TEMP comment, 4/6/2010
  return 1;
}

int ACT_FI_stance()
{
  float target_angle;
  
  set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_ST_F_ANG)); //TEMP 4/6/2010
  set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_ST_F_RATE)); //TEMP 4/6/2010
  
  target_angle =  get_io_float(ID_P_F_ST_F_TANG) + get_io_float(ID_MCH_ANGLE) * 0.5;

//  if ((get_io_float(ID_MCFI_MID_ANKLE_ANGLE) - target_angle) > 0.1) //Ankle cable slackness safety
//  {
//     target_angle = get_io_float(ID_MCFI_MID_ANKLE_ANGLE) - 0.1;
//  }
  
  set_io_float(ID_MCFI_COMMAND_CURRENT, target_angle * get_io_float(ID_MCFI_STIFFNESS)); 
  return 1;
}

int ACT_FI_prepush_entry()
{
    stance_to_prepush_angle = get_io_float(ID_MCFI_MID_ANKLE_ANGLE); //save the stance angle during exit
//  set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_PP_F_ANG)); //TEMP comment, 4/6/2010 
//  set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_PP_F_RATE)); //TEMP comment, 4/6/2010
  return 1;
}

int ACT_FI_prepush()
{ 
  float target_angle;
  
  set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_PP_F_ANG)); //TEMP , 4/6/2010 
  set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_PP_F_RATE)); //TEMP , 4/6/2010
  
  target_angle = stance_to_prepush_angle + get_io_float(ID_P_F_PP_F_TANG);
//  target_angle =  get_io_float(ID_P_F_PP_F_TANG) + get_io_float(ID_MCH_ANGLE) * 0.5;
//  target_angle =  get_io_float(ID_P_F_PP_F_TANG); //Based on relative angle, 4/27/2010

//  if ((get_io_float(ID_MCFI_MID_ANKLE_ANGLE) - target_angle) > 0.1)   //Ankle cable slackness safety
//  {
//     target_angle = get_io_float(ID_MCFI_MID_ANKLE_ANGLE) - 0.1;
//  }
  
  set_io_float(ID_MCFI_COMMAND_CURRENT, target_angle * get_io_float(ID_MCFI_STIFFNESS));
  return 1; 
}

int ACT_FI_afterpush_entry()
{ 
  return 1; 
}

int ACT_FI_afterpush()
{ 
  return 1; 
}

int ACT_FI_flipup_entry()
{ 
 // set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_FU_F_ANG)); //TEMP comment, 4/6/2010
 // set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_FU_F_RATE)); //TEMP comment, 4/6/2010
  return 1; 
}

int ACT_FI_flipup()
{ 
  float target_angle;
  
   set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_FU_F_ANG)); //TEMP , 4/6/2010
   set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_FU_F_RATE)); //TEMP , 4/6/2010
  
  target_angle =  get_io_float(ID_P_F_FU_F_TANG);
  set_io_float(ID_MCFI_COMMAND_CURRENT, target_angle * get_io_float(ID_MCFI_STIFFNESS));
  return 1;
}

int ACT_FI_flipdown_entry()
{ 
//  set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_FD_F_ANG)); //TEMP comment, 4/6/2010
//  set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_FD_F_RATE)); //TEMP comment, 4/6/2010
  return 1;
}

int ACT_FI_flipdown()
{ 
  float target_angle;
  
  set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_FD_F_ANG)); //TEMP, 4/6/2010
  set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_FD_F_RATE)); //TEMP, 4/6/2010
 
 // set_io_ul(ID_MCFI_STIFFNESS, 0x40800000); //TEMP, 4/7/2010 =4
 // set_io_ul(ID_MCFI_DAMPNESS,  0x3E4CCCCC); //TEMP, 4/7/2010 =0.2
  
  target_angle =  get_io_float(ID_P_F_FD_F_TANG) + get_io_float(ID_MCH_ANGLE) * 0.5;
  //target_angle =  get_io_float(ID_P_F_FD_F_TANG); //Control on relative angle, TEMP 4/6/2010
  set_io_float(ID_MCFI_COMMAND_CURRENT, target_angle * get_io_float(ID_MCFI_STIFFNESS));
 //set_io_ul(ID_MCFI_COMMAND_CURRENT, 0x40E66666); //TEMP 4/7/2010 = 7.2
 
 
  return 1;
}

int ACT_FI_stop()
{  
  set_io_float(ID_MCFI_COMMAND_CURRENT, 0.0);
  set_io_float(ID_MCFI_STIFFNESS, 0.0);
  set_io_float(ID_MCFI_DAMPNESS, 0.0);
  return 1;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////


void def_foot_inner_fsm(fsm* fi_fsm)
{

//cout << "defining ankle fsm.. " << endl;

// numStates is the number of states defined in structure.h
int numStates = 6;
// numInpts is the number of inputs defined in structure.h
int numInputs = 7;

// construct the fsm
fsm fsm1(numStates, STATE_FI_flipdown, numInputs);

char* name = (char*) "foot_inner_fsm";
fsm1.set_name(name);

// associate the states with actions

fsm1.state_def(STATE_FI_stance, ACT_FI_stance_entry, ACT_FI_stance, ACT_FI_stance);
fsm1.state_def(STATE_FI_prepush, ACT_FI_prepush_entry, ACT_FI_prepush, ACT_FI_prepush);
fsm1.state_def(STATE_FI_afterpush, ACT_FI_afterpush_entry, ACT_FI_afterpush, ACT_FI_afterpush);
fsm1.state_def(STATE_FI_flipup, ACT_FI_flipup_entry, ACT_FI_flipup, ACT_FI_flipup);
fsm1.state_def(STATE_FI_flipdown, ACT_FI_flipdown_entry, ACT_FI_flipdown, ACT_FI_flipdown);
fsm1.state_def(STATE_FI_stop, ACT_FI_stop, ACT_FI_stop, ACT_FI_stop);


fsm1.print_state_transition_matrix();

// define the transitions
// a transition needs to be defined for every state for every input
// for the sensors which dont make a difference it should stay in the same state

fsm1.trans_def(STATE_FI_stance, COND_FI_stance_to_prepush, STATE_FI_prepush);
fsm1.trans_def(STATE_FI_stance, COND_FI_stance_to_afterpush, STATE_FI_afterpush);   //Emergency transition
fsm1.trans_def(STATE_FI_prepush, COND_FI_prepush_to_afterpush, STATE_FI_afterpush);
fsm1.trans_def(STATE_FI_afterpush, COND_FI_afterpush_to_flipup, STATE_FI_flipup);
fsm1.trans_def(STATE_FI_flipup, COND_FI_flipup_to_flipdown, STATE_FI_flipdown);
fsm1.trans_def(STATE_FI_flipdown, COND_FI_flipdown_to_stance, STATE_FI_stance);
fsm1.trans_def(STATE_FI_stance, COND_FI_stop, STATE_FI_stop);
fsm1.trans_def(STATE_FI_afterpush, COND_FI_stop, STATE_FI_stop);
fsm1.trans_def(STATE_FI_flipup, COND_FI_stop, STATE_FI_stop);
fsm1.trans_def(STATE_FI_flipdown, COND_FI_stop, STATE_FI_stop);

fsm1.print_state_transition_matrix();


// define exit state of fsm
fsm1.exit_state_def(STATE_FI_stop);

fsm1.set_sensor_input_function(get_foot_inner_input);

fsm1.set_state_communication_variable(&g_foot_inner_fsm_state);

//set_io_float(ID_MB_FOOT_INNER_FSM_STATE, (float)g_foot_inner_fsm_state);

copy(fi_fsm, &fsm1);

}

