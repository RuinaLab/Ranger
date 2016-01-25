
//#include <iostream>
#include <mb_includes.h>
#include "fsm.h"
#include "foot_inner_fsm.h"
#include "foot_inner_fsm_conditions.h"
#include "global_params.h"
#include "global_sensors.h"
#include "global_communications.h"

//using namespace std;
//static float fi_stance_to_prepush_angle;
static float counter_fi_prepush;
static float fi_prepush_safety;
static float prepush_max_current = 2.0;

float FI_command_current_limiter(float command_current, float I_lim)
{

float PDval = get_io_float(ID_MCFI_STIFFNESS)*get_io_float(ID_MCFI_MOTOR_POSITION)
              +get_io_float(ID_MCFI_DAMPNESS)*get_io_float(ID_MCFI_MOTOR_VELOCITY);
float I = command_current - PDval;
                      
 if (I_lim > 0.0) //check for upper bound
    {
    if (I>I_lim) //upper bound reached?
      {command_current = I_lim + PDval;} //change upper bound
    }
 else if (I_lim < 0.0) //check for lower bound
    {
    if (I<I_lim) //lower bound reached?
      {command_current = I_lim + PDval;} //change lower bound
    }  
      
 return command_current;     
      
}

//////////////////////////////////////////////////////////////////////////////////////////////////////


int ACT_FI_stance_entry()
{
  float target_angle;
  
  set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_ST_F_ANG)); //TEMP comment, 4/6/2010
  set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_ST_F_RATE)); //TEMP comment, 4/6/2010
  
  target_angle =  get_io_float(ID_P_F_ST_F_TANG) + get_io_float(ID_MCH_ANGLE) * 0.5;
   
  float command_current = FI_command_current_limiter(target_angle * get_io_float(ID_MCFI_STIFFNESS), 1.0);
  set_io_float(ID_MCFI_COMMAND_CURRENT, command_current); 
   
 
  return 1;
}

int ACT_FI_stance()
{
  float target_angle;
  
  //set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_ST_F_ANG)); //TEMP 4/6/2010
  //set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_ST_F_RATE)); //TEMP 4/6/2010
  
  target_angle =  get_io_float(ID_P_F_ST_F_TANG) + get_io_float(ID_MCH_ANGLE) * 0.5;
  // target_angle =  get_io_float(ID_P_F_ST_F_TANG); //Temp 5/3/2010 for feet asymmetry testing
  
//  if ((get_io_float(ID_MCFI_MID_ANKLE_ANGLE) - target_angle) > 0.1) //Ankle cable slackness safety
//  {
//     target_angle = get_io_float(ID_MCFI_MID_ANKLE_ANGLE) - 0.1;
//  }
  
  float command_current = FI_command_current_limiter(target_angle * get_io_float(ID_MCFI_STIFFNESS), 1.0);
  set_io_float(ID_MCFI_COMMAND_CURRENT, command_current);
  //set_io_float(ID_MCFI_COMMAND_CURRENT, target_angle * get_io_float(ID_MCFI_STIFFNESS)); 
  return 1;
}

int ACT_FI_prepush_entry()
{
  //fi_stance_to_prepush_angle = get_io_float(ID_MCFI_MID_ANKLE_ANGLE); //save the stance angle during exit
  //set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_PP_F_ANG)); //TEMP comment, 4/6/2010 
  //set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_PP_F_RATE)); //TEMP comment, 4/6/2010
  
  fi_prepush_safety=1.0;
  counter_fi_prepush = 0;
  set_io_float(ID_MCFI_STIFFNESS, 0.0); //TEMP comment, 4/6/2010 
  set_io_float(ID_MCFI_DAMPNESS, 0.0); //TEMP comment, 4/6/2010
  
  return 1;
}

int ACT_FI_prepush()
{ 
  //float target_angle;
   float prepush_rate = 1.0; //Put a can_id for this
  
  
  //set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_PP_F_ANG)); //TEMP , 4/6/2010 
  //set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_PP_F_RATE)); //TEMP , 4/6/2010
  
  //target_angle = fi_stance_to_prepush_angle + get_io_float(ID_P_F_PP_F_TANG);
 // target_angle =  get_io_float(ID_P_F_PP_F_TANG) + get_io_float(ID_MCH_ANGLE) * 0.5;
//  target_angle =  get_io_float(ID_P_F_PP_F_TANG); //Based on relative angle, 4/27/2010

//  if ((get_io_float(ID_MCFI_MID_ANKLE_ANGLE) - target_angle) > 0.1)   //Ankle cable slackness safety
//  {
//     target_angle = get_io_float(ID_MCFI_MID_ANKLE_ANGLE) - 0.1;
//  }
  

  //float command_current = FI_command_current_limiter(target_angle * get_io_float(ID_MCFI_STIFFNESS), 2.0);
  counter_fi_prepush = counter_fi_prepush + 1;

  //set_io_float(ID_E_TEST2,fi_prepush_safety);
  //set_io_float(ID_E_TEST3,get_io_float(ID_MCFI_LEFT_HS) + get_io_float(ID_MCFI_RIGHT_HS));
  //if (get_io_float(ID_MCFI_MID_ANKLE_ANGLE)>2.8) //safety if robot is lifted
  //if (get_io_float(ID_MCFI_LEFT_LS) + get_io_float(ID_MCFI_RIGHT_LS) >= 1.0) //safety if robot is lifted
  if (get_io_float(ID_MCFI_LEFT_HS) + get_io_float(ID_MCFI_RIGHT_HS) <= 0.1) //safety if robot is lifted
  {
    set_io_float(ID_MCFI_COMMAND_CURRENT, 0.0);
    fi_prepush_safety = 0.0;
  }
  
  if (fi_prepush_safety==1.0)
  {
      float command_current = FI_command_current_limiter(2.0*counter_fi_prepush*prepush_rate, prepush_max_current);
      set_io_float(ID_MCFI_COMMAND_CURRENT, command_current);
  }
  
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
  set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_FU_F_ANG)); //TEMP comment, 4/6/2010
  set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_FU_F_RATE)); //TEMP comment, 4/6/2010
  return 1; 
}

int ACT_FI_flipup()
{ 
  float target_angle;
  
   //set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_FU_F_ANG)); //TEMP , 4/6/2010
   //set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_FU_F_RATE)); //TEMP , 4/6/2010
  
  target_angle =  get_io_float(ID_P_F_FU_F_TANG); //step input
    
  float command_current = FI_command_current_limiter(target_angle * get_io_float(ID_MCFI_STIFFNESS), -0.5);
  set_io_float(ID_MCFI_COMMAND_CURRENT, command_current); 
  //set_io_float(ID_MCFI_COMMAND_CURRENT, target_angle * get_io_float(ID_MCFI_STIFFNESS));
  return 1;
}

int ACT_FI_flipdown_entry()
{ 
   set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_FD_F_ANG)); //TEMP comment, 4/6/2010
   set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_FD_F_RATE)); //TEMP comment, 4/6/2010
  return 1;
}

int ACT_FI_flipdown()
{ 
  float target_angle;
  
  //set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_FD_F_ANG)); //TEMP, 4/6/2010
  //set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_FD_F_RATE)); //TEMP, 4/6/2010
 
  
 target_angle =  get_io_float(ID_P_F_FD_F_TANG) + get_io_float(ID_MCH_ANGLE) * 0.5;
 // target_angle =  get_io_float(ID_P_F_FD_F_TANG); //Control on relative angle, TEMP 4/6/2010
 
  float command_current = FI_command_current_limiter(target_angle * get_io_float(ID_MCFI_STIFFNESS), 1.0);
  set_io_float(ID_MCFI_COMMAND_CURRENT, command_current);
  //set_io_float(ID_MCFI_COMMAND_CURRENT, target_angle * get_io_float(ID_MCFI_STIFFNESS));
 
 
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

