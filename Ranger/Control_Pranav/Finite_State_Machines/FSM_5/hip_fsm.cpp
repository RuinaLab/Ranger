//#include <iostream>
#include <mb_includes.h>
#include "fsm.h"
#include "hip_fsm_conditions.h"
#include "hip_fsm.h"
#include "global_sensors.h"
#include "global_params.h"
#include "global_communications.h"



int ACT_HI_starthold_entry(void)
{  //wont work if parameters are loaded from labview after starting the machine
  //set_io_float(ID_MCH_STIFFNESS, get_io_float(ID_C_HI_SH_H_ANG));
  //set_io_float(ID_MCH_DAMPNESS, get_io_float(ID_C_HI_SH_H_RATE));
  //set_io_float(ID_MCH_COMMAND_CURRENT, get_io_float(ID_P_HI_SH_H_TANG) * get_io_float(ID_C_HI_SH_H_ANG));  // Target angle in radians 
  return 1;
}

int ACT_HI_starthold(void)
{
  set_io_float(ID_MCH_STIFFNESS, get_io_float(ID_C_HI_SH_H_ANG));
  set_io_float(ID_MCH_DAMPNESS, get_io_float(ID_C_HI_SH_H_RATE));
  set_io_float(ID_MCH_COMMAND_CURRENT, get_io_float(ID_P_HI_SH_H_TANG) * get_io_float(ID_C_HI_SH_H_ANG));  // Target angle in radians
////  set_io_float(ID_MCH_COMMAND_CURRENT, get_io_float(ID_A_H_TEST_A0));
  return 1;
}

int ACT_HI_preswing(void)
{
  set_io_float(ID_MCH_COMMAND_CURRENT, 0.0);
  return 1;
}

int ACT_HI_premid_entry(void)
{
  set_io_float(ID_MCH_STIFFNESS, 0);
  set_io_float(ID_MCH_DAMPNESS, 0);
//  set_io_float(ID_MCH_COMMAND_CURRENT, 0.0);
  return 1;
}

int ACT_HI_premid(void)
{
float command_current;

   //if ((get_io_float(ID_MCH_ANGLE)) < 0.0)
   if (1)
   {
   //command_current = get_io_float(ID_A_H_PM_A0) - 0.001*get_io_float(ID_A_H_PM_A1) * get_io_float(ID_E_T_AFTER_HS);
      command_current = get_io_float(ID_A_H_PM_A0);
   } 
   else 
   {
      command_current = 0.0;
   } 
   
   if (command_current < 0.0)
         {command_current = 0.0;}
         
   set_io_float(ID_MCH_COMMAND_CURRENT, command_current);
               
  //        set_io_float(ID_MCH_COMMAND_CURRENT, get_io_float(ID_A_H_PM_A0));          
 //   }
//else   
//    {
//         set_io_float(ID_MCH_COMMAND_CURRENT,0.0);
//     }            
return 1;
}


int ACT_HI_aftermid_entry(void)
{
  set_io_float(ID_MCH_STIFFNESS, 0);
  set_io_float(ID_MCH_DAMPNESS, 0);
  set_io_float(ID_MCH_COMMAND_CURRENT, 0.0);
  return 1;
}

int ACT_HI_aftermid(void)
{

// set_io_float(ID_MCH_COMMAND_CURRENT, get_io_float(ID_A_H_PM_A0));   
 
//  set_io_float(ID_MCH_STIFFNESS, 0);
//  set_io_float(ID_MCH_DAMPNESS, 0);
//  set_io_float(ID_MCH_COMMAND_CURRENT, 0.0);
  return 1;
}

int ACT_HO_preswing(void)
{
  set_io_float(ID_MCH_COMMAND_CURRENT, 0.0);
  return 1;
}

int ACT_HO_premid_entry(void)
{
  set_io_float(ID_MCH_STIFFNESS, 0);
  set_io_float(ID_MCH_DAMPNESS, 0);
//  set_io_float(ID_MCH_COMMAND_CURRENT, 0.0);
  return 1;
}

int ACT_HO_premid(void)
{

float command_current;


//if ((get_io_float(ID_MCH_ANGLE)) > 0.0)
if (1)
   {
   //command_current = -(get_io_float(ID_A_H_PM_A0) - 0.001*get_io_float(ID_A_H_PM_A1) * get_io_float(ID_E_T_AFTER_HS));
   command_current = -get_io_float(ID_A_H_PM_A0);
   }
   else
   {
   command_current =0.0;
   }
   
   if (command_current > 0.0)
      {command_current = 0.0;}           
      
   set_io_float(ID_MCH_COMMAND_CURRENT, command_current);   

  return 1;
}

int ACT_HO_aftermid_entry(void)
{
  set_io_float(ID_MCH_STIFFNESS, 0);
  set_io_float(ID_MCH_DAMPNESS, 0);
  set_io_float(ID_MCH_COMMAND_CURRENT, 0.0);
  return 1;
}

int ACT_HO_aftermid(void)
{
//  set_io_float(ID_MCH_STIFFNESS, 0);
//  set_io_float(ID_MCH_DAMPNESS, 0);
//  set_io_float(ID_MCH_COMMAND_CURRENT, 0.0);
//  set_io_float(ID_MCH_COMMAND_CURRENT, -get_io_float(ID_A_H_PM_A0));  
  return 1;
}

int ACT_HI_ehold_entry(void)
{
  set_io_float(ID_MCH_STIFFNESS, get_io_float(ID_C_H_EH_H_ANG));
  set_io_float(ID_MCH_DAMPNESS, get_io_float(ID_C_H_EH_H_RATE));
  set_io_float(ID_MCH_COMMAND_CURRENT, get_io_float(ID_P_H_EH_H_TANG) * get_io_float(ID_C_H_EH_H_ANG));  // Target angle in radians
  return 1;
}

int ACT_HI_ehold(void)
{
//  set_io_float(ID_MCH_STIFFNESS, get_io_float(ID_C_H_EH_H_ANG));
//  set_io_float(ID_MCH_DAMPNESS, get_io_float(ID_C_H_EH_H_RATE));
//  set_io_float(ID_MCH_COMMAND_CURRENT, get_io_float(ID_P_H_EH_H_TANG) * get_io_float(ID_C_H_EH_H_ANG));  // Target angle in radians
  return 1;
}

int ACT_HO_ehold_entry(void)
{
  set_io_float(ID_MCH_STIFFNESS, get_io_float(ID_C_H_EH_H_ANG));
  set_io_float(ID_MCH_DAMPNESS, get_io_float(ID_C_H_EH_H_RATE));
  set_io_float(ID_MCH_COMMAND_CURRENT, - get_io_float(ID_P_H_EH_H_TANG) * get_io_float(ID_C_H_EH_H_ANG));  // Target angle in radians
  return 1;
}

int ACT_HO_ehold(void)
{
//  set_io_float(ID_MCH_STIFFNESS, get_io_float(ID_C_H_EH_H_ANG));
//  set_io_float(ID_MCH_DAMPNESS, get_io_float(ID_C_H_EH_H_RATE));
//  set_io_float(ID_MCH_COMMAND_CURRENT, - get_io_float(ID_P_H_EH_H_TANG) * get_io_float(ID_C_H_EH_H_ANG));  // Target angle in radians
  return 1;
}

int ACT_H_stop(void)
{
  set_io_float(ID_MCH_STIFFNESS, 0);
  set_io_float(ID_MCH_DAMPNESS, 0);
  set_io_float(ID_MCH_COMMAND_CURRENT, 0.0);
  return 1;
}


void def_hip_fsm(fsm* h_fsm)
{

//cout << endl;
//cout << "defining hip_fsm..  " << endl;

// numStates is the number of states defined in structure.h
int numStates = 10;
// numInpts is the number of inputs defined in structure.h
int numInputs = 12;

// construct the fsm

// define start state
fsm fsm1(numStates, STATE_HI_starthold, numInputs);

char* name = (char*) "hip_fsm";
fsm1.set_name(name);

//debug
fsm1.print_state_transition_matrix();

// associate the states with actions
fsm1.state_def(STATE_HI_starthold, ACT_HI_starthold_entry, ACT_HI_starthold, ACT_HI_starthold);
fsm1.state_def(STATE_HI_preswing, ACT_HI_preswing, ACT_HI_preswing, ACT_HI_preswing);
fsm1.state_def(STATE_HI_premid, ACT_HI_premid_entry, ACT_HI_premid, ACT_HI_premid);
fsm1.state_def(STATE_HI_aftermid, ACT_HI_aftermid_entry, ACT_HI_aftermid, ACT_HI_aftermid);
fsm1.state_def(STATE_HO_preswing, ACT_HO_preswing, ACT_HO_preswing, ACT_HO_preswing);
fsm1.state_def(STATE_HO_premid, ACT_HO_premid_entry, ACT_HO_premid, ACT_HO_premid);
fsm1.state_def(STATE_HO_aftermid, ACT_HO_aftermid_entry, ACT_HO_aftermid, ACT_HO_aftermid);
fsm1.state_def(STATE_H_stop, ACT_H_stop, ACT_H_stop, ACT_H_stop);
fsm1.state_def(STATE_HI_ehold, ACT_HI_ehold_entry, ACT_HI_ehold, ACT_HI_ehold);  //Emergency inner hold state
fsm1.state_def(STATE_HO_ehold, ACT_HO_ehold_entry, ACT_HO_ehold, ACT_HO_ehold);  //Emergency outer hold state


fsm1.print_state_transition_matrix();

// define the transitions
// a transition needs to be defined for every state for every input
// for the sensors which dont make a difference it should stay in the same state

fsm1.trans_def(STATE_HI_starthold, COND_HI_starthold_to_HO_preswing, STATE_HO_preswing);
fsm1.trans_def(STATE_HO_preswing, COND_HO_preswing_to_HO_premid, STATE_HO_premid);
fsm1.trans_def(STATE_HO_premid, COND_HO_premid_to_HO_aftermid, STATE_HO_aftermid);
fsm1.trans_def(STATE_HO_aftermid, COND_HO_aftermid_to_HO_ehold, STATE_HO_ehold);
fsm1.trans_def(STATE_HO_aftermid, COND_HO_aftermid_to_HI_preswing, STATE_HI_preswing); //Added 4/4/2010
fsm1.trans_def(STATE_HO_ehold, COND_HO_ehold_to_HI_preswing, STATE_HI_preswing);
fsm1.trans_def(STATE_HI_preswing, COND_HI_preswing_to_HI_premid, STATE_HI_premid);
fsm1.trans_def(STATE_HI_premid, COND_HI_premid_to_HI_aftermid, STATE_HI_aftermid);
fsm1.trans_def(STATE_HI_aftermid, COND_HI_aftermid_to_HI_ehold, STATE_HI_ehold);
fsm1.trans_def(STATE_HI_aftermid, COND_HI_aftermid_to_HO_preswing, STATE_HO_preswing); //Added 4/4/2010
fsm1.trans_def(STATE_HI_ehold, COND_HI_ehold_to_HO_preswing, STATE_HO_preswing);
fsm1.trans_def(STATE_HI_preswing, COND_H_stop, STATE_H_stop);
fsm1.trans_def(STATE_HO_preswing, COND_H_stop, STATE_H_stop);

fsm1.print_state_transition_matrix();


// define exit state of fsm
fsm1.exit_state_def(STATE_H_stop);

fsm1.set_sensor_input_function(get_hip_sensor_input);

fsm1.set_state_communication_variable(&g_hip_fsm_state);

//set_io_float(ID_MB_HIP_FSM_STATE, (float)g_hip_fsm_state);

copy(h_fsm, &fsm1);

}

