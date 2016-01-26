//#include <iostream>
#include <mb_includes.h>
#include "fsm.h"
#include "hip_fsm.h"
#include "hip_fsm_conditions.h"

#include "global_sensors.h"
#include "global_params.h"
#include "global_communications.h"

int counter_ehold2swing;
int counter_swing;
int counter_free;
//static const int time_swing = 100;


int ACT_H_free_entry(void)
{
  counter_free = 0;
  return 1;
}

int ACT_H_free(void)
{
  counter_free = counter_free + 2;
  
  //float command_current = get_io_float(ID_H_TEST1)*get_io_float(ID_E_H_RATE);
  float command_current  = get_io_float(ID_H_TEST1);
  
  set_io_float(ID_MCH_COMMAND_CURRENT, command_current);
  set_io_float(ID_MCH_STIFFNESS, 0);
  set_io_float(ID_MCH_DAMPNESS, 0);
  
  set_io_float(ID_E_TEST5,0);
  
  
  return 1;
}

int ACT_H_free_exit(void)
{
  counter_free = 0;
  return 1;
}


int ACT_HI_swing_entry(void)
{
  counter_swing = 0;
//  set_io_float(ID_MCH_COMMAND_CURRENT, get_io_float(ID_A_H_PM_A0) );  
  set_io_float(ID_MCH_COMMAND_CURRENT, 0);
  set_io_float(ID_MCH_STIFFNESS, 0);
  set_io_float(ID_MCH_DAMPNESS, 0);
  return 1;
}

int ACT_HI_swing(void)
{
  counter_swing = counter_swing + 2;
  
  set_io_float(ID_MCH_STIFFNESS, 0);
  set_io_float(ID_MCH_DAMPNESS, 0);
//  set_io_float(ID_MCH_DAMPNESS, -get_io_float(ID_H_TEST5)); //negative damping. 

//set_io_float(ID_MCH_COMMAND_CURRENT, get_io_float(ID_A_H_PM_A0) );
   set_io_float(ID_MCH_COMMAND_CURRENT, get_io_float(ID_H_TEST5) );
 //   set_io_float(ID_MCH_COMMAND_CURRENT, 0 );
   
   float GK = (1.32+get_io_float(ID_F_TEST1)), I = get_io_float(ID_H_TEST5);
   float Iinv = 1/I;
   
   float R = (get_io_float(ID_MCH_BATT_POWER)*Iinv - GK * get_io_float(ID_MCH_MOTOR_VELOCITY)) * Iinv;
   set_io_float(ID_E_TEST5,R);
   
  return 1;
}

int ACT_HI_swing_exit(void)
{
  counter_swing = 0;
  return 1;
}

int ACT_HI_ehold_entry(void)
{
   counter_ehold2swing = 0;
  
  //set_io_float(ID_MCH_STIFFNESS, 0);
  //set_io_float(ID_MCH_DAMPNESS, 0);
  //set_io_float(ID_MCH_COMMAND_CURRENT,get_io_float(ID_H_TEST2));
  
    //set_io_float(ID_MCH_STIFFNESS, get_io_float(ID_C_H_EH_H_ANG));
  //set_io_float(ID_MCH_DAMPNESS, get_io_float(ID_C_H_EH_H_RATE));
  //set_io_float(ID_MCH_COMMAND_CURRENT, get_io_float(ID_P_H_EH_H_TANG) * get_io_float(ID_C_H_EH_H_ANG));  // Target angle in radians
  return 1;
}

int ACT_HI_ehold(void)
{

counter_ehold2swing = counter_ehold2swing + 1;

 set_io_float(ID_MCH_STIFFNESS, 0);
 set_io_float(ID_MCH_DAMPNESS, 0);
 set_io_float(ID_MCH_COMMAND_CURRENT,get_io_float(ID_H_TEST2));

 // set_io_float(ID_MCH_STIFFNESS, get_io_float(ID_C_H_EH_H_ANG));
 // set_io_float(ID_MCH_DAMPNESS, get_io_float(ID_C_H_EH_H_RATE));
 // set_io_float(ID_MCH_COMMAND_CURRENT, get_io_float(ID_P_H_EH_H_TANG) * get_io_float(ID_C_H_EH_H_ANG));  // Target angle in radians
  return 1;
}

//===================================================/
 /******** HO not used **************/
 
int ACT_HO_swing_entry(void)
{
  counter_swing = 0;
  //set_io_float(ID_MCH_COMMAND_CURRENT, -get_io_float(ID_A_H_PM_A0) );
  set_io_float(ID_MCH_STIFFNESS, 0);
  set_io_float(ID_MCH_DAMPNESS, 0);
  return 1;
}

int ACT_HO_swing(void)
{
  set_io_float(ID_MCH_STIFFNESS, 0);
  set_io_float(ID_MCH_DAMPNESS, 0);
/*  if (counter_swing<time_swing)
   {
    set_io_float(ID_MCH_COMMAND_CURRENT, -get_io_float(ID_A_H_PM_A0) );
   }
  else
  {
    counter_swing = counter_swing + 1;
    set_io_float(ID_MCH_COMMAND_CURRENT, 0 );
   }*/
  //set_io_float(ID_MCH_STIFFNESS, 0);
  //set_io_float(ID_MCH_DAMPNESS, 0);
  return 1;
}

int ACT_HO_ehold_entry(void)
{
  counter_ehold2swing = 0;
  
  set_io_float(ID_MCH_STIFFNESS, get_io_float(ID_C_H_EH_H_ANG));
  set_io_float(ID_MCH_DAMPNESS, get_io_float(ID_C_H_EH_H_RATE));
  set_io_float(ID_MCH_COMMAND_CURRENT, - get_io_float(ID_P_H_EH_H_TANG) * get_io_float(ID_C_H_EH_H_ANG));  // Target angle in radians
  return 1;
}

int ACT_HO_ehold(void)
{
counter_ehold2swing = counter_ehold2swing + 1;
//  set_io_float(ID_MCH_STIFFNESS, get_io_float(ID_C_H_EH_H_ANG));
//  set_io_float(ID_MCH_DAMPNESS, get_io_float(ID_C_H_EH_H_RATE));
//  set_io_float(ID_MCH_COMMAND_CURRENT, - get_io_float(ID_P_H_EH_H_TANG) * get_io_float(ID_C_H_EH_H_ANG));  // Target angle in radians
  return 1;
}
 /******** HO not used **************/
 //===================================================/
 
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
int numStates = 6;
// numInpts is the number of inputs defined in structure.h
int numInputs = 9;

// construct the fsm

// define start state
fsm fsm1(numStates, STATE_H_free, numInputs);

char* name = (char*) "hip_fsm";
fsm1.set_name(name);

//debug
fsm1.print_state_transition_matrix();

// associate the states with actions
fsm1.state_def(STATE_H_free, ACT_H_free_entry, ACT_H_free, ACT_H_free_exit);
fsm1.state_def(STATE_HI_ehold, ACT_HI_ehold_entry, ACT_HI_ehold, ACT_HI_ehold);
fsm1.state_def(STATE_HI_swing, ACT_HI_swing_entry, ACT_HI_swing, ACT_HI_swing_exit);
fsm1.state_def(STATE_HO_ehold, ACT_HO_ehold_entry, ACT_HO_ehold, ACT_HO_ehold);
fsm1.state_def(STATE_HO_swing, ACT_HO_swing_entry, ACT_HO_swing, ACT_HO_swing);
fsm1.state_def(STATE_H_stop, ACT_H_stop, ACT_H_stop, ACT_H_stop);

fsm1.print_state_transition_matrix();

// define the transitions
// a transition needs to be defined for every state for every input
// for the sensors which dont make a difference it should stay in the same state

//fsm1.trans_def(STATE_H_free, COND_H_free_to_HI_ehold, STATE_HI_ehold);
//fsm1.trans_def(STATE_HI_ehold, COND_HI_ehold_to_HI_swing, STATE_HI_swing);
fsm1.trans_def(STATE_HI_swing, COND_HI_swing_to_HI_free, STATE_H_free);
fsm1.trans_def(STATE_H_free, COND_HI_free_to_HI_swing, STATE_HI_swing);
//fsm1.trans_def(STATE_H_free, COND_H_free_to_HO_ehold, STATE_HO_ehold);
//fsm1.trans_def(STATE_HO_ehold, COND_HO_ehold_to_HO_swing, STATE_HO_swing);
//fsm1.trans_def(STATE_HO_swing, COND_HO_swing_to_HO_free, STATE_H_free);
//fsm1.trans_def(STATE_HI_ehold, COND_HI_ehold_to_H_free, STATE_H_free);

fsm1.print_state_transition_matrix();


// define exit state of fsm
fsm1.exit_state_def(STATE_H_stop);

fsm1.set_sensor_input_function(get_hip_sensor_input);

fsm1.set_state_communication_variable(&g_hip_fsm_state);

//set_io_float(ID_MB_HIP_FSM_STATE, (float)g_hip_fsm_state);

copy(h_fsm, &fsm1);

}

