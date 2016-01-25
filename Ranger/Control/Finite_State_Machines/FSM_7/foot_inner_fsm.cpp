
//#include <iostream>
#include <mb_includes.h>
#include "fsm.h"
#include "foot_inner_fsm.h"
#include "foot_inner_fsm_conditions.h"
#include "global_params.h"
#include "global_sensors.h"
#include "global_communications.h"

//using namespace std;
static const int fi_prepush_FLAG = 1; //1 is pd on prepush, 2 is open loop prepush
static float fi_stance_to_prepush_angle;
static float fi_prepush_safety;
static float discrete_fi_prepush_dgain;
static float discrete_fi_prepush_dcurrent;
//static float discrete_fi_afterpush_dcurrent;
static float fi_ap_count;
//static int fi_fd_state, fi_fu_state;


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
  
  //target_angle = get_io_float(ID_P_F_ST_F_TANG) + get_io_float(ID_MCFI_MOTOR_POSITION) - get_io_float(ID_E_FI_ABSANG);
  target_angle =  get_io_float(ID_P_F_ST_F_TANG) + get_io_float(ID_MCH_ANGLE) * 0.5;

  float command_current = FI_command_current_limiter(target_angle * get_io_float(ID_MCFI_STIFFNESS), 1.0);
  set_io_float(ID_MCFI_COMMAND_CURRENT, command_current); 
   
  //discrete_fi_afterpush_dcurrent = get_io_float(ID_F_TEST2);  //Set afterpush current here
 
  return 1;
}

int ACT_FI_stance()
{
  float target_angle;
  
  //set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_ST_F_ANG)); //TEMP 4/6/2010
  //set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_ST_F_RATE)); //TEMP 4/6/2010
  
  // target_angle = get_io_float(ID_P_F_ST_F_TANG) + get_io_float(ID_MCFI_MOTOR_POSITION) - get_io_float(ID_E_FI_ABSANG);
  target_angle =  get_io_float(ID_P_F_ST_F_TANG) + get_io_float(ID_MCH_ANGLE) * 0.5;
  
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
  if (fi_prepush_FLAG==1)
  {
    fi_stance_to_prepush_angle = get_io_float(ID_MCFI_MID_ANKLE_ANGLE); //save the stance angle during exit
    set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_PP_F_ANG));
    set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_PP_F_RATE)); 
  }
  else if (fi_prepush_FLAG==2)
  {
     fi_prepush_safety=1.0;
   
     set_io_float(ID_MCFI_STIFFNESS, 0.0); //TEMP comment, 4/6/2010 
     set_io_float(ID_MCFI_DAMPNESS, 0.0); //TEMP comment, 4/6/2010
     
  }
  
  if (((int) get_io_float(ID_D_F_ON)) == 1)
  {
    float robot_velocity_mid_stance  = get_io_float(ID_E_MIDSTANCE_LEGRATE);
    
    if (fi_prepush_FLAG==1)
        discrete_fi_prepush_dgain = -get_io_float(ID_DC_F_PP_L_ABSRATE)*(robot_velocity_mid_stance-get_io_float(ID_DP_F_PP_L_ABSRATE) );
    else if (fi_prepush_FLAG==2)
        discrete_fi_prepush_dcurrent = -get_io_float(ID_DC_F_PP_L_ABSRATE)*(robot_velocity_mid_stance-get_io_float(ID_DP_F_PP_L_ABSRATE) );

  }
  else
  {
     if (fi_prepush_FLAG==1)
       discrete_fi_prepush_dgain = 0.0;
     else if (fi_prepush_FLAG==2)
        discrete_fi_prepush_dcurrent = 0.0;  
  }
  
  if (fi_prepush_FLAG==1)
    {set_io_float(ID_DA_F_PP_A0,(get_io_float(ID_C_F_PP_F_ANG)+discrete_fi_prepush_dgain));
     set_io_float(ID_MCFI_STIFFNESS, (get_io_float(ID_C_F_PP_F_ANG)+discrete_fi_prepush_dgain));}
  else if (fi_prepush_FLAG==2)
    set_io_float(ID_DA_F_PP_A0,discrete_fi_prepush_dcurrent);
   
  
  
  /*if (get_io_float(ID_E_INNER_ANG_RATE)<0.25)
    {
       discrete_fi_prepush_dcurrent = 0.0;//get_io_float(ID_F_TEST2);
    }
  else
    {
       discrete_fi_prepush_dcurrent = 0.0;
    }*/
      
     

  
  return 1;
}

int ACT_FI_prepush()
{ 

if (fi_prepush_FLAG==1)
 {
  float target_angle;
  //set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_PP_F_ANG)); //TEMP , 4/6/2010 
  //set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_PP_F_RATE)); //TEMP , 4/6/2010
  target_angle = fi_stance_to_prepush_angle + get_io_float(ID_P_F_PP_F_TANG);
    if (target_angle>=2.6) //Feet safety
      {target_angle=2.6;}
    if (target_angle<=1.0)
       {target_angle = 1.0;} 

  float command_current  = target_angle * get_io_float(ID_MCFI_STIFFNESS);
   // float command_current = FI_command_current_limiter(target_angle * get_io_float(ID_MCFI_STIFFNESS), get_io_float(ID_A_F_PP_A0));
  set_io_float(ID_MCFI_COMMAND_CURRENT, command_current);
 }
 
else if (fi_prepush_FLAG==2)
 {
    if (get_io_float(ID_MCFI_MID_ANKLE_ANGLE)>2.8) //safety if robot is lifted
    {
      fi_prepush_safety = 0.0;
    }
  
    if (fi_prepush_safety>0.5)
      {  
      float command_current = get_io_float(ID_A_F_PP_A0)+discrete_fi_prepush_dcurrent;   
      if (command_current >= 3.0)                                           
          {set_io_float(ID_MCFI_COMMAND_CURRENT, 3.0);}
      else if (command_current <=2.0)
          {set_io_float(ID_MCFI_COMMAND_CURRENT, 2.0);}   
      else
          {set_io_float(ID_MCFI_COMMAND_CURRENT, command_current);}     
      }
    else
    {
      set_io_float(ID_MCFI_COMMAND_CURRENT, 0.0);
    }
  
  }
//  if ((get_io_float(ID_MCFI_MID_ANKLE_ANGLE) - target_angle) > 0.1)   //Ankle cable slackness safety
//  {
//     target_angle = get_io_float(ID_MCFI_MID_ANKLE_ANGLE) - 0.1;
//  }
  
  
  return 1; 
}

int ACT_FI_afterpush_entry()
{ 
  fi_ap_count = 0.0;
  set_io_float(ID_MCFI_STIFFNESS, 0.0); 
  set_io_float(ID_MCFI_DAMPNESS, 0.0); 
  return 1; 
}

int ACT_FI_afterpush()
{ 
  fi_ap_count = fi_ap_count + 1.0;
  if (fi_ap_count<=get_io_float(ID_F_TEST1))
  {  
      float command_current = 0.0;//discrete_fi_afterpush_dcurrent;
      set_io_float(ID_MCFI_COMMAND_CURRENT, command_current);
  }
  else
  {
      set_io_float(ID_MCFI_COMMAND_CURRENT, 0.0);
  }
  return 1; 
}

int ACT_FI_flipup_entry()
{ 
   //fi_fu_state = 0;
  set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_FU_F_ANG)); 
  set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_FU_F_RATE)); 
  return 1; 
}

int ACT_FI_flipup()
{ 
  float target_angle;
  
   //set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_FU_F_ANG)); //TEMP , 4/6/2010
   //set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_FU_F_RATE)); //TEMP , 4/6/2010
   
   /*if (get_io_float(ID_MCFI_MOTOR_VELOCITY)<=get_io_float(ID_H_TEST1))
   {
     fi_fu_state = 1;
   }
  
     
   if (fi_fu_state == 0)
      {
        // P on velocity starts 
        set_io_float(ID_MCFI_STIFFNESS, 0.0);
        set_io_float(ID_MCFI_DAMPNESS, 0.0);
   
        float command_current = -get_io_float(ID_F_TEST2)*(get_io_float(ID_MCFI_MOTOR_VELOCITY)-get_io_float(ID_F_TEST1));  
        //float command_current = -get_io_float(ID_F_TEST2); 
        set_io_float(ID_MCFI_COMMAND_CURRENT, command_current );
        //  P on velocity end 
      }  
   else
    {
   //  PD on position starts //
   set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_FU_F_ANG)); //TEMP , 4/6/2010
   set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_FU_F_RATE)); //TEMP , 4/6/2010
  
   target_angle =  get_io_float(ID_P_F_FU_F_TANG); //step input

   float command_current = FI_command_current_limiter(target_angle * get_io_float(ID_MCFI_STIFFNESS), -0.3);
   set_io_float(ID_MCFI_COMMAND_CURRENT, command_current);
  //  PD on position ends //
   } */
   
  //Old flip up code 
  target_angle =  get_io_float(ID_P_F_FU_F_TANG); //step input
    
  float command_current = FI_command_current_limiter(target_angle * get_io_float(ID_MCFI_STIFFNESS), -0.6);
  set_io_float(ID_MCFI_COMMAND_CURRENT, command_current); 
  //set_io_float(ID_MCFI_COMMAND_CURRENT, target_angle * get_io_float(ID_MCFI_STIFFNESS));
  return 1;
}

int ACT_FI_flipdown_entry()
{ 
  //fi_fd_state = 0;
  set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_FD_F_ANG)); //TEMP comment, 4/6/2010
  set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_FD_F_RATE)); //TEMP comment, 4/6/2010
  return 1;
}

int ACT_FI_flipdown()
{ 
  float target_angle;
  
  //set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_FD_F_ANG)); //TEMP, 4/6/2010
  //set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_FD_F_RATE)); //TEMP, 4/6/2010
 
   /* if (get_io_float(ID_MCFI_MOTOR_VELOCITY)>=get_io_float(ID_H_TEST2))
   {
        fi_fd_state = 1;
   }   
  
     if (fi_fd_state == 0)
      {
        //  P on velocity starts 
        set_io_float(ID_MCFI_STIFFNESS, 0.0);
        set_io_float(ID_MCFI_DAMPNESS, 0.0);  
        float command_current = -get_io_float(ID_F_TEST4)*(get_io_float(ID_MCFI_MOTOR_VELOCITY)-get_io_float(ID_F_TEST3)); 
        //float command_current = get_io_float(ID_F_TEST4);
        set_io_float(ID_MCFI_COMMAND_CURRENT,command_current);
        //  P on velocity end 

      }  
   else
    {
    
    // PD on position starts//
   set_io_float(ID_MCFI_STIFFNESS, get_io_float(ID_C_F_FD_F_ANG)); //TEMP, 4/6/2010
   set_io_float(ID_MCFI_DAMPNESS, get_io_float(ID_C_F_FD_F_RATE)); //TEMP, 4/6/2010
   
   target_angle =  get_io_float(ID_P_F_FD_F_TANG) + get_io_float(ID_MCH_ANGLE) * 0.5;
   //target_angle =  get_io_float(ID_P_F_FD_F_TANG); //Control on relative angle, TEMP 4/6/2010
      
    float command_current = FI_command_current_limiter(target_angle * get_io_float(ID_MCFI_STIFFNESS), 0.4);
    set_io_float(ID_MCFI_COMMAND_CURRENT, command_current);
    }
    
    //  PD on position end // */
    
    /* Old flip down code */
 // target_angle = get_io_float(ID_P_F_FD_F_TANG) + get_io_float(ID_MCFI_MOTOR_POSITION) - get_io_float(ID_E_FI_ABSANG);
 target_angle =  get_io_float(ID_P_F_FD_F_TANG) + get_io_float(ID_MCH_ANGLE) * 0.5;
 // target_angle =  get_io_float(ID_P_F_FD_F_TANG); //Control on relative angle, TEMP 4/6/2010
 
  float command_current = FI_command_current_limiter(target_angle * get_io_float(ID_MCFI_STIFFNESS), 0.5);
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

