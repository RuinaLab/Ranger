#include <mb_includes.h>
//#include <iostream>
#include "fsm.h"
#include "foot_outer_fsm.h"
#include "foot_outer_fsm_conditions.h"
#include "global_params.h"
#include "global_sensors.h"
#include "global_communications.h"

//using namespace std;
static const int fo_prepush_FLAG = 1; //1 is pd on prepush, 2 is open loop prepush
static float fo_stance_to_prepush_angle;
static float fo_prepush_safety;
static float discrete_fo_prepush_dgain;
static float discrete_fo_prepush_dcurrent;
//static float discrete_fo_prepush_dcurrent;
//static float discrete_fo_afterpush_dcurrent;
static float fo_ap_count;
//static int fo_fd_state, fo_fu_state;
//////////////////////////////////////////////////////////////////////////////////////////////////////

float FO_command_current_limiter(float command_current, float I_lim)
{

float PDval = get_io_float(ID_MCFO_STIFFNESS)*get_io_float(ID_MCFO_MOTOR_POSITION)
              +get_io_float(ID_MCFO_DAMPNESS)*get_io_float(ID_MCFO_MOTOR_VELOCITY);
float I = command_current - PDval;
                      
 if (I_lim > 0) //check for upper bound
    {
    if (I>I_lim) //upper bound reached?
      {command_current = I_lim + PDval;} //change upper bound
    }
 else if (I_lim < 0) //check for lower bound
    {
    if (I<I_lim) //lower bound reached?
      {command_current = I_lim + PDval;} //change lower bound
    }  
      
 return command_current;     
      
}


int ACT_FO_stance_entry()
{
  float target_angle;
  
  set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_C_F_ST_F_ANG)); //TEMP comment, 4/6/2010
  set_io_float(ID_MCFO_DAMPNESS, get_io_float(ID_C_F_ST_F_RATE)); //TEMP comment, 4/6/2010
  
  //target_angle = get_io_float(ID_P_F_ST_F_TANG) + get_io_float(ID_MCFO_MOTOR_POSITION) - get_io_float(ID_E_FO_ABSANG);
  target_angle = get_io_float(ID_P_F_ST_F_TANG) - get_io_float(ID_MCH_ANGLE) * 0.5;;  
    
  float command_current = FO_command_current_limiter(target_angle * get_io_float(ID_MCFO_STIFFNESS), 1.0);
  set_io_float(ID_MCFO_COMMAND_CURRENT, command_current);
  
  //discrete_fo_afterpush_dcurrent = get_io_float(ID_F_TEST2);  //Set afterpush current here
  
  return 1;
}

int ACT_FO_stance()
{
  float target_angle;
  
  //set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_C_F_ST_F_ANG)); //TEMP , 4/6/2010
  //set_io_float(ID_MCFO_DAMPNESS, get_io_float(ID_C_F_ST_F_RATE)); //TEMP , 4/6/2010
  
  
  //target_angle = get_io_float(ID_P_F_ST_F_TANG) + get_io_float(ID_MCFO_MOTOR_POSITION) - get_io_float(ID_E_FO_ABSANG);
  target_angle =  get_io_float(ID_P_F_ST_F_TANG) - get_io_float(ID_MCH_ANGLE) * 0.5;
  //  target_angle =  get_io_float(ID_P_F_ST_F_TANG); //Temp 5/3/2010 for feet asymmetry testing

//  if ((get_io_float(ID_MCFO_MID_ANKLE_ANGLE) - target_angle) > 0.1) //Ankle cable slackness safety
//  {
//     target_angle = get_io_float(ID_MCFO_MID_ANKLE_ANGLE) - 0.1;
//  }
  
  float command_current = FO_command_current_limiter(target_angle * get_io_float(ID_MCFO_STIFFNESS), 1.0);
  set_io_float(ID_MCFO_COMMAND_CURRENT, command_current);
  //set_io_float(ID_MCFO_COMMAND_CURRENT, target_angle * get_io_float(ID_MCFO_STIFFNESS)); 
  return 1;
}

int ACT_FO_prepush_entry()

{

  if (fo_prepush_FLAG==1)
  {
  fo_stance_to_prepush_angle = get_io_float(ID_MCFO_RIGHT_ANKLE_ANGLE); //save the stance angle during exit
  set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_C_F_PP_F_ANG)); 
  set_io_float(ID_MCFO_DAMPNESS, get_io_float(ID_C_F_PP_F_RATE));
  }
  
  else if (fo_prepush_FLAG==2)
  {  
   fo_prepush_safety = 1.0;
   set_io_float(ID_MCFO_STIFFNESS, 0.0); //TEMP comment, 4/6/2010
   set_io_float(ID_MCFO_DAMPNESS, 0.0); //TEMP comment, 4/6/2010
  }
  
  if (((int) get_io_float(ID_D_F_ON)) == 1)
  {
    float robot_velocity_mid_stance  = get_io_float(ID_E_MIDSTANCE_LEGRATE);
    
    if (fo_prepush_FLAG==1)  
      discrete_fo_prepush_dgain = -get_io_float(ID_DC_F_PP_L_ABSRATE)*(robot_velocity_mid_stance-get_io_float(ID_DP_F_PP_L_ABSRATE) );
      
    else if (fo_prepush_FLAG==1)
      discrete_fo_prepush_dcurrent = -get_io_float(ID_DC_F_PP_L_ABSRATE)*(robot_velocity_mid_stance-get_io_float(ID_DP_F_PP_L_ABSRATE) );
     
  }
  else
  {
     if (fo_prepush_FLAG==1)
        discrete_fo_prepush_dgain = 0.0;
      else if (fo_prepush_FLAG==2) 
         discrete_fo_prepush_dcurrent = 0.0;
    
  }  
  
  if (fo_prepush_FLAG==1)
    {set_io_float(ID_DA_F_PP_A0,(get_io_float(ID_C_F_PP_F_ANG)+discrete_fo_prepush_dgain));
     set_io_float(ID_MCFO_STIFFNESS, (get_io_float(ID_C_F_PP_F_ANG)+discrete_fo_prepush_dgain)); }
  else if (fo_prepush_FLAG == 2)
    set_io_float(ID_DA_F_PP_A0,discrete_fo_prepush_dcurrent);
 
 /* if (get_io_float(ID_E_OUTER_ANG_RATE)<0.25)
    {
       discrete_fo_prepush_dcurrent = 0.0;//get_io_float(ID_F_TEST2);
    }
  else
    {
       discrete_fo_prepush_dcurrent = 0.0;
    }*/
      
  return 1;
}

int ACT_FO_prepush()
{ 

  if (fo_prepush_FLAG==1)
  {
    float target_angle;
    //set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_C_F_PP_F_ANG)); //TEMP , 4/6/2010
    //set_io_float(ID_MCFO_DAMPNESS, get_io_float(ID_C_F_PP_F_RATE)); //TEMP , 4/6/2010
    target_angle = fo_stance_to_prepush_angle + get_io_float(ID_P_F_PP_F_TANG); 
    if (target_angle>=2.6) //Feet safety
      {target_angle=2.6;}
    if (target_angle<=1.0)
       {target_angle = 1.0;}  

   float command_current = target_angle * get_io_float(ID_MCFO_STIFFNESS);
   //float command_current = FO_command_current_limiter(target_angle * get_io_float(ID_MCFO_STIFFNESS), get_io_float(ID_A_F_PP_A0));
   set_io_float(ID_MCFO_COMMAND_CURRENT, command_current);
  }
  else if (fo_prepush_FLAG==2)
  { 
    if (get_io_float(ID_MCFO_RIGHT_ANKLE_ANGLE)>2.8) //safety if robot is lifted
    {
      fo_prepush_safety = 0.0;
    }
  
    if (fo_prepush_safety>=0.3)
    {  
      float command_current = get_io_float(ID_A_F_PP_A0)+discrete_fo_prepush_dcurrent;   
      if (command_current >= 3.0)                                           
          {set_io_float(ID_MCFO_COMMAND_CURRENT, 3.0);}
      else if (command_current <=2.0)
          {set_io_float(ID_MCFO_COMMAND_CURRENT, 2.0);}
      else
          {set_io_float(ID_MCFO_COMMAND_CURRENT, command_current);} 
    }
    else
    {
        set_io_float(ID_MCFO_COMMAND_CURRENT, 0.0);
     }
  }
  
  
  //  if ((get_io_float(ID_MCFO_MID_ANKLE_ANGLE) - target_angle) > 0.1)   //Ankle cable slackness safety
//  {
//     target_angle = get_io_float(ID_MCFO_MID_ANKLE_ANGLE) - 0.1;
//  }

  return 1; 
}

int ACT_FO_afterpush_entry()
{ 
  fo_ap_count = 0.0;
  set_io_float(ID_MCFO_STIFFNESS, 0.0);
  set_io_float(ID_MCFO_DAMPNESS, 0.0); 
  return 1; 
}

int ACT_FO_afterpush()
{ 
  fo_ap_count = fo_ap_count + 1.0;
  
  if (fo_ap_count<=get_io_float(ID_F_TEST1))
  {  
    float command_current = 0.0;//discrete_fo_afterpush_dcurrent;                                            
    set_io_float(ID_MCFO_COMMAND_CURRENT, command_current);
  }
  else
  {
        set_io_float(ID_MCFO_COMMAND_CURRENT, 0.0);
  }
  return 1; 
}

int ACT_FO_flipup_entry()
{ 
  //fo_fu_state = 0;
  set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_C_F_FU_F_ANG)); //TEMP comment, 4/6/2010
  set_io_float(ID_MCFO_DAMPNESS, get_io_float(ID_C_F_FU_F_RATE)); //TEMP comment, 4/6/2010
  return 1;
}

int ACT_FO_flipup()
{ 
  float target_angle;
  
  //set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_C_F_FU_F_ANG)); //TEMP , 4/6/2010
  //set_io_float(ID_MCFO_DAMPNESS, get_io_float(ID_C_F_FU_F_RATE)); //TEMP , 4/6/2010
  
   
   /*  if (get_io_float(ID_MCFO_MOTOR_VELOCITY)<=get_io_float(ID_H_TEST1))
   {
     fo_fu_state = 1;
   }
  
   if (fo_fu_state == 0)
      {
   //  P on velocity starts 
        set_io_float(ID_MCFO_STIFFNESS, 0.0);
        set_io_float(ID_MCFO_DAMPNESS, 0.0);
   
        float command_current = -get_io_float(ID_F_TEST2)*(get_io_float(ID_MCFO_MOTOR_VELOCITY)-get_io_float(ID_F_TEST1));  
        //float command_current = -get_io_float(ID_F_TEST2); 
        set_io_float(ID_MCFO_COMMAND_CURRENT, command_current );
   //  P on velocity end 
      } 
  
  else
  {
  //  PD on position starts //
   set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_C_F_FU_F_ANG)); 
   set_io_float(ID_MCFO_DAMPNESS, get_io_float(ID_C_F_FU_F_RATE)); 
  
   target_angle =  get_io_float(ID_P_F_FU_F_TANG);

   float command_current = FO_command_current_limiter(target_angle * get_io_float(ID_MCFO_STIFFNESS), -0.3);
   set_io_float(ID_MCFO_COMMAND_CURRENT, command_current);
   //  PD on position ends 
  } */
  
   /* Old flip up code */
  target_angle =  get_io_float(ID_P_F_FU_F_TANG);
  
  float command_current = FO_command_current_limiter(target_angle * get_io_float(ID_MCFO_STIFFNESS), -0.6);
  set_io_float(ID_MCFO_COMMAND_CURRENT, command_current);
  //set_io_float(ID_MCFO_COMMAND_CURRENT, target_angle * get_io_float(ID_MCFO_STIFFNESS));
  
  return 1;
}

int ACT_FO_flipdown_entry()
{ 
  //fo_fd_state = 0;
  set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_C_F_FD_F_ANG)); //TEMP comment, 4/6/2010
  set_io_float(ID_MCFO_DAMPNESS, get_io_float(ID_C_F_FD_F_RATE));  //TEMP comment, 4/6/2010
  return 1;
}

int ACT_FO_flipdown()
{ 
  float target_angle;
  
  //set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_C_F_FD_F_ANG)); //TEMP , 4/6/2010
  //set_io_float(ID_MCFO_DAMPNESS, get_io_float(ID_C_F_FD_F_RATE));  //TEMP , 4/6/2010
  
 /*  if (get_io_float(ID_MCFO_MOTOR_VELOCITY)>=get_io_float(ID_H_TEST2))
   {
        fo_fd_state = 1;
   }   
  
  if (fo_fd_state == 0)
      {
        //  P on velocity starts
        set_io_float(ID_MCFO_STIFFNESS, 0.0);
        set_io_float(ID_MCFO_DAMPNESS, 0.0);  
        float command_current = -get_io_float(ID_F_TEST4)*(get_io_float(ID_MCFO_MOTOR_VELOCITY)-get_io_float(ID_F_TEST3)); 
        //float command_current = get_io_float(ID_F_TEST4);
        set_io_float(ID_MCFO_COMMAND_CURRENT,command_current);
        //  P on velocity end  
      }  
  else
  {
  //  PD on position starts //
    set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_C_F_FD_F_ANG)); 
    set_io_float(ID_MCFO_DAMPNESS, get_io_float(ID_C_F_FD_F_RATE));  
    
    target_angle =  get_io_float(ID_P_F_FD_F_TANG) - get_io_float(ID_MCH_ANGLE) * 0.5;
    //target_angle =  get_io_float(ID_P_F_FD_F_TANG) ; //TEMP 4/6/2010
    
    float command_current = FO_command_current_limiter(target_angle * get_io_float(ID_MCFO_STIFFNESS), 0.4);
    set_io_float(ID_MCFO_COMMAND_CURRENT, command_current);
    //set_io_float(ID_MCFO_COMMAND_CURRENT, target_angle * get_io_float(ID_MCFO_STIFFNESS));
  //  PD on position end //
  }  */

    /* Old flip down code */
   //target_angle = get_io_float(ID_P_F_FD_F_TANG) + get_io_float(ID_MCFO_MOTOR_POSITION) - get_io_float(ID_E_FO_ABSANG);
  target_angle =  get_io_float(ID_P_F_FD_F_TANG) - get_io_float(ID_MCH_ANGLE) * 0.5;
 // target_angle =  get_io_float(ID_P_F_FD_F_TANG) ; //TEMP 4/6/2010
 
   float command_current = FO_command_current_limiter(target_angle * get_io_float(ID_MCFO_STIFFNESS),0.5);
  set_io_float(ID_MCFO_COMMAND_CURRENT, command_current);
  //set_io_float(ID_MCFO_COMMAND_CURRENT, target_angle * get_io_float(ID_MCFO_STIFFNESS));
  return 1;
}

int ACT_FO_startstance_entry()
{
  //set_io_float(ID_MCFO_STIFFNESS, get_io_float(ID_C_F_SST_F_ANG)); //TEMP comment, 4/6/2010
  //set_io_float(ID_MCFO_DAMPNESS, get_io_float(ID_C_F_SST_F_RATE)); //TEMP comment, 4/6/2010
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

