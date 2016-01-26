/* global_communications.h
   
   This file will be used to define the global communication variables which 
   will be used to communicate between the fsms. 

*/

#ifndef _GLOBAL_COMMUNICATIONS_H
#define _GLOBAL_COMMUNICATIONS_H


// stop commands
extern int g_top_fsm_stop_command;
extern int g_hip_fsm_stop_command;
extern int g_foot_inner_stop_command;
extern int g_foot_outer_stop_command;
extern int g_ankle_fsm_stop_command;
extern int g_steering_fsm_stop_command;

// states all the finite state machines are in
extern int g_hip_fsm_state;
extern int g_foot_inner_fsm_state;
extern int g_foot_outer_fsm_state;
extern int g_ankle_fsm_state;
extern int g_top_fsm_state;
extern int g_system_init_fsm_state;
extern int g_steering_fsm_state;


// error reporting
extern int g_hip_fsm_error_code;
extern int g_foot_inner_fsm_error_code;
extern int g_foot_outer_fsm_error_code;
extern int g_ankle_fsm_error_code;
extern int g_top_fsm_error_code;
extern int g_steering_error_code;

#endif
