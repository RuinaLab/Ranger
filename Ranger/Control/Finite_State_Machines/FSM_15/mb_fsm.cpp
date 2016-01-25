#include "mb_fsm.h"

#include <mb_includes.h>
#include "fsm.h"

#include "def_fsm.h"

//using namespace std;


//int g_hip_fsm_stop_command;
//int g_foot_inner_fsm_stop_command;
//int g_foot_outer_fsm_stop_command;
//int g_steering_fsm_stop_command;
//int g_ui_fsm_stop_command;

int g_hip_fsm_state;
int g_foot_inner_fsm_state;
int g_foot_outer_fsm_state;
//int g_system_init_fsm_state;
int g_steering_fsm_state;
int g_ui_fsm_state;
int g_leg_fsm_state;


//int g_hip_fsm_error_code;
//int g_foot_inner_fsm_error_code;
//int g_foot_outer_fsm_error_code;
//int g_steering_fsm_error_code;
//int g_ui_fsm_error_code;

// define the finite state machines
fsm* hip_fsm;
fsm* foot_inner_fsm;
fsm* foot_outer_fsm;
//fsm* system_init_fsm;
fsm* steering_fsm;
fsm* ui_fsm;
fsm* leg_fsm;

void init_global_comm(){
//int g_hip_fsm_stop_command = 0;
//int g_foot_inner_stop_command = 0;
//int g_foot_outer_stop_command = 0;
//int g_steering_fsm_stop_command = 0;
//int g_ui_fsm_stop_command = 0;
}


void mb_fsm_init(void){

// setup

// all the fsms have to be created as heap objects
hip_fsm = new fsm();
foot_inner_fsm = new fsm();
foot_outer_fsm = new fsm();
//system_init_fsm = new fsm();
steering_fsm = new fsm();
ui_fsm = new fsm();
leg_fsm = new fsm();

def_hip_fsm(hip_fsm); // lower level FSM called by top_fsm
def_foot_inner_fsm(foot_inner_fsm);  // lower level FSM called by top_fsm
def_foot_outer_fsm(foot_outer_fsm);  // lower level FSM called by top_fsm
//def_system_init_fsm(system_init_fsm); // System initialization FSM called by top_fsm
def_steering_fsm(steering_fsm); // lower level FSM called by top_fsm
def_UI_fsm(ui_fsm);
def_leg_fsm(leg_fsm);


//init_global_comm();

// checking

hip_fsm->check(1);
foot_inner_fsm->check(1);
foot_outer_fsm->check(1);
//system_init_fsm->check(1);
steering_fsm->check(1);
ui_fsm->check(1);
leg_fsm->check(1);


}

//////////////////////////////////////////////////////////////////////////////////
void mb_fsm_run(void)
{

//read_global_sensors();
//top_fsm->run();


/* These fsm's are run in ui_fsm.cpp */
// **** TEST CODE **** (functions below eventually to be called from top_fsm->run().
//hip_fsm->run();
//set_io_float(ID_MB_HIP_FSM_STATE, (float)g_hip_fsm_state);
//
//foot_inner_fsm->run();
//set_io_float(ID_MB_FOOT_INNER_FSM_STATE, (float)g_foot_inner_fsm_state);
//
//foot_outer_fsm->run();
//set_io_float(ID_MB_FOOT_OUTER_FSM_STATE, (float)g_foot_outer_fsm_state);

//steering_fsm->run();
//set_io_float(ID_MB_STEERING_FSM_STATE, (float)g_steering_fsm_state);


//system_init_fsm->run();
//if ( (int)get_io_float(ID_MB_SYSTEM_INIT_FSM_STATE) != g_system_init_fsm_state) 
//  set_io_float(ID_MB_SYSTEM_INIT_FSM_STATE, (float)g_system_init_fsm_state);

ui_fsm->run();
if ((int) get_io_float(ID_MB_UI_FSM_STATE) != g_ui_fsm_state )
  set_io_float(ID_MB_UI_FSM_STATE, (float) g_ui_fsm_state);


}

