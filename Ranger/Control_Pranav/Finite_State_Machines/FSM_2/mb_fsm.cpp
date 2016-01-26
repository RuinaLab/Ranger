#include <mb_fsm.h>

#include <mb_includes.h>
#include "fsm.h"
#include "hip_fsm.h"
#include "foot_inner_fsm.h"
#include "foot_outer_fsm.h"
#include "ankle_fsm.h"
#include "top_fsm.h"
#include "system_init_fsm.h"
//#include "steering_fsm.h"
//#include "read_global_sensors.h"

//using namespace std;

//define global params

//define global sensors

int g_top_fsm_stop_command;
int g_hip_fsm_stop_command;
int g_foot_inner_fsm_stop_command;
int g_foot_outer_fsm_stop_command;
int g_ankle_fsm_stop_command;
int g_steering_fsm_stop_command;

int g_hip_fsm_state;
int g_ankle_fsm_state;
int g_foot_inner_fsm_state;
int g_foot_outer_fsm_state;
int g_top_fsm_state;
int g_system_init_fsm_state;
int g_steering_fsm_state;

int g_hip_fsm_error_code;
int g_foot_inner_fsm_error_code;
int g_foot_outer_fsm_error_code;
int g_ankle_fsm_error_code;
int g_top_fsm_error_code;
int g_steering_fsm_error_code;


// define the finite state machines
fsm* hip_fsm;
fsm* foot_inner_fsm;
fsm* foot_outer_fsm;
fsm* ankle_fsm;
fsm* top_fsm;
fsm* system_init_fsm;
//fsm* steering_fsm;

void init_global_comm(){

int g_top_fsm_stop_command = 0;
int g_hip_fsm_stop_command = 0;
int g_foot_inner_stop_command = 0;
int g_foot_outer_stop_command = 0;
int g_ankle_fsm_stop_command = 0;
int g_steering_fsm_stop_command = 0;

}

void mb_fsm_init(void){

// setup

// all the fsms have to be created as heap objects
top_fsm = new fsm();
hip_fsm = new fsm();
foot_inner_fsm = new fsm();
foot_outer_fsm = new fsm();
ankle_fsm = new fsm();
system_init_fsm = new fsm();
//steering_fsm = new fsm();

def_top_fsm(top_fsm); // top level FSM
def_hip_fsm(hip_fsm); // lower level FSM called by top_fsm
def_foot_inner_fsm(foot_inner_fsm);  // lower level FSM called by top_fsm
def_foot_outer_fsm(foot_outer_fsm);  // lower level FSM called by top_fsm
def_ankle_fsm(ankle_fsm); // lower level FSM called by top_fsm
def_system_init_fsm(system_init_fsm); // System initialization FSM called by top_fsm
//def_steering_fsm(steering_fsm); // lower level FSM called by top_fsm


init_global_comm();

// checking

top_fsm->check(1);
hip_fsm->check(1);
foot_inner_fsm->check(1);
foot_outer_fsm->check(1);
ankle_fsm->check(1);
system_init_fsm->check(1);
//steering_fsm->check(1);


}

//////////////////////////////////////////////////////////////////////////////////
void mb_fsm_run(void)
{

//read_global_sensors();
//top_fsm->run();

// **** TEST CODE **** (functions below eventually to be called from top_fsm->run().
hip_fsm->run();
set_io_float(ID_MB_HIP_FSM_STATE, (float)g_hip_fsm_state);
foot_inner_fsm->run();
set_io_float(ID_MB_FOOT_INNER_FSM_STATE, (float)g_foot_inner_fsm_state);
foot_outer_fsm->run();
set_io_float(ID_MB_FOOT_OUTER_FSM_STATE, (float)g_foot_outer_fsm_state);
system_init_fsm->run();
set_io_float(ID_MB_SYSTEM_INIT_FSM_STATE, (float)g_system_init_fsm_state);
//steering_fsm->run();
//set_io_float(ID_MB_STEERING_FSM_STATE, (float)g_steering_fsm_state);


}
//////////////////////////////////////////////////////////////////////////////////
