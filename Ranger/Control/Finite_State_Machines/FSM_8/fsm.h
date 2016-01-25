
#ifndef _FSM_H

#define _FSM_H

#include <string>

// an action takes as input an array of fsm's, and an integer denoting the length of the array
typedef int (*action)();
typedef void (*sensorInputFcn)(int*, int);

class fsm
{

public:
fsm(int numStates, int startState, int numSensors); // takes as inputs the number of 
                                                   // states, the start state and the
                                                   // number of sensor inputs
fsm(); 

~fsm();


void state_def(int state, action); // every state has to be associated 
                                   // with an action 
                                     
void state_def(int state, action, int is_exit_state); // every state has to be associated at least a main 
                                                      // action

void state_def(int state, action init_action, action main_action, action exit_action, int is_exit_state); 

void state_def(int state, action init_action, action main_action, action exit_action); 
                                                      // with three action functions. 
                                                      // where actions functions are not desired, pass NULL pointer


void trans_def(int state, int input, int newState); // this function defines a transition

void exit_state_def(int state);  // specifies that a certain state can set the 
                                 // ready to exit flag

void set_sensor_input_function(sensorInputFcn);

void set_name(char* fsm_name);

friend void copy(fsm* dest, fsm* source);

void run();

int get_current_state();

int ready_to_exit();    // this should be called to check if fsm is ready to exit

void stop();            // call this function to stop the machine

void set_state_communication_variable(int* stop_comm_var);

void print_state_transition_matrix();
void print_transition_insertion_history();

void check(int toPrint);



private:
action *m_init_action_array;
action *m_action_array;
action *m_exit_action_array;
sensorInputFcn m_sensor_input_function;

int *m_state_transition_matrix;
int *m_transition_insertion_history;
int *m_exit_state_array;
int *m_phase_in_state_array;  
int m_ready_to_exit_flag;
int m_stopped;
int m_completely_defined;
int m_current_state;
int m_next_state;
int m_num_states;
int m_num_sensors;
int m_start_state;
int m_transition_counter;
int m_first_run;

char* m_name;
int* m_state_communication_variable;

int m_current_sensor_input;
int m_sensor_array_length;
int* m_sensor_array;


};

void copy(fsm* dest, fsm* source);

#endif
