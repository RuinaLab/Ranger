
#include "fsm.h"
#include <stdlib.h>

//using namespace std;

fsm::fsm()
{
}

fsm::fsm(int numStates, int startState, int numSensors)
{

//cout << "constructing an fsm with " << numStates << " states and " << numSensors << " sensor inputs.." << endl;

m_current_state = startState;

m_num_states = numStates;

m_num_sensors = numSensors;

m_start_state = startState;


// allocating memory for init, main, and exit action arrays

m_init_action_array = new action[m_num_states];
for (int i=0;i<m_num_states;i++){
  m_init_action_array[i]=NULL;
}

m_action_array = new action[m_num_states];
for (int i=0;i<m_num_states;i++){
  m_action_array[i]=NULL;
}

m_exit_action_array = new action[m_num_states];
for (int i=0;i<m_num_states;i++){
  m_exit_action_array[i]=NULL;
}


// allocating memory for state transition matrix

m_state_transition_matrix = new int[numStates*numSensors];

for(int i=0;i<m_num_states;i++){
 for (int j=0;j<m_num_sensors;j++){
   m_state_transition_matrix[i*m_num_sensors+j] = -1;
 }
}


// allocating memory for transition insertion history matrix

m_transition_insertion_history = new int[numStates*numSensors];

for(int i=0;i<m_num_states;i++){
 for (int j=0;j<m_num_sensors;j++){
   m_transition_insertion_history[i*m_num_sensors+j] = -1;
 }
}

// allocating memory for exit state array

m_exit_state_array = new int[numStates];
for (int i=0;i<m_num_states;i++){
  m_exit_state_array[i] = 0;
}


// allocating memory for phase in state array

m_phase_in_state_array = new int[numStates];
for (int i=0;i<m_num_states;i++){
  m_phase_in_state_array[i] = 0;
}

// allocating memory for sensor_array

m_sensor_array = new int[numSensors];
for (int i=0;i<m_num_sensors;i++){
  m_sensor_array[i] = 0;
}


m_phase_in_state_array[startState] = 1;

// setting ready to exit flag

m_ready_to_exit_flag = 0;

// setting stopped flag

m_stopped = 1;

// setting completely defined flag

m_completely_defined = 0;

// initializing transition counter

m_transition_counter= 0;

// setting first run flag

m_first_run = 1;

// initializing name

m_name = (char *) " ";

// initializing current sensor input variable

m_current_sensor_input = -1;

}

fsm::~fsm()
{

  delete[] m_init_action_array;
  
  delete[] m_action_array;

  delete[] m_exit_action_array;
  
  delete[] m_state_transition_matrix;
  
  delete[] m_transition_insertion_history;
  
  delete[] m_exit_state_array;
 
  delete[] m_phase_in_state_array;
 
  delete[] m_sensor_array;
 
}

void fsm::set_name(char* fsm_name)
{

  m_name = fsm_name;

}

void fsm::state_def(int state, action a)
{

//cout << "associating state " << state << " with an action.." << endl; 

if (m_action_array[state]!=NULL) {
    //std::cout << "state is already associated with an action" << endl;
}

else {
   m_init_action_array[state] = a;
   m_action_array[state] = a;
   m_exit_action_array[state] = a;
}

}

void fsm::state_def(int state, action init_action, action main_action, action exit_action)
{

//cout << "associating state " << state << " with an action.." << endl; 

if (m_init_action_array[state]!=NULL) {
    //std::cout << "state is already associated with an init action" << endl;
}
else if (m_action_array[state]!=NULL) {
    //std::cout << "state is already associated with a main action" << endl;
}
else if (m_exit_action_array[state]!=NULL) {
    //std::cout << "state is already associated with an exit action" << endl;
}

else {
   m_init_action_array[state] = init_action;
   m_action_array[state] = main_action;
   m_exit_action_array[state] = exit_action;
}

}

void fsm::state_def(int state, action init_action, action main_action, action exit_action, int is_exit_state)
{

//cout << "associating state " << state << " with an action.." << endl; 

if (m_init_action_array[state]!=NULL) {
    //std::cout << "state is already associated with an init action" << endl;
  
}
else if (m_action_array[state]!=NULL) {
    //std::cout << "state is already associated with a main action" << endl;
   
}
else if (m_exit_action_array[state]!=NULL) {
    //std::cout << "state is already associated with an exit action" << endl;
 
}

else {
   m_init_action_array[state] = init_action;
   m_action_array[state] = main_action;
   m_exit_action_array[state] = exit_action;
}

if (is_exit_state) { exit_state_def(state); }

}

void fsm::state_def(int state, action a, int is_exit_state)
{

//cout << "associating state " << state << " with an action.." << endl; 

if (m_action_array[state]!=NULL) {
    //std::cout << "state is already associated with an action" << endl;
  
}

else {
   m_action_array[state] = a;
}

if (is_exit_state) { exit_state_def(state); }

}


void fsm::trans_def(int state, int input, int newState)
{

   //cout << "defining a transition: #" << m_transition_counter << endl;

   if (m_state_transition_matrix[state*m_num_sensors+input] != -1) {

   //cout << "this transition has already been defined:" << endl;
   //cout << state << " " << input << "--> "; 
   //cout << m_state_transition_matrix[state*m_num_sensors+input] << endl;

  
   }
   
   else {
	
   m_state_transition_matrix[state*m_num_sensors+input] = newState;
   m_transition_insertion_history[state*m_num_sensors+input] = m_transition_counter;

   //cout << state << " " << input << "--> "; 
   //cout << m_state_transition_matrix[state*m_num_sensors+input] << endl;

   m_transition_counter++;

   }
	
}

void fsm::exit_state_def(int state){

  m_exit_state_array[state] = 1;

}

void fsm::set_sensor_input_function(sensorInputFcn s){
  
  m_sensor_input_function = s;
  
}


void fsm::check(int toPrint)   // this function will check if the fsm is well defined
{                   // if there are inputs for which state transitions have
                    // not been defined it will assume that it stays in the 
                    // same state    
                    
int i,j;
int error = 0;

//cout << "->" << m_name << ": checking completion of fsm definition.." << endl;

if (toPrint) print_state_transition_matrix();
		
for (i=0;i<m_num_states;i++){
  for (j=0;j<m_num_sensors;j++){

	if ((m_state_transition_matrix[i*m_num_sensors+j]>= m_num_states) || (m_state_transition_matrix[i*m_num_sensors+j]<0)){
            //cout << "->" << m_name << ": ";
            //cout << "A transition has not been defined for state " << i 
            //          << " with input " << j << endl;
            //cout << "assuming self-transition" << endl;
            m_state_transition_matrix[i*m_num_sensors+j] = i;
            m_transition_insertion_history[i*m_num_sensors+j] = m_transition_counter;
            m_transition_counter++;
        }
            
     }
 }

   for (int i=1;i<m_num_states;i++) {
      if (m_action_array == NULL){
         //cout << "An action has not been defined for state " << i << endl;	
         error = 1;		
      }
   }

   if (error==0) m_completely_defined = 1;

   if (error == 1) {
 
  }	

  if (toPrint) print_state_transition_matrix();

}

void fsm::print_state_transition_matrix()
{

  //cout << "->" << m_name << ": printing state transition matrix:" << endl;
  //cout << endl;

   //cout << "         inputs" << endl;

   for (int i=0;i<m_num_states;i++){
     //cout << "state " << i << ": ";
     for (int j=0;j<m_num_sensors;j++){
        //cout << m_state_transition_matrix[i*m_num_sensors+j] << " ";
     }
     //cout << endl;
   }
   //std::cout << endl;
   
}

void fsm::print_transition_insertion_history()
{


  //cout << "->" << m_name << ": printing state transition matrix:" << endl;
  //cout << endl;

  //cout << "         inputs" << endl;

   for (int i=0;i<m_num_states;i++){
     //cout << "state " << i << ": ";
     for (int j=0;j<m_num_sensors;j++){
        //cout << m_transition_insertion_history[i*m_num_sensors+j] << " ";
     }
     //cout << endl;
   }
   //std::cout << endl;
   
}

void fsm::run()
{


   //cout << "->running " << m_name << ".. " << endl;	

   if (m_first_run == 1) {	

   //will print out the state transition matrix

   //print_state_transition_matrix();

   // will run check and tell you if the fsm is not fully defined and exit with an error

   check(0);
   
   } 

   // if the fsm is correct it will run the fsm	
   if (m_completely_defined){
   
       if (m_stopped) {
           m_current_state = m_start_state;
           for (int i=0;i<m_num_states;i++){
              m_phase_in_state_array[i] = 0;
           }
           m_phase_in_state_array[m_start_state] = 1;
           m_stopped = 0;

       }

      // communicate state information
      *m_state_communication_variable = m_current_state; 

      // get sensor input 

      m_sensor_input_function(m_sensor_array, m_num_sensors);

      int possible_transition_counter = 0;
//      int active_sensor_array[m_num_sensors];
      int active_sensor_array[100];   // **** TEST CODE ****
  
      for (int i = 0;i<m_num_sensors;i++){      
	if (m_sensor_array[i] == 1) {
            active_sensor_array[possible_transition_counter] = i;
	    possible_transition_counter++;
        }
      }

      if (possible_transition_counter == 0) {
   	 //cout << "->" << m_name << ": no active transition condition" << endl;
         // stay in current state 
         //cout << "->" << m_name << ": executing action for state " << m_current_state << endl;
         if (m_phase_in_state_array[m_current_state] == 1){
	    if (m_init_action_array[m_current_state] != NULL) {
               //cout << "->" << m_name << ": start action for state " << m_current_state << endl;
               m_init_action_array[m_current_state](); 
            }
            if (m_action_array[m_current_state] != NULL) {
               //cout << "->" << m_name << ": main action for state " << m_current_state << endl;
               m_action_array[m_current_state](); 
            }
            m_phase_in_state_array[m_current_state] = 2;
          }
         else if (m_phase_in_state_array[m_current_state] == 2){
            if (m_action_array[m_current_state] != NULL) {
               //cout << "->" << m_name << ": main action for state " << m_current_state << endl;
               m_action_array[m_current_state](); 
            }
         }

      }
    
      else {
      
      if (possible_transition_counter == 1){
         m_current_sensor_input = active_sensor_array[0];
      }

      else {
      int current_transition = active_sensor_array[0]; 
      for (int j=1;j<possible_transition_counter;j++){
         if (m_transition_insertion_history[m_current_state*m_num_sensors+active_sensor_array[j]] <  m_transition_insertion_history[m_current_state*m_num_sensors+current_transition]) {
            current_transition = active_sensor_array[j];
         }
      }          
       
      m_current_sensor_input = current_transition;      
      } 
  
      //cout << "->" << m_name << ": executing action for state " << m_current_state << endl;
 
      // do action for current state
      if ((m_phase_in_state_array[m_current_state] == 1)&& 
          (m_state_transition_matrix[m_current_state*m_num_sensors+m_current_sensor_input] == m_current_state)){
	  if (m_init_action_array[m_current_state] != NULL) {
              //cout << "->" << m_name << ": start action for state " << m_current_state << endl;
              m_init_action_array[m_current_state]();
          }  
          if (m_action_array[m_current_state] != NULL) {
              //cout << "->" << m_name << ": main action for state " << m_current_state << endl;   
              m_action_array[m_current_state]();               
          }
          m_phase_in_state_array[m_current_state] = 2;
      }
      else if ((m_phase_in_state_array[m_current_state] == 1)&& 
          (m_state_transition_matrix[m_current_state*m_num_sensors+m_current_sensor_input] != m_current_state)){
	  if (m_init_action_array[m_current_state] != NULL) {
              //cout << "->" << m_name << ": start action for state " << m_current_state << endl;
              m_init_action_array[m_current_state]();
          }  
          if (m_action_array[m_current_state] != NULL) {
              //cout << "->" << m_name << ": main action for state " << m_current_state << endl;   
              m_action_array[m_current_state]();               
          }
          if (m_exit_action_array[m_current_state] != NULL) {
              //cout << "->" << m_name << ": exit action for state " << m_current_state << endl;   
              m_exit_action_array[m_current_state]();               
          }
          m_phase_in_state_array[m_current_state] = 0;
      }
      else if ((m_phase_in_state_array[m_current_state] == 2) && 
               (m_state_transition_matrix[m_current_state*m_num_sensors+m_current_sensor_input] == m_current_state)){
           if (m_action_array[m_current_state] != NULL) {
               //cout << "->" << m_name << ": main action for state " << m_current_state << endl;
               m_action_array[m_current_state](); 
          }
      }
      else if ((m_phase_in_state_array[m_current_state] == 2) && 
               (m_state_transition_matrix[m_current_state*m_num_sensors+m_current_sensor_input] != m_current_state)){
           if (m_exit_action_array[m_current_state] != NULL) {
               //cout << "->" << m_name << ": exit action for state " << m_current_state << endl;
               m_exit_action_array[m_current_state](); 
           }
           m_phase_in_state_array[m_current_state] = 0;
      }
  
     // transition to next state 
     m_next_state = m_state_transition_matrix[m_current_state*m_num_sensors+m_current_sensor_input];
     //std::cout << "->" << m_name << ": transition to state " << m_next_state << endl;
     if (m_current_state != m_next_state) {
         m_current_state = m_next_state;
         m_phase_in_state_array[m_current_state] = 1;
         if (m_exit_state_array[m_current_state] == 1)
             m_ready_to_exit_flag = 1;

         //cout << "->" << m_name << ": executing action for state " << m_current_state << endl;  
         if (m_init_action_array[m_current_state] != NULL) {
              //cout << "->" << m_name << ": start action for state " << m_current_state << endl;
              m_init_action_array[m_current_state]();
         }  
         if (m_action_array[m_current_state] != NULL) {   
              //cout << "->" << m_name << ": main action for state " << m_current_state << endl;
              m_action_array[m_current_state]();               
         }
         m_phase_in_state_array[m_current_state] = 2;
     }

    } // else

     if (m_first_run == 1) m_first_run = 0;	
      
  } // if


} // function

int fsm::get_current_state()
{

   return m_current_state;

}

int fsm::ready_to_exit()
{
 
  if (m_ready_to_exit_flag){
      //cout << "->" << m_name << ": ready to exit" << endl;
  }

  return m_ready_to_exit_flag;

}

void fsm::stop()
{
  
   m_stopped = 1;

}


void fsm::set_state_communication_variable(int* state_comm_var)
{

    m_state_communication_variable = state_comm_var;

}

// friend helper function

void copy(fsm* dest, fsm* source)
{


//cout << "copying an fsm with " << source->m_num_states << " states and " << source->m_num_sensors << " sensor inputs.." << endl;

// copying init, main, and exit action arrays

dest->m_init_action_array = new action[source->m_num_states];
for (int i=0;i<source->m_num_states;i++){
  dest->m_init_action_array[i]=source->m_init_action_array[i];
}


dest->m_action_array = new action[source->m_num_states];
for (int i=0;i<source->m_num_states;i++){
  dest->m_action_array[i]=source->m_action_array[i];
}

dest->m_exit_action_array = new action[source->m_num_states];
for (int i=0;i<source->m_num_states;i++){
  dest->m_exit_action_array[i]=source->m_exit_action_array[i];
}

// copying sensor input function

dest->m_sensor_input_function = source->m_sensor_input_function;

// copying state transition matrix

dest->m_state_transition_matrix = new int[source->m_num_states*source->m_num_sensors];

for(int i=0;i<source->m_num_states;i++){
 for (int j=0;j<source->m_num_sensors;j++){
   dest->m_state_transition_matrix[i*source->m_num_sensors+j] = source->m_state_transition_matrix[i*source->m_num_sensors+j];
 }
}

// copying transition insertion history matrix

dest->m_transition_insertion_history = new int[source->m_num_states*source->m_num_sensors];

for(int i=0;i<source->m_num_states;i++){
 for (int j=0;j<source->m_num_sensors;j++){
   dest->m_transition_insertion_history[i*source->m_num_sensors+j] = source->m_transition_insertion_history[i*source->m_num_sensors+j];
 }
}

// copying exit state array

dest->m_exit_state_array = new int[source->m_num_states];

for (int i=0;i<source->m_num_states;i++){
  dest->m_exit_state_array[i] = source->m_exit_state_array[i];
}

// copying phase in state array

dest->m_phase_in_state_array = new int[source->m_num_states];

for (int i=0;i<source->m_num_states;i++){
  dest->m_phase_in_state_array[i] = source->m_phase_in_state_array[i];
}

// copying ready to exit flag

dest->m_ready_to_exit_flag = source->m_ready_to_exit_flag;

// copying stopped flag

dest->m_stopped = source->m_stopped;

// copying completely defined flag

dest->m_completely_defined = source->m_completely_defined;

// copying current state

dest->m_current_state = source->m_current_state;

// copying next state

dest->m_next_state = source->m_next_state;

// copying number of states, and number or inputs

dest->m_num_states = source->m_num_states;

dest->m_num_sensors = source->m_num_sensors;

// copying start state

dest->m_start_state = source->m_start_state;

// copying transition counter

dest->m_transition_counter= source->m_transition_counter;

// copying first run flag

dest->m_first_run = source->m_first_run;

// copying name

dest->m_name = source->m_name;

// copy state communication variable 

dest->m_state_communication_variable = source->m_state_communication_variable;


// copying sensor input

dest->m_current_sensor_input = source->m_current_sensor_input;

// copying sensor array

dest->m_sensor_array = new int[source->m_num_sensors];
for (int i=0;i<source->m_num_sensors;i++) {
    dest->m_sensor_array[i] = source->m_sensor_array[i];
 }

}  
  



