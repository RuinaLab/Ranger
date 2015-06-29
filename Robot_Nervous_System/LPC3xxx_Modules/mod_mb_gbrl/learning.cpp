/*
	Gradient Based Reinforcement Learning algorithm
	to optimize control parameters for different environments
	onboard Ranger.
	
	@author Nicolas Champagne-Williamson
	@date September 2010
*/

#include <mb_includes.h>

// ****** VARIABLES **************************************
//the policy parameters we are optimizing
#define GBRL_PARAMS_SIZE (2) //change this to match gbrl_params
static unsigned short gbrl_params[GBRL_PARAMS_SIZE] = {
  ID_P_H_PM_TIME,
  ID_P_H_PM_H_TRATE
  };
 
static int gbrl_index = -1; //the current parameter; -1 means find baseline

//the gradient vector
static float gbrl_grad[GBRL_PARAMS_SIZE];

//the perturbation 
static float gbrl_pert_delta; //magnitude of perturbation, in percent
static float gbrl_pert_saved; //old value before the perturbation
static float gbrl_pert; //the perturbation 

//the number of steps to average over
static unsigned long int gbrl_max_steps;
static unsigned long int gbrl_start_step = 0; //number of steps at beginning of averaging
static unsigned long int gbrl_curr_step = 0; //current number of steps

//reward function estimator variables
static unsigned long int gbrl_reward_baseline = 0.0;


// ****** FUNCTIONS **************************************

/**
  Runs the gbrl algorithm.
  Gets average gradient from all parameters, then
  updates the control policy.
*/
void gbrl(void){


  gbrl_max_steps = get_io_float(ID_GBRL_MAX_STEPS); //update parameter from labview
  gbrl_pert_delta = get_io_float(ID_GBRL_DELTA); //update parameter from labview
  gbrl_curr_step = get_io_float(ID_E_STEP_NO); //get the current step number
  
  if (gbrl_curr_step - gbrl_start_step < gbrl_max_steps){ //still averaging
    gbrl_average_reward();
  } else { //done averaging
    gbrl_start_step = gbrl_curr_step; //get start step number; reset steps

    if (gbrl_index < 0){ //found the baseline
      set_io_float(ID_GBRL_CURRENT_PARAM, -1);
      gbrl_reward_baseline = gbrl_calc_reward();
    } else { //found some gradient   
      set_io_float(ID_GBRL_CURRENT_PARAM, gbrl_params[gbrl_index]);
      gbrl_calc_grad(); //find change from perturbation
      gbrl_deperturb(); //restore parameter
    }

    gbrl_index++;
    if (gbrl_index < GBRL_PARAMS_SIZE){ //still checking params
      gbrl_perturb(); 
    } else { //done averaging all params
      gbrl_update_policy();
      gbrl_index = -1;
    }
  } 
}

/**
  Finds the change in output with the perturbed
  versus the old parameter values.
*/
void gbrl_calc_grad(void){
  float reward = gbrl_calc_reward();
  float change = reward - gbrl_reward_baseline;
  float grad = change; // / gbrl_pert;
  gbrl_grad[gbrl_index] = grad;
}

/**
  Perturb the control parameter given by
  gbrl_index.
*/
void gbrl_perturb(void){
  unsigned short param = gbrl_params[gbrl_index]; //get the current parameter
  
  gbrl_pert_saved = get_io_float(param); //save the old value before perturbing
  gbrl_pert = gbrl_pert_saved * gbrl_pert_delta; //calculate the amount of perturbation
  
  set_io_float(param, gbrl_pert_saved + gbrl_pert); //set the current parameter to perturbed
}

/**
  Restores the old value of the control parameter
  before perturbation.
*/
void gbrl_deperturb(void){
  unsigned short param = gbrl_params[gbrl_index]; //get the current parameter
  set_io_float(param, gbrl_pert_saved); //set the current parameter to pre-perturbed value
}

/**
  Updates the control policy to optimize the output.
*/
void gbrl_update_policy(void){
  int i;
  
  //Calculate the sum of gradients
  float total_grad = 0.0;
  for (i = 0; i < GBRL_PARAMS_SIZE; i++){
    total_grad += (gbrl_grad[i] < 0 ? -gbrl_grad[i] : gbrl_grad[i]);
  }
  
  
  //Calculate the total percent delta we're moving
  float scaling = 2.0;
  float total_delta = get_io_float(ID_GBRL_DELTA) * GBRL_PARAMS_SIZE * scaling;
  // ******** TEST **********
  //set_io_float(ID_T_TEST6, total_delta);
  // ******** TEST **********
  
  
  for (i = 0; i < GBRL_PARAMS_SIZE; i++){
    unsigned short param = gbrl_params[i]; //get the parameter
    float grad = gbrl_grad[i]; //get the gradient for that parameter
    float percent = 1.0 + ((grad / total_grad) * total_delta); //find the proportion of contribution to total gradient
    float value = get_io_float(param); //get the current value of the parameter
    value = value * percent; //update the value of the parameter
    
    // ******** TEST **********
    if (i == 0){
      set_io_float(ID_T_TEST1, value);
      set_io_float(ID_T_TEST2, gbrl_grad[0]);
    }
    if (i == 1){
      set_io_float(ID_T_TEST3, value);
      set_io_float(ID_T_TEST4, gbrl_grad[1]);
    }
    // ******** TEST **********
    
    
    set_io_float(param, value); //set the parameter
  }
}








