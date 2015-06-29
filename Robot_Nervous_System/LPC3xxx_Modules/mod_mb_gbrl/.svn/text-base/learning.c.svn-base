/*
	Gradient Based Reinforcement Learning algorithm
	to optimize control parameters for different environments
	onboard Ranger.
	
	@author Nicolas Champagne-Williamson
	@date September 2010
*/

#include <mb_includes.h>

//Parameters for GBRL - CHANGE THESE
#define GBRL_DELTA (0.02)
#define GBRL_MAX_STEPS (2)


// ************************************************************
//the policy parameters we are optimizing
#define GBRL_PARAMS_SIZE (1) //change this to match gbrl_params
static unsigned short gbrl_params[GBRL_PARAMS_SIZE] = {
  ID_P_H_PM_H_TRATE
  };
  
static int gbrl_index = -1; //the current parameter; -1 means find baseline

//the gradient vector
static float gbrl_grad[GBRL_PARAMS_SIZE];

//the perturbation 
static float gbrl_pert_delta = GBRL_DELTA; //magnitude of perturbation, in percent
static float gbrl_pert_saved; //old value before the perturbation
static float gbrl_pert; //the perturbation 

//the number of steps to average over
static unsigned long int gbrl_max_steps = GBRL_MAX_STEPS;
static unsigned long int gbrl_start_step = 0; //number of steps at beginning of averaging
static unsigned long int gbrl_curr_step = 0; //current number of steps

//averaging variables - power consumption
static float gbrl_power_sum = 0.0; //sum of power consumption
static int gbrl_power_count = 0; //number of data points
static float gbrl_power_baseline = 0.0; //the baseline power consumption without any perturbations

/**
  Runs the gbrl algorithm.
  Gets average gradient from all parameters, then
  updates the control policy.
*/
void gbrl(void){
  gbrl_curr_step = mb_io_get_ul(ID_E_STEP_NO); //get the current step number
  if (gbrl_curr_step - gbrl_start_step < gbrl_max_steps){ //still averaging
    gbrl_average();
  } else { //done averaging
    gbrl_start_step = mb_io_get_ul(ID_E_STEP_NO); //get start step number; reset steps
    if (gbrl_index < 0){ //found the baseline
      gbrl_calc_baseline();
//      gbrl_index = 0; TEST
    } else { //found some gradient
//      gbrl_calc_grad(); //find change from perturbation
//      gbrl_deperturb(); //restore parameter
//      gbrl_index++;
//      if (gbrl_index < GBRL_PARAMS_SIZE){ //still checking params
//        gbrl_perturb();
//      } else { //done averaging all params
//        gbrl_update_policy();
//        gbrl_index = -1;
//      }
    }
  } 
}

/**
  Calculates the effect of the current parameter
  on the optimized values.
*/
void gbrl_average(void){
  float mch_power = mb_io_get_float(ID_MCH_BATT_POWER);
  float mcfo_power = mb_io_get_float(ID_MCFO_BATT_POWER);
  float mcfi_power = mb_io_get_float(ID_MCFI_BATT_POWER);
  float mcsi_power = mb_io_get_float(ID_MCSI_BATT_POWER);
  float total_power = (mch_power + mcfi_power + mcfi_power + mcsi_power);
  float curr_power = total_power / 4.0; //average power across all boards
  gbrl_power_sum += curr_power; //add current power to sum
  gbrl_power_count++; //increase num data points
  
  // ************* TEST CODE *************//
  mb_io_set_float(ID_E_TEST3, curr_power);
  mb_io_set_float(ID_E_TEST4, gbrl_power_count);
  mb_io_set_float(ID_E_TEST5, gbrl_power_sum);
  // *************************************//
}

/**
  Calculates the baseline average output (power
  consumption)
*/
void gbrl_calc_baseline(void){
  gbrl_power_baseline = gbrl_power_sum / gbrl_power_count;
  gbrl_power_sum = 0.0;
  gbrl_power_count = 0;
  // ************* TEST CODE *************//
  mb_io_set_float(ID_E_TEST4, gbrl_power_count);
  mb_io_set_float(ID_E_TEST5, gbrl_power_sum);
  mb_io_set_float(ID_E_TEST6, gbrl_power_baseline);
  // *************************************//
}

/**
  Finds the change in output with the perturbed
  versus the old parameter values.
*/
void gbrl_calc_grad(void){
  float power = gbrl_power_sum / gbrl_power_count;
  float change = power - gbrl_power_baseline;
  float grad = change / gbrl_pert;
  gbrl_grad[gbrl_index] = grad;
}

/**
  Perturb the control parameter given by
  gbrl_index.
*/
void gbrl_perturb(void){
  unsigned short param = gbrl_params[gbrl_index]; //get the current parameter
  gbrl_pert_saved = mb_io_get_float(param); //save the old value before perturbing
  gbrl_pert = gbrl_pert_saved * gbrl_pert_delta; //calculate the amount of perturbation
  mb_io_set_float(param, gbrl_pert_saved + gbrl_pert); //set the current parameter to perturbed
}

/**
  Restores the old value of the control parameter
  before perturbation.
*/
void gbrl_deperturb(void){
  unsigned short param = gbrl_params[gbrl_index]; //get the current parameter
  mb_io_set_float(param, gbrl_pert_saved); //set the current parameter to pre-perturbed value
}

/**
  Updates the control policy to optimize the output.
*/
void gbrl_update_policy(void){
  int i;
  for (i = 0; i < GBRL_PARAMS_SIZE; i++){
    unsigned short param = gbrl_params[i]; //get the parameter
    float grad = gbrl_grad[i]; //get the gradient for that parameter
    float value = mb_io_get_float(param); //get the current value of the parameter
    value = value + grad; //update the value of the parameter
    mb_io_set_float(param, value); //set the parameter
  }
}








