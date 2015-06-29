/**
	Reward Function Estimator
	
	@author Nicolas Champagne-Williamson
	@date October 2010
*/

#include <mb_includes.h>

// ***************************************************************************
// REWARD FUNCTION
// ***************************************************************************

// Cost of Transport Estimation
static float gbrl_cot = 0.0; //last calculated cost of transport

/**
  Returns the output of the reward function.
  Resets averaging values.
*/
float gbrl_calc_reward(void){
  float power = gbrl_calc_power();
  float velocity = gbrl_calc_velocity();
  gbrl_cot = power/velocity; //Cost of Transport
  
  set_io_float(ID_GBRL_COT, gbrl_cot);
  
  
  float stability = gbrl_calc_stability();
  
  set_io_float(ID_GBRL_REWARD, stability);
  
  return velocity;
}

/**
  Averages needed values for calculating the reward function.
*/
void gbrl_average_reward(void){
  gbrl_average_power();
  gbrl_average_velocity();
  gbrl_average_stability();
}

// ***************************************************************************
// POWER CONSUMPTION
// ***************************************************************************

// Variables
static unsigned int gbrl_power_start_time = 0; //start time for piece of integral
static float gbrl_power_sum = 0.0; //sum of power consumption
static float gbrl_power_total_time = 0; //number of data points
static float gbrl_power = 0.0; //last calculated average power consumption

/**
  Returns the average power.
  Resets power averaging values.
*/
float gbrl_calc_power(void){
  gbrl_power = gbrl_power_sum / gbrl_power_total_time; //calculate average power
  gbrl_power_sum = 0.0; //reset sum
  gbrl_power_total_time = 0; //reset counter
  
  set_io_float(ID_GBRL_POWER, gbrl_power);
  
  return gbrl_power;
}

/**
  Continues averaging power.
  avg = integral(A to B, f(x)dx) / (B - A)
*/
void gbrl_average_power(void){
  unsigned int curr_time = mb_get_timestamp();
  float elapsed_time = (curr_time - gbrl_power_start_time) / 1000.0; //elapsed time (dx)
  float mch_power = get_io_float(ID_MCH_BATT_POWER);
  float mcfo_power = get_io_float(ID_MCFO_BATT_POWER);
  float mcfi_power = get_io_float(ID_MCFI_BATT_POWER);
  float mcsi_power = get_io_float(ID_MCSI_BATT_POWER);
  float total_power = (mch_power + mcfi_power + mcfi_power + mcsi_power);// total power across all boards (f(x))
  gbrl_power_sum += total_power * elapsed_time; //add current power to sum (f(x) * dx)
  gbrl_power_total_time += elapsed_time; //increase divisor (B - A)
  gbrl_power_start_time = curr_time;
}

// ***************************************************************************
// VELOCITY
// ***************************************************************************

// Velocity Estimation
static unsigned int gbrl_vel_start_time = 0; //start time for time elapsed
static float gbrl_vel_start_dist = 0.0; //start distance for distance traveled
static float gbrl_velocity = 0.0; //the last calculated average velocity

/**
  Returns the average velocity.
  Resets velocity averaging values.
*/
float gbrl_calc_velocity(void){
  unsigned int curr_time = mb_get_timestamp();
  float elapsed_time = (curr_time - gbrl_vel_start_time) / 1000.0; //find elapsed time
  float curr_dist = get_io_float(ID_E_TOTAL_DISTANCE);
  float elapsed_dist = curr_dist - gbrl_vel_start_dist; //find elapsed dist
  gbrl_velocity = elapsed_dist / elapsed_time; 
  gbrl_vel_start_time = curr_time; //reset time
  gbrl_vel_start_dist = curr_dist; //reset distance
  
  set_io_float(ID_GBRL_VELOCITY, gbrl_velocity);
  
  return gbrl_velocity;
}

/**
  Continues averaging the velocity.
*/
void gbrl_average_velocity(void){
 //No averaging needed
}

// ***************************************************************************
// STABILITY
// ***************************************************************************

// Stability Estimation
static unsigned int gbrl_count = 0; 
static float gbrl_total_stability = 0.0;
static float gbrl_avg_stability = 0.0;
static float gbrl_prev_stability = 0.0;

/**
  Returns the average stability.
*/
float gbrl_calc_stability(void){
  gbrl_avg_stability = gbrl_total_stability / (float) gbrl_count;
  
  gbrl_count = 0;
  gbrl_total_stability = 0.0;
  
  return gbrl_avg_stability;
}

/**
  Continues averaging the stability.
*/
void gbrl_average_stability(void){
  float curr_stability;
  
  gbrls_calc_stability(); //calculate the current stability TODO: use this value later
  
  curr_stability = gbrls_get_stability();
  if (curr_stability != gbrl_prev_stability){ //took another step
    gbrl_count++;
    gbrl_total_stability += curr_stability;
    gbrl_prev_stability = curr_stability;
  }
}



