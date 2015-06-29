/**
	@file stability.cpp
	
	Stability Estimator for Ranger.
	More stable = can take larger perturbations before
	falling forwards or backwards.
	
	Will first try estimating stability with fluctuations
	in velocity per step. Maybe fluctuations in step size as well.
	
  - Seems from experiments that if the step velocity is lower or equal to 0.2 m/s,
  will not make the next step.
  - Also if step time is greater than 0.74 s.
  - Also if step length is less than 0.4 m.
  - Change in velocity wasn't too descriptive so far, but seems to be getting off if
  larger than 0.43 m/s from previous step
  
  
  
  
  
  
	@author Nicolas Champagne-Williamson
	@date Oct 2010
*/

#include <mb_includes.h>

// ********** VARIABLES **********
/**
  The stability measure should approximate how likely Ranger is to falling.
  
  * 0 = Ranger is on the border of falling over. Any perturbation in the wrong 
         direction is certain dooooom!
  
  * <0 = Ranger is falling or has fallen or if it hasn't it will shortly.
  
  * >0 = Ranger is walking stably (as far as we can tell). Greater the value,
          the greater the perturbation Ranger can absorb and not fall over.
*/
static float gbrls_stability = 0.0;

/**
  The safety threshold for Ranger. We don't want the 
  stability measure to go below this value to keep Ranger
  safely walking.
*/
//static float gbrls_threshold = 0.0;

//Heelstrike detection
static unsigned long int gbrls_start_step = 0;
static unsigned long int gbrls_curr_step = 0;

//Step Velocity
static float gbrls_step_velocity = 0.0;
static float gbrls_prev_step_velocity = 0.0;
static float gbrls_delta_step_velocity = 0.0;
static float gbrls_step_velocity_thresh = 0.2; // vel > 0.2

//Step Time
static float gbrls_step_time = 0.0;
static float gbrls_prev_step_time = 0.0;
static float gbrls_delta_step_time = 0.0;
static float gbrls_step_time_thresh = 0.74; //time < 0.74

//Step Length
static float gbrls_start_dist = 0.0;
static float gbrls_curr_dist = 0.0;
static float gbrls_step_length = 0.0;
static float gbrls_prev_step_length = 0.0;
static float gbrls_delta_step_length = 0.0;
static float gbrls_step_length_thresh = 0.4; //length < 0.4


// ********** FUNCTIONS ************

/**
  Returns the current stability of the robot.
  Call whenever, but make sure to call calc_stability every tick
*/
float gbrls_get_stability(void){
  return gbrls_stability;
}

/**
  This function should be called every tick.
*/
void gbrls_calc_stability(void){
  if (gbrls_heelstrike()) { //heelstrike, calculate stuff
    gbrls_calc_step_length();
    gbrls_calc_step_time();
    gbrls_calc_step_velocity();
    
    //Calc deviations from threshold - positive = safe, negative = BAD
//    float time_dev = gbrls_step_time_thresh - gbrls_step_time; //want time < thresh
//    float length_dev = gbrls_step_length_thresh - gbrls_step_length; //want length < thresh
//    float vel_dev = gbrls_step_velocity - gbrls_step_velocity_thresh; //want vel > thresh
//    float sign = (time_dev < 0 || length_dev < 0 || vel_dev < 0 ? -1.0 : 1.0);
//    gbrls_stability = sign * abs(time_dev * vel_dev * length_dev); //function of step length, step time, and step velocity

    float length_delta = abs(gbrls_delta_step_length / gbrls_prev_step_length);
    float time_delta = abs(gbrls_delta_step_time / gbrls_prev_step_time);
    float vel_delta = abs(gbrls_delta_step_velocity / gbrls_prev_step_velocity);
    float total_delta = length_delta + time_delta + vel_delta;
    gbrls_stability = -total_delta;
    
    set_io_float(ID_GBRL_STABILITY, gbrls_stability);
  }
}

/**
  Returns true (1) if step number has increased,
  indicating a hs.
*/
unsigned int gbrls_heelstrike(void){
  gbrls_curr_step = get_io_float(ID_E_STEP_NO); //get the current step number
  if (gbrls_curr_step - gbrls_start_step > 0) { //heelstrike!
    gbrls_start_step = gbrls_curr_step;
    return 1;
  } else { //still on current step
    return 0;
  }
}

/**
  Calculates the average velocity of the previous step.
  Call every heelstrike.
*/
void gbrls_calc_step_velocity(void){
  //Calc step velocity
  gbrls_step_velocity = gbrls_step_length / gbrls_step_time;
  
  //Calc changefrom prev step velocity
  gbrls_delta_step_velocity = abs(gbrls_step_velocity - gbrls_prev_step_velocity);
  gbrls_prev_step_velocity = gbrls_step_velocity;
  
  set_io_float(ID_GBRL_STEP_VELOCITY, gbrls_step_velocity);
}

/**
  Calculates the previous step length.
  Call every heelstrike.
*/
void gbrls_calc_step_length(void){
  //Calc step length
  gbrls_curr_dist = get_io_float(ID_E_TOTAL_DISTANCE);
  gbrls_step_length = gbrls_curr_dist - gbrls_start_dist;
  gbrls_start_dist = gbrls_curr_dist;
  
  //Calc change from prev step length
  gbrls_delta_step_length = abs(gbrls_step_length - gbrls_prev_step_length);
  gbrls_prev_step_length = gbrls_step_length;
  
  set_io_float(ID_GBRL_STEP_LENGTH, gbrls_step_length);
}

/**
  Calculates the previous step time.
  Call every heelstrike.
*/
void gbrls_calc_step_time(void){
  //Calc step time
  gbrls_step_time = get_io_float(ID_E_STEP_TIME)/1000.0; //get time in sec from ms
  
  //Calc change from prev step time
  gbrls_delta_step_time = abs(gbrls_step_time - gbrls_prev_step_time);
  gbrls_prev_step_time = gbrls_step_time;
  
  set_io_float(ID_GBRL_STEP_TIME, gbrls_step_time);
}

static float abs(float val){return (val < 0 ? -val : val);}



