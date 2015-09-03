/**
  @file motor_controller.c
 
  Motor controlling functions for use on the B2A motor controller boards.
  
  Example Hardware and Register setup for PWM control of motors:
  @code
  // *******************************************************************************
  // Motor Controller Setup
  // *******************************************************************************
  //Motor control initialization
  //FIO0DIR |= (1<<9);   	// set P0.9 to be output (PWM A)
  //FIO0DIR |= (1<<7);     // set P0.7 to be output (PWM B)
  FIO0DIR |= (1<<11);     // set P0.11 to be output (Low side enable A)
  FIO1DIR |= (1<<17);     // set P1.17 to be output (Low side enable B)
  FIO0DIR |= (1<<6);      // set P0.6 to be output (Watchdog timer) 
  //FIO0CLR = (1<<9);  	// turn off PWM A output
  //FIO0CLR = (1<<7);  	// turn off PWM B output
  FIO0SET = (1<<11);  	// turn on low side A enable output
  FIO1SET = (1<<17);  	// turn on low side B enable output
  FIO0CLR = (1<<6);  	// turn off watchdog timer output
  //Initializes the PWM registers and PWM2 and PWM6 outputs
  //Pin Function Select
  PINSEL0 &= ~(3<<14);
  PINSEL0 |= (2<<14);  //PWM 2 on Pin P0.7 Enabled (bits 14/15)
  PINSEL0 &= ~(3<<18);
  PINSEL0 |= (2<<18);  //PWM 6 on Pin P0.9 Enabled (bits 18/19)
  //PWM Prescale Register. Controls number of PClK cycles per Timer Counter Cycle
  PWMPR= 0; //Will run at maximum rate
  //PWM Match Registers
  //PWM Match Register 0. Resets Timer Counter and sets all PWM outputs to 1.
  PWMMR0= 600; //100 kHz PWM frequency (60000000/600 = 100,000)
  PWMMR1= 0;
  PWMMR2= 0;
  PWMMR3= 0;
  PWMMR4= 0;
  PWMMR5= 0;
  PWMMR6= 0;
  //PWM Match Control Register
  PWMMCR= (PWMMCR|0x2) & ~PWMMCR_RB; //Should set Timer Counter to reset upon reaching Match Register 0
  //PWM Timer Control Register.
  PWMTCR= (PWMTCR|0x1) // Enable Timer Counter
  		| (PWMTCR|0x8) // Enable PWM Mode
  		& ~PWMTCR_RB; 
               //Must Occur after Initialization of Match Register 0.
  //PWM Control Register. Enables individual PWM types and outputs.
  PWMPCR= (PWMPCR|0x4400) & ~PWMPCR_RB; //Should mean the same thing as next 4 lines (PWM Control Register)
  //PWMSEL2= 0; //Enables Single Edge PWM 2 Control
  //PWMSEL6= 0; //Enables Single Edge PWM 6 Control
  //PWMENA2= 1; //PWM 2 Output Enabled
  //PWMENA6= 1; //PWM 6 Output Enabled
  //PWM Latch Enable Register. Updates Match values.
  //Each bit directly relates to a Match Register
  //PWMLER= 0;  //Latch Register to zero
  PWMMR2= 0;       // out of 600  -ve  	
  PWMMR6= 0;       // out of 600  +ve  	  	 
  // Latch enable reg automatically cleared once PWM match values are set
  PWMLER= (PWMLER|0x45) & ~PWMLER_RB;  //Enable update of PWM match registers 0, 2 and 6.  0x45 --> bit0, bit2, bit6
  @endcode
  
  @author Nicolas Williamson 
  @date 5/18/09.

 */

#include <includes.h>

//Variables
static volatile signed long int mc_pwm = 0; //the current pwm of the controller
static volatile fixed mc_target_current = 0; //the target current of the pid current controller
//static int mc_target_position = 0; // the target position of the pid position controller
static volatile fixed mc_c1 = 0;
static volatile fixed mc_new_c1 = 0;
static volatile fixed mc_c2 = 0;
static volatile fixed mc_new_c2 = 0;
static volatile fixed mc_command_current = 0;

/* State Variables */
static volatile unsigned long int mc_therm_off = 0; //whether the motor was turned off due to the thermal limit
static volatile unsigned long int mc_mech_off = 0; //whether the motor was turned off due to the mechanical limit
static volatile unsigned long int mc_off = 0; //whether the motor controller is off (1) or not (0)
static volatile unsigned long int mc_sleep = 0; //whether the motor controller is sleeping (1) or not (0)
static volatile unsigned long int mc_shutdown = 0; //whether the motor controller is shutdown (1) or not (0)
static volatile unsigned long int mc_direction = 0;

static MC_DATA mc_data; //parameters of the motor controller
static long int mc_positive = 1; //whether the motor is allowed to turn in the positive direction
static long int mc_negative = 1; //whether the motor is allowed to turn in the negative direction
static unsigned long int mc_phase; //what phase we are in (x:0 or 1:1-x)

static float mc_save_stiff;
static float mc_save_damp;
static float mc_save_comm;

/**
  Initializes the motor controller. Must be called before the motor
  controller module can be used properly.
  @param max_volts The maximum voltage the motor can take.
  @param max_current The maximum allowed current for the motor.
  @param min_current The minimum allowed current for the motor, often just -1 * @c max_current.
  @param max_target The maximum allowed current to be commanded using @ref mc_set_target_current.
  @param min_target The minimum allowed current to be commanded using @ref mc_set_target_current.
  @param thermal_current_limit The nominal or max continuous current for the motor.
  @param thermal_time_constant The Winding Thermal Time Constant.
  @param kp Proportional constant for the PID current controller.
  @param ki Integral constant for the PID current controller.
  @param kd Derivative constant for the PID current controller.
  @param max_pwm Maximum allowed PWM for the motor, often to limit voltage.
  @param error_limit If there are this many errors in a row, shutdown the motor. (DEPRECATED)
  @param smart_error_limit If the integral of the error is greater than this value, shutdown the motor.
  @param op The operation of the motor, either @c MC_NORMAL or @c MC_BACKWARDS. Used if positive PWM and positive position don't match.
  @param current A function that returns the motor current as a fixed point value in amps.
  @param position A function that returns the motor position as a floating point number in radians.
  @param velocity A function that returns the motor velocity as a floating point number in radians.
*/
void mc_init(
    float max_volts, 
		float max_current, 
		float min_current,
    float max_target, // dont see this in soft_setup_init
    float min_target,  // dont see this in soft_setup_init
    float thermal_current_limit, //from spec sheet
    float thermal_time_constant, //tau, from spec sheet
		float kp, 
		float ki, 
		float kd, 
		int max_pwm, 
		int error_limit, // set 4 now
    float smart_error_limit, // set 10 now
    MC_OPERATION op,        // I dont see this in soft_setup_init
		FIXED_VOID_F current, 
		FLOAT_VOID_F position,
		FLOAT_VOID_F velocity) //initializes the motor controller
{
  mc_data.operation = op;
  
	mc_data.motor_voltage_max = max_volts;	//Volts	
	mc_data.current_max = float_to_fixed(max_current);	//Amps
	mc_data.current_min = float_to_fixed(min_current);	//Amps
  mc_data.target_max = float_to_fixed(max_target);
  mc_data.target_min = float_to_fixed(min_target);
  
  mc_data.thermal_limit = float_to_fixed((thermal_current_limit*thermal_current_limit)*thermal_time_constant);
  mc_data.tau_inv = float_to_fixed(1.0/thermal_time_constant);
  mc_data.thermal_safe = fixed_mult(float_to_fixed(0.75), mc_data.thermal_limit);
  mc_data.thermal_diff = float_to_fixed(1.0/(thermal_current_limit - fixed_to_float(mc_data.thermal_safe)));
  mc_data.dt = float_to_fixed(1.0/(float)(1000*SCHED_SPEED));
  
	mc_data.kp = float_to_fixed(kp);
	mc_data.ki = float_to_fixed(ki);
	mc_data.kd = float_to_fixed(kd);	
	mc_data.integral = 0;
	mc_data.max_pwm = max_pwm;
	mc_data.max_integral = float_to_fixed((((float)max_pwm)/ki));
	mc_data.error_limit  = error_limit;
  mc_data.smart_error_limit = float_to_fixed(smart_error_limit);
  mc_data.smart_error_limit_inverse = float_to_fixed(1.0*10.0/smart_error_limit); 
	mc_data.get_current = current;
	mc_data.get_position = position;
	mc_data.get_velocity = velocity;
}

/**
  Sets the target current of the PID current controller to @c new_current.
  The current controller will attempt to achieve this current on the next call to @ref mc_pid_current.
  @param new_current The target current for the PID current controller. @c target_min &lt @c new_current &lt @c target_max.
*/
void mc_set_target_current(float new_current) 
{
	fixed curr = float_to_fixed(new_current);
	mc_set_target_current_fixed(curr);
}

/**
  Sets the target current of the PID current controller to @c current as a fixed point value.
  The current controller will attempt to achieve this current on the next call to @ref mc_pid_current.
  This function is faster than @ref mc_set_target_current, and its use is preferred. 
  @param current The target current in fixed point.
*/
static void mc_set_target_current_fixed(fixed current){
//  fixed curr = mc_data.operation * current;
  fixed curr = current;
  if (curr < mc_data.target_min){
  	error_occurred(ERROR_MC_TCURR_OOB);
    curr = mc_data.target_min;
  }
  if (curr > mc_data.target_max){
  	error_occurred(ERROR_MC_TCURR_OOB);
    curr = mc_data.target_max;
	}
  mc_target_current = curr;
}

/**
  Sets the target current to @c new_current without any limiting.
  @param new_current The new unlimited target current.
*/
void mc_set_unsafe_target_current(float new_current) 
{
	fixed curr = float_to_fixed(new_current);
	mc_target_current = /*mc_data.operation * */curr;
}
  
/**
  A compliant controller for the motor. 
  Similar to a PD-position controller. To use, the user should call @ref mc_set_stiffness,
  @ref mc_set_dampness, and @ref mc_set_command_current to set the parameters of the controller.
  A call to @ref mc_set_command_current will latch the new stiffness and dampness values; without this
  call, dampness and stiffness (and the command current) won't change. The formula used in the
  control is @code Icontrol = Icommand - (stiffness * pos) - (dampness * vel) @endcode.
  Icontrol is passed to the PID current controller, which will attempt to go to the desired current.
  To request a specific position (in radians) with a given stiffness, use: @code Icommand = stiffness * desired_position @endcode.
*/
void mc_compliant_control(void){
  float pos_rads = mc_data.get_position();
  fixed pos = float_to_fixed(pos_rads);
  fixed vel = float_to_fixed(mc_data.get_velocity());
  fixed control = mc_command_current - fixed_mult(mc_c1, pos) - fixed_mult(mc_c2, vel);
  mc_set_target_current_fixed(control);
  mc_pid_current();
}

/**
  Sets the command current used by @ref mc_compliant_control to @c new_command.
  Also latches new values for stiffness and dampness.
  @param new_command The command current for the compliant controller.
*/
void mc_set_command_current(float new_command)
{
  mc_command_current = float_to_fixed(new_command);
  mc_save_comm = new_command;
  //if (new_command != 7.2){mcu_led_red_blink(10);}
  mc_c1 = mc_new_c1;
  mc_c2 = mc_new_c2;
}
float mc_get_command_current(void){
  return mc_save_comm;
}

/**
  Sets the stiffness used by @ref mc_compliant_control to @c new_c1.
  This value will not be used (latched) until @ref mc_set_command_current has been called.
  @param new_c1 The new stiffness for the compliant controller.
*/
void mc_set_stiffness(float new_c1){
  mc_new_c1 = float_to_fixed(new_c1); 
  mc_save_stiff = new_c1;
  //if (new_c1 != 4.0){mcu_led_red_blink(10);}
}
float mc_get_stiffness(void){
  return mc_save_stiff;
}

/**
  Sets the dampness used by @ref mc_compliant_control to @c new_c2.
  This value will not be used (latched) until @ref mc_set_command_current has been called.
  @param new_c2 The new dampness for the compliant controller.
*/
void mc_set_dampness(float new_c2){
  mc_new_c2 = float_to_fixed(new_c2); 
  mc_save_damp = new_c2;
  //if (new_c2-0.2 > 0.01 || new_c2-0.2 < -0.01){mcu_led_red_blink(10);}else{mcu_led_green_blink(10);}
}
float mc_get_dampness(void){
  return mc_save_damp;
}

/**
  Controls the motor in a given direction.
  The direction, either MC_POSITIVE or MC_NEGATIVE, is defined as the
  direction a target current (Not pwm!) with that sign causes the motor to turn. 
  The command is either for the motor to stop running in that
  direction or to continue moving in that direction. If the motor is stopped in a given
  direction, it may continue to move in the opposite direction, provided that direction hasn't
  also been stopped. Note that the directions may be controlled independently of one another.
  @param direction The direction, either MC_POSITIVE or MC_NEGATIVE, in which to control the motor.
  @param command The command, either MC_STOP or MC_START.
*/
void mc_direction_control(MOTOR_DIRECTION direction, MOTOR_CONTROL command) 
{
  int dir = direction/* * mc_data.operation*/; //if running backwards, opposite direction
	if (dir == MC_POSITIVE){
		mc_positive = command;
	} else if (dir == MC_NEGATIVE){
		mc_negative = command;
	}
}

/**
  Returns whether the motor is running or not.
  @return The status of the motor.
    - @b 0: if the motor is off, asleep, or shutdown
    - @b 1: otherwise
  @private
*/
static int mc_is_running(void){
  int running = (!mc_off) && (!mc_sleep) && (!mc_shutdown) && (!mc_therm_off) && (!mc_mech_off);
  if (!running)error_occurred(ERROR_MC_NOT_RUNNING);
  return running;
}

/**
  Gets the parameters currently being used by the motor controller.
  This may be used to read or change the parameters values.
  @return A pointer to the @ref MC_DATA struct used to hold most of the 
  motor controller's parameters.
  @deprecated
*/
MC_DATA* mc_get_parameters(void) 
{
	return &mc_data;
}	

/**
  Gets the current PWM being sent to the motor.
  @return The current PWM.
*/
int mc_get_pwm(void) 
{
	return mc_pwm;
}

/**
  Updates the motor controllers watchdog timer.
  This needs to be called frequently, otherwise the motor controller will shut itself off. 
  It is automatically called from @ref mc_pid_current, @ref mc_run_no_control, and @ref mc_set_pwm,
  so the user shouldn't have to explicitly call this unless they don't use
  any of those functions. 
  The watchdog will not update if the motor controller is shutdown or off.
*/
void mc_update_watchdog(void) 
{
  if (!mc_shutdown && !mc_off && !mc_therm_off && !mc_mech_off){
  	if(FIO0PIN & (1<<6)){
  		FIO0CLR = (1<<6); //make watchdog low
  	}
  	else {
  		FIO0SET = (1<<6); //set watchdog hi
  	}
  }
}

/**
  Sets the shutdown state of the motor.
  If the motor is shutdown, any calls to @ref mc_set_pwm will be forced to 0,
  @ref mc_pid_current will not calculate PWMs, and @ref mc_update_watchdog will NOT update
  the watchdog, causing it to starve and shut off the motor controller in hardware.
  @param shutdown If this value is 0, the motor is not shutdown, else shutdown the motor.
*/
void mc_set_shutdown(int shutdown){
  unsigned long sd = ((unsigned long)shutdown) & 0x7FFF;
  mc_shutdown = sd;
}

/**
  Sets the sleep state of the motor.
  If the motor is asleep, it will continue to update the watchdog, but will not run the pid
  current controller and any attempt to use @ref mc_set_pwm will be forced to 0.
  @param sleep 0 = Awake, not 0 = Asleep.
*/
void mc_set_sleep(int sleep){
  unsigned long sl = ((unsigned long)sleep) & 0x7FFF;
  mc_sleep = sl;
}

/**
  Turns off the motor controller until it is safe to turn back on.
  Sets the appropriate state and causes an error. Sets the PWM to 0 and
  lets the watchdog timer starve out. Also turns on the green LED.
  @param reason Why this function was called.
  @private
*/
static void mc_turnoff(MC_SAFETY_CAUSE reason){
  switch(reason){ //set appropriate state and cause error
    case MC_THERM: mc_therm_off = 1;
      error_occurred(ERROR_MC_TEMP_OFF);
      break;
    case MC_MECH: mc_mech_off = 1;
      error_occurred(ERROR_MC_MECH_OFF);
      break;
    default: mc_off = 1;
      error_occurred(ERROR_MC_SHUTOFF);
      break;
  }
  
  mc_pwm = 0;
  PWMMR2 = 0;		//Both PWMs set to 0
	PWMMR6 = 0;
	PWMLER = (1<<6)|(1<<2);
  
  mcu_led_green_on();
}

/**
  Turns on the motor controller once it is safe to turn back on.
  Sets the appropriate state and causes an error. Starts feeding the
  watchdog timer again. Also turns off the green LED.
  @param reason Why this function was called.
  @private
*/
static void mc_turnon(MC_SAFETY_CAUSE reason){
  switch(reason){ //set appropriate state and cause error
    case MC_THERM: mc_therm_off = 0;
      error_occurred(ERROR_MC_TEMP_ON);
      break;
    case MC_MECH: mc_mech_off = 0;
      error_occurred(ERROR_MC_MECH_ON);
      break;
    default: //shouldn't get here
      break;
  }
  
  mcu_led_green_off();
}

/**
  Sets the PWM signal going to the motor to @c pwm.
  @param pwm The PWM going to the motor. @code -1*max_pwm &lt pwm &lt max_pwm @endcode
*/
void mc_set_pwm(int pwm)
{
  const unsigned long int full = PWMMR0;
  const unsigned long int offset = 14;

	mc_update_watchdog();
  
  
  if (!mc_is_running()){ //controller is asleep, shutdown, or off
    pwm = 0;
  } 
  
  //Shut off all H-bridge switches when in sleep mode
  if (mc_sleep)
  {
    PWMMR2 = 0;  			//Set duty cycle of PWM2 (B) to 0
  	PWMMR6 = 0;				//Set duty cycle of PWM6 (A) to 0
  	PWMLER = (1<<2)|(1<<6);  	//Transfer new PWMMR2 & PWMMR6 values to reg at next PWM match
    
    FIO0CLR = (1<<11);  	// turn off low side A enable output
    FIO1CLR = (1<<17);  	// turn off low side B enable output
  }
  else  //Set PWM normally and enable low-side switches
  {
  
    FIO0SET = (1<<11);  	// turn on low side A enable output
    FIO1SET = (1<<17);  	// turn on low side B enable output
    
    pwm = mc_data.operation * pwm;
    
	  mc_pwm = pwm;
  
	  //check bounds
	  if (pwm > mc_data.max_pwm)
    {
      pwm = mc_data.max_pwm;
      error_occurred(ERROR_MC_PWM_LIMIT);
    }
	  else if (pwm < -1*mc_data.max_pwm)
    {
      pwm = -1*mc_data.max_pwm;
      error_occurred(ERROR_MC_PWM_LIMIT);
    }
	
    if (mc_phase == 0)
    { //Normal operation: PWMA = x%, PWMB = 0%
  	  if(pwm < 0) 
      { //Negative current, clockwise  
  		  PWMMR2 = 0;  			//Set duty cycle of PWM2 to 0 so that only one motor direction is on
  		  PWMMR6 = -1 * pwm;				//Set duty cycle of PWM6 (out of 600 max)
  		  PWMLER = (1<<2)|(1<<6);  	//Transfer new PWMMR2 & PWMMR6 values to reg at next PWM match
  	  }
  	  else 
      { //Positive current, counter-clockwise
  		  PWMMR2 = pwm;	 //Set duty cycle of PWM2 (out of 600 max)	
  		  PWMMR6 = 0;   //Set duty cycle of PWM6 to 0 so that only one motor direction is on		
  		  PWMLER = (1<<2)|(1<<6);  	 	//Transfer new PWMMR2 & PWMMR6 values to reg at next PWM match
  	  }
      mc_phase = 1;
    } 
    else 
    { //Alternate Operation: PWMA = 100%, PWMB = 100-x%
  
      if(pwm < 0) {//negative current, clockwise
        PWMMR2 = full - (pwm * -1) - offset;
        PWMMR6 = full;
        PWMLER = (1<<2)|(1<<6);
      } else { //positive current, counter-clockwise
        PWMMR2 = full;
        PWMMR6 = full - (pwm) - offset;
        PWMLER = (1<<2)|(1<<6);
      }
    
      mc_phase = 0;
    }  
  }  
}

static float mc_mult;
/** Returns the current multiplier for either thermal or mech current limiting. */
float mc_get_mult(void){ return mc_mult;}
/**
  Uses a PID controller to try and reach the target current.
  The target currect used by this function should be set using
  @ref mc_set_target_current or @ref mc_set_target_current_fixed.
  The constants for the current controller are given by the @c kp, @c ki, and @c kd
  given to @ref mc_init.
  Updates the watchdog, and uses both a thermal limiting multiplier and integrating
  current limits.
*/
void mc_pid_current(void) 
{
	volatile fixed mc_error = 0;	
	static unsigned char start_count = 2;
	signed long int curr_pwm = 0;
	long long int pid_sum, p_component, i_component;
  fixed thermal_multiplier, mechanical_multiplier;
  
  mc_update_watchdog(); //feed watchdog if running or sleeping

  if (!start_count){ //wait a few iterations to allow the adcx to get meaningful data
    
    fixed current = mc_data.get_current(); //motor current on the robot
    fixed target_current = mc_target_current; //the target current      
    thermal_multiplier = mc_get_thermal_limiter(current); //thermal multiplier, est. temp and limit current
    mechanical_multiplier = mc_smart_check_current(current);
    
    //During direction control, we limit the target_current to the correct direction.
    //Used to do PWM, but ran into problem with Integral control
    if (target_current < 0) { //trying to go in negative direction
      target_current = target_current * mc_negative;
    } else if (target_current > 0) { //trying to go in positive direction, but not allowed
      target_current = target_current * mc_positive;
    }
	
   	if (mc_is_running())
    {
			// ******************************* calc prop and integral **************************
      //Calculate the error:
			mc_error = (target_current - current);
      
      //Integral saturation during direction control is fixed by limiting the 
      //target_current during direction_control, not the PWM
			mc_data.integral += mc_error;
      
			// ****************** check for saturation of integral constant ********************
			if (mc_data.integral > mc_data.max_integral){
				mc_data.integral = mc_data.max_integral; 
        error_occurred(ERROR_MC_INTEG_SATUR);
			} else if (mc_data.integral < -1*mc_data.max_integral){
				mc_data.integral = -1*mc_data.max_integral;
        error_occurred(ERROR_MC_INTEG_SATUR);
			}
      
 			// Sum the terms of the PID algorithm to give the new PWM duty cycle value
			//Note: Need to bound each component such that their sum can't overflow
			p_component = fixed_mult_to_long((mc_error),(mc_data.kp));
			i_component = fixed_mult_to_long((mc_data.integral),(mc_data.ki));
			pid_sum = p_component + i_component;
		
			// check for fixed point overflow
			if(pid_sum > FIXED_MAX) {
				pid_sum = FIXED_MAX;
        error_occurred(ERROR_MC_FIXED_LIMIT);
			} else if (pid_sum < -1*FIXED_MAX) {
				pid_sum = -1*FIXED_MAX;
        error_occurred(ERROR_MC_FIXED_LIMIT);
			}
			
      if(thermal_multiplier < mechanical_multiplier){ //use whichever multiplier is smaller
        pid_sum = fixed_mult(pid_sum, thermal_multiplier);
      } else {
        pid_sum = fixed_mult(pid_sum, mechanical_multiplier);
      }
      
			curr_pwm = fixed_to_int((pid_sum));
					
			// ********************** check if PWM is within limits ****************************
			if (curr_pwm > mc_data.max_pwm){
				curr_pwm = mc_data.max_pwm;
				error_occurred(ERROR_MC_PWM_LIMIT);
			}
			else if (curr_pwm < -1*mc_data.max_pwm){
				curr_pwm = -1*mc_data.max_pwm;
				error_occurred(ERROR_MC_PWM_LIMIT);
			}
					
			mc_set_pwm(curr_pwm);	
			
		} else {
      mc_set_pwm(0);
    }
  } else {
		start_count--;
	} 
}

/**
  Runs the motor with no control.
  User should set the PWM explicitly with @ref mc_set_pwm.
  Updates the watchdog timer and checks the current limits.
  @note Should only be used in debugging.
*/
void mc_run_no_control(void)
{
	if (mc_is_running()){
		mc_update_watchdog();
//		mc_check_current(mc_data.get_current());
	}
}

/**
  Checks the motor current for limits.
  If the motor current is outside of the @c current_limits, this returns a multiplier which 
  will be multiplied by the target current to slowly decrease its value.
  Also integrates the motor current error (the amount the motor current is above the @c current_limit),
  and if this value is greater than @c smart_error_limit, calls @ref mc_turnoff to completely turn off the motor,
  until it is safely back within bounds.
  @param current The newest current to add to the smart current limiting.
  @return A multiplier to limit the current if it is out of bounds.
*/
static fixed mc_smart_check_current(fixed current){
  //Check max current for mechanical shutoff
  //Must change error_limit to around 20.
  static fixed current_error = 0;
  fixed motor_current = current;
  static fixed multiplier = FIXED_ONE;
  
  // we'll set the limit to 4.5 amps.. if it is below these limits then the things are fine.
  // as soon as we try to go above this limit a multiplier less than one will be generated.  
  // ****** Integrate Error ****** //
  if (motor_current > 0){
    current_error = current_error + (motor_current - mc_data.current_max);
  } else if (motor_current < 0){
    current_error = current_error - (motor_current - mc_data.current_min);
  }
  if (current_error <= 0){  
    current_error = 0;
    if (mc_mech_off){ //previously turned off due to mechanical limit
      mc_turnon(MC_MECH); //integrator <= 0 so safe to turn back on
    }
  }

  // ****** Calculate multiplier ****** //
  multiplier = FIXED_ONE - (fixed_mult(current_error , mc_data.smart_error_limit_inverse));
  if (multiplier < 0){multiplier = 0;}
  mc_mult = fixed_to_float(multiplier);
  // smart_erroe_limit_inverse is not exactly 1/smart_error_limit, which would mean multiplier goes from one to zero when error goes from 0 to error limit
  // instead I wanted the multiplier to decay faster becasue its taking time to respond/ or the error is growing very fast.
  // so I am settign it to be like 1/.1/smart_error_limit so that it goes to zero quicker and then remains zero
  
  // ****** Check absolute limit ****** //
  if (current_error >= mc_data.smart_error_limit){ //set limit to ~10, can be changed.
    mc_turnoff(MC_MECH);
  }
  //this is so that when curren go to dangerous level of 8 
  //then the error 8-4.5 = 3.5 will have about three tic tocs to reach to 10 and stops.
  //3.3 ms is the mechanichal time constant.  
        
  return multiplier;
                     
}

/**
  Calculates a thermal limiting multiplier for use by @ref mc_pid_current.
  Using the @c thermal_current_limit and @c thermal_time_constant passed to @ref mc_init,
  this function will estimate the current temperature of the motor and find a multiplier
  that when multiplied by the current will limit the temperature. If the motor is too hot,
  this function will call @ref mc_turnoff to completely shutdown the motor until it cools down.
  @warning This function uses the motor current and Newton's Law of Cooling to estimate the temperature, 
  it doesn't measure the motor temperature directly, so care should be taken to let the motors cool down when
  the robot is turned off and on, as the residual heat of the motors will not be taken into account.
  @param current The newest motor current to add to the thermal checking.
*/
static fixed mc_get_thermal_limiter(fixed current){
  //Check max current for thermal shutoff
  //Must create current_factor, thermal_safe, thermal_limit
  
  static fixed multiplier = FIXED_ONE;
  static fixed a = 0; // a is  C/R*(temperature - environmental temperature), C is heat capacity of motor, R is electric resitance which causes the heating I^2*R
  // DANGEREROUS TO INITIALIZE A to zero, eg after lettting the robo on for a while, then switching off and then turning on again
  
  fixed a_dot_dt;  //derivative of 'a' multiplied by time step dt
  fixed motor_current = current;
  a_dot_dt = fixed_mult((fixed_mult(motor_current, motor_current)-fixed_mult(mc_data.tau_inv, a)), mc_data.dt); // derivative is -(1/tau)*a + I^2 , this is newton's law of cooling
  a = a + a_dot_dt;   //  
 
  
  if (a < mc_data.thermal_safe){ //below limit, full current
    multiplier = FIXED_ONE;
    if (mc_therm_off){ //previously turned off by thermal limit, but safe again
      mc_turnon(MC_THERM); //turn on controller
    }
  } else if (a >= mc_data.thermal_safe && a  < mc_data.thermal_limit){ //above safety limit, use mult factor
    multiplier = FIXED_ONE - (fixed_mult(a - mc_data.thermal_safe, mc_data.thermal_diff));  //mc.thermal_diff = float_to_fixed(1.0/(thermal_limit - thermal_safe))
    error_occurred(ERROR_MC_TEMP_SAFE);   
  } else if (a >= mc_data.thermal_limit){ //above strict limit, shutdown
    mc_turnoff(MC_THERM); //turn off controller due to thermal limit
    multiplier = 0;
  } else {
    multiplier = FIXED_ONE; //should never reach here
  }
    
  return multiplier;
 
} 
    
  
