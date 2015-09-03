/*
  @file motor_controller.h
*/

#ifndef __MOTOR_CONTROLLER_H__
#define __MOTOR_CONTROLLER_H__

/**
  The possible directions of the motor used by @ref mc_direction_control.
*/
typedef enum mc_directions{
  MC_POSITIVE = 1, /**< The direction a positive PWM turns the motor. */
  MC_NEGATIVE = -1  /**< The direction a negative PWM turns the motor. */
} MOTOR_DIRECTION;
/**
  Commands to the motor used by @ref mc_direction_control.
*/
typedef enum mc_controls{
  MC_STOP = 0, /**< The motor is stopped and cannot apply PWM in the given direction. */
  MC_START = 1 /**< The motor is running freely and can apply PWM in the given direction. */
} MOTOR_CONTROL;
/**
  The operation of the motor, either normal or backwards, for cases where
  PWM direction does not match up with position and/or velocity direction.
*/
typedef enum mc_direction_multipliers {
  MC_NORMAL = 1, /**< PWM and position direction are the same. */
  MC_BACKWARDS = -1 /**< PWM and position direction are backwards (positive PWM moves the motor in the negative direction). */
} MC_OPERATION;

/** 
  The cause of some safety limit of the motor controller.
  For use in @ref mc_turnoff and @ref mc_turnon.
*/
typedef enum mc_safety_causes{
  MC_NONE = 0,
  MC_THERM = 1, /**< Safety issue caused by thermal limiter. */
  MC_MECH = 2 /**< Safety issue cause by mechanical (current) limiter. */
} MC_SAFETY_CAUSE;

/**
  Parameters used by the motor controller.
  Initialized and set up by @ref mc_init. Can be changed at runtime
  with @ref mc_get_parameters (note that most are in fixed point value though).
*/
typedef struct mc_data{
  float motor_voltage_max; /**< The maximum voltage to the motor. */
  fixed current_max; /**< The maximum current to the motor. */
  fixed current_min; /**< The minimum current to the motor (often -1 * @c current_max). */
  fixed target_max; /**< Maximum current commanded using @ref mc_set_target_current or @ref mc_set_target_current_fixed. */
  fixed target_min; /**< Minimum current commanded using @ref mc_set_target_current or @ref mc_set_target_current_fixed. */
  fixed thermal_safe; /**< Temperature above which current will be multiplied by a thermal limiting factor < 1. */
  fixed thermal_limit; /**< Temperature limit for the motor. If motor_temp > thermal limit, motor will be shutoff. */
  fixed tau_inv; /**< @code 1.0/thermal_time_constant @endcode of the motor. */
  fixed thermal_diff; /**< @code 1.0/(thermal_limit - thermal_safe) @endcode */
  fixed dt; /**< Time in seconds between calls to @ref mc_get_thermal_limiter. @code 1.0 / (SCHED_SPEED * 1000) @endcode. */
  unsigned int error_limit; /**< Maximum number of current out of bounds errors allowed before error shutoff. */
  fixed smart_error_limit; /**< Maximum integral of errors allowed before error shutoff. */
  fixed smart_error_limit_inverse; /**< I need to divide by this number so I inverted it to multiply instead */
  fixed kp; /**< The proportional constant of the PID current controller. */
  fixed ki; /**< The integral constant of the PID current controller. */
  fixed kd; /**< The derivative constant of the PID current controller. */
  fixed integral; /**< The integral value of the PID current controller. */
  fixed max_integral; /**< Maximum integral value allowed by PID current controller, to avoid over-saturation. */
  int max_pwm; /**< Maximum PWM allowed to go to the motor. */
  MC_OPERATION operation; /**< The operation of the motor controller. @see MC_OPERATION. */
  FIXED_VOID_F get_current; /**< A function that returns the motor current as a fixed point value in amps. */
  FLOAT_VOID_F get_position; /**< A function that returns the motor position as a floating point number in radians. */
  FLOAT_VOID_F get_velocity; /**< A function that returns the motor velocity as a floating point number in radians. */
} MC_DATA;



// *********************** Function Declarations ************************
// Public Functions

//initializes the motor controller
void mc_init(float max_volts, //the maximum voltage the mc can take
  float max_current, //max current that can safely be put through the motor
  float min_current, //ditto, but in negative direction
  float max_target,
  float min_target,
  float thermal_current_limit,
  float thermal_time_constant,
  float kp,
  float ki,
  float kd,
  int max_pwm,
  int error_limit,
  float smart_error_limit,
  MC_OPERATION op,
  FIXED_VOID_F current,
  FLOAT_VOID_F position,
  FLOAT_VOID_F velocity);
  
//Control Functions
void mc_pid_current(void); //PID controller for current
void mc_set_target_current(float new_current);//sets the target_current of the pwm controller to new_current
MC_DATA* mc_get_parameters(void); //to set/update/read parameters
int mc_get_pwm(void);//returns the current pwm 
void mc_set_pwm(int pwm); //sets the pwm
void mc_run_no_control(void); //checks current and feeds watchdog; for debugging
void mc_set_error_level(unsigned long int error_level); //sets the error reporting level

//Safety Functions
static fixed mc_get_thermal_limiter(fixed current);
static fixed mc_smart_check_current(fixed current);
void mc_direction_control(MOTOR_DIRECTION dir, //the direction being controlled, either positive or negative
	MOTOR_CONTROL command); //controls whether the motor is allowed to turn in a given direction (MC_STOP or MC_START)
void mc_set_sleep(int sleep); //sets pwm to 0 if sleep
void mc_set_shutdown(int shutdown); //doesn't update watchdog timer if shutdown
static int mc_is_running(void); //returns true (1) if the motor controller is running, 0 otherwise
void mc_set_unsafe_target_current(float new_current);
float mc_get_mult(void);
static void mc_turnoff(MC_SAFETY_CAUSE reason);
static void mc_turnon(MC_SAFETY_CAUSE reason);

//Compliant control stuff
void mc_compliant_control(void);
void mc_set_command_current(float new_command);
void mc_set_stiffness(float new_c1);
void mc_set_dampness(float new_c2);
float mc_get_stiffness(void);
float mc_get_dampness(void);
float mc_get_command_current(void);

//Private Functions
static void mc_set_target_current_fixed(fixed new_current);
void mc_update_watchdog(void);//feeds the watchdog. Call this every ms or so
static void mc_check_current(fixed current); //checks the motor current; if the current is out of bounds for too long, calls mc_shutdown

/* HARDWARE SETUP
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
*/

#endif // MOTOR_CONTROLLER_H

