/**

	@file utility.c
	
	Lots of useful board independent functions for use in any module,
	such as conversions from float to int (bits, not number), and fixed point
	number functions.
	
	@author Nicolas Williamson 
  @date January 2010
	
*/

#include <includes.h>

#ifndef __VERSION_0_1__
#warning RangerOS mismatch, expected v0.1. 
#endif

// ***** Fixed Point ***** //
/**
  Changes from floating point to fixed point number.
  @param m A floating point number.
  @return A fixed point representation of the given floating point number.
*/
fixed float_to_fixed(float m){
  return ((fixed)((m) * FIXED_ONE));
}
/**
  Changes from an int to fixed point number.
  @param m An integer.
  @return A fixed point representation of the given @c int.
*/
fixed int_to_fixed(int m){
  return ((fixed)(m<<16));
}
/**
  Changes from fixed point to a floating point number.
  @param m A fixed point number.
  @return A floating point representation of the given fixed point number.
*/
float fixed_to_float(fixed m){
  return ((float)m*FIXED_FLOAT);
}
/**
  Changes from fixed point to integer.
  Doesn't round, only drops the decimal.
  @param m A fixed point number.
  @return An integer representation of the given floating point number.
*/
int fixed_to_int(fixed m){
  return (((fixed)(m))>>16);
}
/**
  Multiplies two fixed point numbers.
  @param a The first fixed point number.
  @param b The second fixed point number.
  @return The fixed point result of the multiplication of the two given fixed point numbers.
*/
fixed fixed_mult(fixed a, fixed b){
  return ((fixed)((((long long int)a)*((long long int)b))>>16));
}
/**
  Multiplies two fixed point numbers as a @c long @c long @c int 
  to hold the overflow.
  @param a The first fixed point number.
  @param b The second fixed point number.
  @return The fixed point result of the multiplication of the two given fixed point
  numbers stored in a long long int to hold any overflow.
*/
long long int fixed_mult_to_long(fixed a, fixed b){
  return (((((long long int)a)*((long long int)b))>>16));
}
/**
  Calculates the result of plugging in @c value into the linear equation 
  described by the @c offset and @c coeff as a fixed point value.
  Used in converting between units.
  @code retval = (coeff * value) + offset @endcode
  @param offset The offset of the linear equation. Often called @c b.
  @param coeff The coefficient of the linear equation. Often called @c m.
  @param value The value to be plugged into the linear equation. Often called @c x.
  @return The fixed point result of plugging @c value into the linear equation.
*/
fixed linear_to_fixed(int offset, float coeff, int value){
  int a, b, c, d, e;
  a = value;
  b = a - offset; //subtract the offset
  c = -1 * int_to_fixed(b); //convert to fixed point
  d = float_to_fixed(coeff); //convert coefficient to fixed point
  e = fixed_mult(c,d); //multiply gain to get amps in fixed point
  return e;
}
/**
  Returns the absolute value of the given fixed point number.
  @param f The fixed point value.
  @return The absolute value of the given fixed point number.
*/
fixed fixed_abs(fixed f){
  if (f < 0){return -f;}
  else{return f;}
}


// ***** Dummy Functions ***** //
/**
  A dummy function with @c void inputs that returns @c void.
  Use this as the default for function pointers.
*/
void voidvoid(void){error_occurred(ERROR_UTIL_DUMMY);} 
/** Doesn't throw an error if called. @see voidvoid */
void voidvoid_noerror(void){;} 
/**
  A dummy function with an @c int input that returns @c void.
  Use this as the default for function pointers.
  @param a Dummy integer.
*/
void voidint(int a){error_occurred(ERROR_UTIL_DUMMY);}
/** Doesn't throw an error if called. @see voidint */
void voidint_noerror(int a){;}
/**
  A dummy function with @c void inputs that returns a @c float.
  Use this as the default for function pointers.
*/
float floatvoid(void){error_occurred(ERROR_UTIL_DUMMY);return 0.0;}
/** Doesn't throw an error if called. @see floatvoid */
float floatvoid_noerror(void){return 0.0;}
/**
  A dummy function with @c void inputs that returns a @c int.
  Use this as the default for function pointers.
*/
int intvoid(void){error_occurred(ERROR_UTIL_DUMMY);return 0;}
/** Doesn't throw an error if called. @see intvoid */
int intvoid_noerror(void){return 0;}

// ***** TicToc Functions ***** //
int tt_time = 0; /**< The time it took for the last code block to execute. */
VOID_INT_F tt_print = voidint; /**< A print or transmit function for the execution time. */
/**
  Initializes the TicToc functionality.
  @note Must call this before using @ref tic or @ref toc.
  @param print_f A function which takes an int and returns void that will be
  used to print out the execution time.
*/
void tic_toc_init(VOID_INT_F print_f){
  tt_print = print_f;
}
/**
  Starts the clock. Put this function call right before the block of
  code for which you wish to know the execution time.
*/
void tic(void){
  tt_time = T0TC;
}
/**
  Stops the clock and prints out the execution time.
  Call this function right after the block of code
  for which you wish to know the execution time.
  Will use the print function passed at initialization.
*/
void toc(void){
  tt_time = (T0TC - tt_time);
  tt_print(tt_time);
  tt_time = 0;
}

