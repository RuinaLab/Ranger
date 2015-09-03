/*

	@file utility.h
	
	@author Nicolas Williamson
	
*/

#ifndef __UTILITY_H__
#define __UTILITY_H__

// ***** FIXED POINT ***** //
/**
  Fixed point numbers are stored in 32 bits with the bits defined as follows:
  - Bits 0:15 = Decimal Value
  - Bits 16:30 = Integral Value
  - Bit 31 = Sign bit, according to 2's Complement
  @note Fixed point numbers are @c signed.
*/
typedef signed long int fixed; 
#define FIXED_ONE (0x01<<16) /**< A 1 in fixed point. */
#define FIXED_MAX ((fixed)0x7FFFFFFF) /**< The maximum positive fixed point value. */
#define FIXED_FLOAT (1.0/FIXED_ONE) /**< Conversion multiplier from fixed to floating point. */
//Fixed Point Functions
fixed float_to_fixed(float m); 
fixed int_to_fixed(int m);
float fixed_to_float(fixed m);
int fixed_to_int(fixed m);
fixed fixed_mult(fixed a, fixed b);
fixed fixed_div(fixed a, fixed b);
long long int fixed_mult_to_long(fixed a, fixed b);
fixed linear_to_fixed(int offset, float coeff, int value);
fixed fixed_abs(fixed f);

typedef void(*VOID_VOID_F)(void); /**< Pointer to a function which takes void parameter and returns void. */
typedef void(*VOID_INT_F)(int); /**< Pointer to a function which takes int parameter and returns void. */
typedef int(*INT_VOID_F)(void); /**< Pointer to a function which takes void parameter and returns an int. */
typedef float(*FLOAT_VOID_F)(void);  /**< Pointer to a function which takes void parameter and returns a float. */
typedef fixed(*FIXED_VOID_F)(void);  /**< Pointer to a function which takes void parameter and returns a fixed. */

void voidvoid(void);
void voidint(int a);
int intvoid(void);
float floatvoid(void);
void voidvoid_noerror(void);
void voidint_noerror(int a);
int intvoid_noerror(void);
float floatvoid_noerror(void);

/***** TIC TOC *****/
void tic_toc_init(VOID_INT_F print_f);
void tic(void);
void toc(void);

#endif

