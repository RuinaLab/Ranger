/*

	utility.h
	
	Nicolas Williamson
	
*/

#ifndef __UTILITY_H__
#define __UTILITY_H__

/***** FIXED POINT *****/
typedef signed long int fixed;
#define FIXED_ONE (0x01<<16)
#define FIXED_MAX ((fixed)0x7FFFFFFF)
#define FIXED_FLOAT (1.0/FIXED_ONE) //so that this is precomputed
fixed float_to_fixed(float m);
fixed int_to_fixed(int m);
float fixed_to_float(fixed m);
int fixed_to_int(fixed m);
fixed fixed_mult(fixed a, fixed b);
fixed fixed_div(fixed a, fixed b);
long long int fixed_mult_to_long(fixed a, fixed b);
fixed linear_to_fixed(int offset, float coeff, int value);
fixed fixed_abs(fixed f);

/***** EMPTY FUNCTIONS *****/
typedef void(*VOID_VOID_F)(void);
typedef void(*VOID_INT_F)(int);
typedef int(*INT_VOID_F)(void);
typedef float(*FLOAT_VOID_F)(void);
typedef fixed(*FIXED_VOID_F)(void);
void voidvoid(void);
void voidint(int a);
int intvoid(void);
float floatvoid(void);

/***** TIC TOC *****/
void tic(void);
void toc(void);

#endif

