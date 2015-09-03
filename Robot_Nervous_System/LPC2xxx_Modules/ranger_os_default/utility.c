/*

	utility.c
	
	Lots of useful board independent functions for use from anywhere,
	such as conversions from float to int (bits, not number), and fixed point
	number functions.
	
	Nicolas Williamson - January 2010
	
*/

#include <includes.h>

/************* FIXED POINT ******************************************/
fixed float_to_fixed(float m){
  return ((fixed)((m) * FIXED_ONE));
}
fixed int_to_fixed(int m){
  return ((fixed)(m<<16));
}
float fixed_to_float(fixed m){
  return ((float)m*FIXED_FLOAT);
}
int fixed_to_int(fixed m){
  return (((fixed)(m))>>16);
}
fixed fixed_mult(fixed a, fixed b){
  return ((fixed)((((long long int)a)*((long long int)b))>>16));
}
long long int fixed_mult_to_long(fixed a, fixed b){
  return (((((long long int)a)*((long long int)b))>>16));
}
fixed linear_to_fixed(int offset, float coeff, int value){
	int a, b, c, d, e;
  a = value;
  b = a - offset; //subtract the offset
  c = -1 * int_to_fixed(b); //convert to fixed point
	d = float_to_fixed(coeff); //convert coefficient to fixed point
	e = fixed_mult(c,d); //multiply gain to get amps in fixed point
	return e;
}
fixed fixed_abs(fixed f){
  if (f < 0){return -f;}
  else{return f;}
}


/************* EMPTY FUNCTIONS ******************************************/
void voidvoid(void){error_occurred(ERROR_UTIL_DUMMY);}
void voidint(int a){error_occurred(ERROR_UTIL_DUMMY);}
float floatvoid(void){error_occurred(ERROR_UTIL_DUMMY);return 0.0;}
int intvoid(void){error_occurred(ERROR_UTIL_DUMMY);return 0;}

/************* TIC TOC ******************************************/
char tt_string[20];
int tt_str_length;
int tt_time = 0;
void tic(void){
  tt_time = T0TC;
}

void toc(void){
  tt_time = (T0TC - tt_time);
//  tt_str_length = sprintf(tt_string,"%i\n\r", tt_time);
//  uarti_tx_buf(tt_string, tt_str_length);
  tt_time = 0;
}

