#ifndef __RANGERMATH_H__
#define __RANGERMATH_H__

/* Define a boolean data type */
typedef enum { false, true } bool;

extern const float PI;
extern const float TWO_PI;
extern const float SQRT_TWO;

float Sin(float);
float Cos(float);
float Tan(float);
float Atan(float);
float Tanh(float);
float Sqrt(float);
float Fmod(float,float);

#endif // __RANGERMATH_H__
