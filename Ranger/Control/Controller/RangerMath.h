#ifndef __RANGERMATH_H__
#define __RANGERMATH_H__

#include <stdbool.h>

extern const float PI;
extern const float TWO_PI;
extern const float SQRT_TWO;
extern const float INV_TWO_PI;
extern const float HALF_PI;

float Sin(float);
float Cos(float);
float Tan(float);
float Atan(float);
float Tanh(float);
float Sqrt(float);
float Fmod(float x, float den);
float Clamp(float x, float min, float max);

float SquareWave(float time, float period, float min, float max);
float SineWave(float time, float period, float min, float max);
float TriangleWave(float time, float period, float min, float max);

#endif // __RANGERMATH_H__
