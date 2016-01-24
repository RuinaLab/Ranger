#ifndef __RANGERMATH_H__
#define __RANGERMATH_H__

#include <stdbool.h>

extern const float PI;
extern const float TWO_PI;
extern const float SQRT_TWO;
extern const float INV_TWO_PI;
extern const float HALF_PI;
extern const float DEG_TO_RAD;

extern unsigned int FAST_RAND_IDX;

float LinInterpVar(float x, float* X, float*Y, int nGrid); // Linear interpolation over a variable-spacing data set

float Sin(float);
float Cos(float);
float Tan(float);
float Atan(float);
float Tanh(float);
float Sqrt(float);
float Abs(float x);
float Fmod(float x, float den);
float Clamp(float x, float min, float max);
float FastRand(void);
float Mean(float* X, int n);

float SquareWave(float time, float period, float min, float max);
float SineWave(float time, float period, float min, float max);
float TriangleWave(float time, float period, float min, float max);
float SawToothWave(float time, float period, float min, float max);
bool PulseWave(float time, float period);

#endif // __RANGERMATH_H__
