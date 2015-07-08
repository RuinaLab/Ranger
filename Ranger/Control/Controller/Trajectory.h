#ifndef __TRAJECTORY_H__
#define __TRAJECTORY_H__

typedef struct coeff5{
	float t0, t1, a0, a1, a2, a3, a4, a5;
} poly_coeff;

poly_coeff* data_to_coeff(const float(*data)[4], int row);
int getIndex(float t, poly_coeff COEFFS[], int length);
poly_coeff trajCoeff(float y0, float y1, float yd0, float yd1, float ydd0, float ydd1);
float getY(poly_coeff c, float phi);
float getYd(poly_coeff c, float phi);
float getYdd(poly_coeff c, float phi);		

#endif  // __TRAJECTORY_H__

