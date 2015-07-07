typedef struct coeff5{
	float a0, a1, a2, a3, a4, a5;
} poly_coeff;

typedef struct data_input{
	float phi, t0, t1, y0, y1, yd0, yd1, ydd0, ydd1;
} input;

input* trajInterp(float t,  const float (*data)[4], int row);
poly_coeff trajCoeff(float y0, float y1, float yd0, float yd1, float ydd0, float ydd1);
float getY(poly_coeff c, float phi);
float getYd(poly_coeff c, float phi);
float getYdd(poly_coeff c, float phi);