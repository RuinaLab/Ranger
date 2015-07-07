typedef struct coeff5{
	float a0, a1, a2, a3, a4, a5;
} poly_coeff;

void trajInterp(float t,  const float (*data)[4], int row);
poly_coeff trajCoeff(float y0, float y1, float yd0, float yd1, float ydd0, float ydd1);
void trajEval(poly_coeff c, float phi, float real_t);