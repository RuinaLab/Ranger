#include "stdafx.h"
#include "stdio.h"
#include "math.h"
#include "Trajectory.h"


void trajInterp(float t, const float (*data)[4], int row){
	int k = 0;
	int found = 0;
	while(k<row && !found){
		if(data[k][0]>=t){
			found = 1;
		}
		else{
			k++;
		}
	}

	if(found){
		float phi, t0, t1, y0, y1, yd0, yd1, ydd0, ydd1;
		
		if(k==0){
			t0 = data[0][0]; 
			y0 = data[0][1];
			yd0 = data[0][2];
			ydd0 = data[0][3];
			t1 = data[1][0];
			y1 = data[1][1];
			yd1 = data[1][2];
			ydd1 = data[1][3];
			phi = (t-t0)/(t1-t0);
			poly_coeff c = trajCoeff(y0, y1, yd0, yd1, ydd0, ydd1);
			trajEval(c, phi, t);
			/*FILE *out;
			out = fopen("poly5test.m", "a");
			fprintf(out, "%f %f %f %f;\n", t, y1, yd1, ydd1);
			fclose(out);*/
		}else{
			t0 = data[k-1][0]; 
			y0 = data[k-1][1];
			yd0 = data[k-1][2];
			ydd0 = data[k-1][3];
			t1 = data[k][0];
			y1 = data[k][1];
			yd1 = data[k][2];
			ydd1 = data[k][3];
			phi = (t-t0)/(t1-t0);
			poly_coeff c = trajCoeff(y0, y1, yd0, yd1, ydd0, ydd1);
			trajEval(c, phi, t);
		}
	}
	return;
}

poly_coeff trajCoeff(float y0, float y1, float yd0, float yd1, float ydd0, float ydd1){
	poly_coeff c;
	float Y1, Y2, Y3;
	
	c.a0 = y0;
	c.a1 = yd0;
	c.a2 = ydd0/2;
	Y1 = y1 - c.a0 - c.a1 - c.a2;
	Y2 = yd1 - c.a1 - 2*(c.a2);
	Y3 = ydd1 - 2*(c.a2);
	c.a3 = 10*Y1 - 4*Y2 + Y3/2;
	c.a4 = 5*Y1 - Y2 - 2*(c.a3);
	c.a5 = Y1 - c.a4 - c.a3;

	return c; 
	// works for interval [0, 1]
	/*float y, yd, ydd;
	float T[] = {0.0, 1.0};
	int n = 100;
	float dt = (T[1]-T[0])/n;
	float t = 0;
	while(t<=T[1]){
		y = a5*pow(t,5) + a4*pow(t,4) + a3*pow(t,3) + a2*pow(t,2) + a1*t + a0;
		yd = 5*a5*pow(t,4) + 4*a4*pow(t,3) + 3*a3*pow(t,2) + 2*a2*t + a1;
		ydd = 20*a5*pow(t,3) + 12*a4*pow(t,2) + 6*a3*t + 2*a2;
		fprintf(out, "%f %f %f %f;\n", t+t0, y, yd, ydd);
		t = t + dt;
	}*/
}

void trajEval(poly_coeff c, float phi, float real_t){
	FILE *out;
	out = fopen("poly5test.m", "a");
	float y, yd, ydd;
	y = c.a5*pow(phi,5) + c.a4*pow(phi,4) + c.a3*pow(phi,3) + c.a2*pow(phi,2) + c.a1*phi + c.a0;
	yd = 5*c.a5*pow(phi,4) + 4*c.a4*pow(phi,3) + 3*c.a3*pow(phi,2) + 2*c.a2*phi + c.a1;
	ydd = 20*c.a5*pow(phi,3) + 12*c.a4*pow(phi,2) + 6*c.a3*phi + 2*c.a2;
	fprintf(out, "%f %f %f %f;\n", real_t, y, yd, ydd);
	fclose(out);
	return;
}
