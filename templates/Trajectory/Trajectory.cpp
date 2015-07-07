#include "stdafx.h"
#include "stdio.h"
#include "math.h"
#include "Trajectory.h"


input* trajInterp(float t, const float (*data)[4], int row){
	int k = 0;
	int found = 0;
	while(k<row && !found){
		if(data[k][0]>=t){
			found = 1;
		}else{
			k++;
		}
	}

	input d;
	if(!found){
		printf("not found");
		return NULL;
	}else{
		if(k==0){
			d.t0 = data[0][0]; 
			d.y0 = data[0][1];
			d.yd0 = data[0][2];
			d.ydd0 = data[0][3];
			d.t1 = data[1][0];
			d.y1 = data[1][1];
			d.yd1 = data[1][2];
			d.ydd1 = data[1][3];
			d.phi = (t-d.t0)/(d.t1-d.t0);
		}else{
			d.t0 = data[k-1][0]; 
			d.y0 = data[k-1][1];
			d.yd0 = data[k-1][2];
			d.ydd0 = data[k-1][3];
			d.t1 = data[k][0];
			d.y1 = data[k][1];
			d.yd1 = data[k][2];
			d.ydd1 = data[k][3];
			d.phi = (t-d.t0)/(d.t1-d.t0);
		}
	}
	
	return &d;
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
}


float getY(poly_coeff c, float phi){
	float y;
	y = c.a5*pow(phi,5) + c.a4*pow(phi,4) + c.a3*pow(phi,3) + c.a2*pow(phi,2) + c.a1*phi + c.a0;
	return y;
}

float getYd(poly_coeff c, float phi){
	float yd;
	yd = 5*c.a5*pow(phi,4) + 4*c.a4*pow(phi,3) + 3*c.a3*pow(phi,2) + 2*c.a2*phi + c.a1;
	return yd;
}

float getYdd(poly_coeff c, float phi){
	float ydd;
	ydd = 20*c.a5*pow(phi,3) + 12*c.a4*pow(phi,2) + 6*c.a3*phi + 2*c.a2;
	return ydd;
}