#include "stdafx.h"
#include "stdio.h"
#include "stdlib.h"
//#include "math.h"
#include "Trajectory.h"

poly_coeff* data_to_coeff(const float(*data)[4], int row){
	//dynamically allocating 2D array
	/*int col = 7; //time & the 6 coefficients for the polynomial
	float *COEFFS[col]; 
	for(int i=0; i<row; i++){
		COEFFS[i] = (float *)malloc(col*sizeof(float));
	}*/

	poly_coeff *COEFFS = (poly_coeff *) malloc ((row-1)*sizeof(poly_coeff));

	float t0, y0, yd0, ydd0, t1, y1, yd1, ydd1;
	poly_coeff c;

	for(int i=0; i<row-1; i++){	
		t0 = data[i][0]; 
		y0 = data[i][1];
		yd0 = data[i][2];
		ydd0 = data[i][3];
		t1 = data[i+1][0];
		y1 = data[i+1][1];
		yd1 = data[i+1][2];
		ydd1 = data[i+1][3];
		c = trajCoeff(y0, y1, yd0, yd1, ydd0, ydd1);
		c.t0 = t0;
		c.t1 = t1;
		COEFFS[i] = c;
	}

	return COEFFS;
} 

int getIndex(float t, poly_coeff COEFFS[], int length){
	int k = 0;
	int found = 0;

	while(k<length && !found){
		if(COEFFS[k].t1>=t){
			found = 1;
		}else{
			k++;
		}
	}

	if(found){
		return k;
	}else{
		if(t < COEFFS[0].t0){
			//not found, t is less than min time in data
			return -1; 
		}else{
			//not found, t is greater than max time in data
			return -2;
		}
	}
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
	c.t0 = 0;
	c.t1 = 0;

	return c; 
}

//geting the position of a 5th order polymoniaml with coefficients c at the scaled time phi
float getY(poly_coeff c, float phi){
	float y;
	y = c.a0 + phi*(c.a1 + phi*(c.a2 + phi*(c.a3 + phi*(c.a4 + phi*(c.a5) ) ) ) );
	return y;
}

//geting the slope of a 5th order polymoniaml with coefficients c at the scaled time phi
float getYd(poly_coeff c, float phi){
	float yd;
	yd = c.a1 + phi*(2*c.a2 + phi*(3*c.a3 + phi*(4*c.a4 + phi*(5*c.a5) ) ) );
	return yd;
}

//geting the curvature of a 5th order polymoniaml with coefficients c at the scaled time phi
float getYdd(poly_coeff c, float phi){
	float ydd;
	ydd = 2*c.a2 + phi*(6*c.a3 + phi*(12*c.a4 + phi*(20*c.a5) ) );
	return ydd;
}



/*
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
*/