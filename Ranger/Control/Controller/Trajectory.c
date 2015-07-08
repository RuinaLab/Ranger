#include "stdio.h"
#include "stdlib.h"
#include "Trajectory.h"

//Converts a 2-D array of data regarding position/slope/curvature to 
//an array of structs poly_coeff.
//Struct poly_coeff contains two times (starting & finishing time)
//that specifies a time interval and the 6 coefficients of the 
//5th order polynomial that lies in this time interval.  
poly_coeff* data_to_coeff(const float(*data)[4], int row){
	//dynamically allocating 2D array
	/*int col = 7; //time & the 6 coefficients for the polynomial
	float *COEFFS[col]; 
	for(int i=0; i<row; i++){
		COEFFS[i] = (float *)malloc(col*sizeof(float));
	}*/

	//the size of the coefficients array should be one less than the # of rows in the data array
	//every two adjacent rows of data output a single set of coefficients 
	//b/c a polynomial euqation needs to be calculatd for for each time interval in a piece-wise plot
	poly_coeff *COEFFS = (poly_coeff *) malloc ((row-1)*sizeof(poly_coeff));

	float t0, y0, yd0, ydd0, t1, y1, yd1, ydd1;
	poly_coeff c;

	int i;
	for(i=0; i<row-1; i++){	
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

//Finds the index of an element in the COEFFS array that
//contains time t
//Returns -1 if t is less than min time in the array
//Returns -2 if t is greater than max time in the array  
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

//Computes the set of coefficients of a 5th order polynomial given the information about 
//position/splope/curvature.
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
