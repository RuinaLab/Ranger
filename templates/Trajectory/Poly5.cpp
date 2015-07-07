// Poly5.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <stdio.h>
#include <stdlib.h>
#include "TrajData.h"
#include "Trajectory.h"

#define DATA TRAJ_DATA_Test2

int _tmain(int argc, _TCHAR* argv[])
{	int i = 0;
	FILE *out;
	out = fopen("poly5test.m", "w+");
	fprintf(out, "TestData = [...\n");
	fclose(out);

	int row = sizeof(DATA)/sizeof(DATA[0][0])/4;
	printf("rowDATA = %d\n", row);
	
	poly_coeff *COEFFS = data_to_coeff(DATA, row);
	/*printf("COEFFS table\n");
	for(int i=0; i<row-1; i++){
		printf("%f %f %f %f\n", COEFFS[i].t0, COEFFS[i].t1, COEFFS[i].a0, COEFFS[i].a1);
	}*/

	out = fopen("poly5test.m", "a");

	float T[] = {0.0, 5.0};
	float dt = 1.0/100;
	float t = 0.0;
	float y, yd, ydd;
	float phi;
	poly_coeff c;
	while(t<=T[1]){
		int index = getIndex(t, COEFFS, row-1); //length of the the COEFFS array is row-1
		if(index == -1){
			//input time less than the time interval
			index = 0;
			c = COEFFS[index];
			phi = 0;
		}else if(index == -2){
			//input time greater than the time interval
			index = row-2; 
			c = COEFFS[index];
			phi = 1;
		}else{
			//found the time interval, evaluate the polynomial at time t
			c = COEFFS[index];
			phi = (t-c.t0)/(c.t1-c.t0);
		}
		
		y = getY(c, phi);
		yd = getYd(c, phi);
		ydd = getYdd(c, phi);
		fprintf(out, "%f %f %f %f;\n", t, y, yd, ydd);
		
		t = t + dt;
	}

	fprintf(out, "];\n");
	fprintf(out,"subplot(3,1,1);\n");
	fprintf(out, "plot(TestData(:,1),TestData(:,2));\n");
	fprintf(out,"subplot(3,1,2);\n");
	fprintf(out, "plot(TestData(:,1),TestData(:,3));\n");
	fprintf(out,"subplot(3,1,3);\n");
	fprintf(out, "plot(TestData(:,1),TestData(:,4));\n");
	fclose(out);

	while(1);
	return 0;
}


