// Poly5.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include "TrajData.h"
#include "Trajectory.h"

#define DATA TRAJ_DATA_Test2

int _tmain(int argc, _TCHAR* argv[])
{	int i = 0;
	FILE *out;
	out = fopen("poly5test.m", "w+");
	fprintf(out, "TestData = [...\n");
	fclose(out);

	int DATA_row = sizeof(DATA)/sizeof(DATA[0][0])/4;
	printf("rowDATA = %d\n", DATA_row);
	
	float T[] = {0.0, 5.0};
	float dt = 1.0/100;
	printf("dt is %f\n", dt);
	float t = 0.0;
	float y, yd, ydd;
	out = fopen("poly5test.m", "a");
	while(t<=T[1]){
		input* in = trajInterp(t, DATA, DATA_row);
		if(in!=NULL){
			input in_data = *in;
			poly_coeff c = trajCoeff(in_data.y0, in_data.y1, in_data.yd0, in_data.yd1, in_data.ydd0, in_data.ydd1);
			//printf("phi in main is %f\n", in_data.phi);
			y = getY(c, in_data.phi);
			yd = getYd(c, in_data.phi);
			ydd = getYdd(c, in_data.phi);
			fprintf(out, "%f %f %f %f;\n", t, y, yd, ydd);	
		}else{
			printf("not found\n");
		}
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


