#include "stdafx.h"
#include "stdio.h"
#include "math.h"
#include "Trajectory.h"

void trajEval(const float data[2][4]){
	float a5, a4, a3, a2, a1, a0;
	float Y1, Y2, Y3;
	float y0, y1, yd0, yd1, ydd0, ydd1;
	y0 = data[0][1];
	yd0 = data[0][2];
	ydd0 = data[0][3];
	y1 = data[1][1];
	yd1 = data[1][2];
	ydd1 = data[1][3];
	
	a0 = y0;
	a1 = yd0;
	a2 = ydd0/2;
	Y1 = y1 - a0 - a1 - a2;
	Y2 = yd1 - a1 - 2*a2;
	Y3 = ydd1 - 2*a2;
	a3 = 10*Y1 - 4*Y2 + Y3/2;
	a4 = 5*Y1 - Y2 - 2*a3;
	a5 = Y1 - a4 - a3;

	FILE *out;
	out = fopen("poly5test.m", "w+");
	fprintf(out, "TestData = [...\n");
	float T[] = {0.0, 1.0};
	float y, yd, ydd;
	int n = 100;
	float dt = (T[1]-T[0])/n;
	float t = 0;
	while(t<=T[1]){
		y = a5*pow(t,5) + a4*pow(t,4) + a3*pow(t,3) + a2*pow(t,2) + a1*t + a0;
		yd = 5*a5*pow(t,4) + 4*a4*pow(t,3) + 3*a3*pow(t,2) + 2*a2*t + a1;
		ydd = 20*a5*pow(t,3) + 12*a4*pow(t,2) + 6*a3*t + 2*a2;
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

	return;
}